#include "gst_streamer.hpp"

#include <atomic>
#include <condition_variable>
#include <deque>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include <gst/app/gstappsrc.h>
#include <gst/gst.h>

namespace omr_vision::camera {

class GstStreamer::Impl {
public:
    Impl(int width, int height, int fps, int bitrate_kbps, std::string rtsp_url)
        : width_(width),
          height_(height),
          fps_(fps),
          bitrate_kbps_(bitrate_kbps),
          rtsp_url_(std::move(rtsp_url)) {
        if (!gst_is_initialized()) {
            gst_init(nullptr, nullptr);
        }
    }

    ~Impl() {
        try {
            stop();
        } catch (...) {  // NOLINT(bugprone-empty-catch)
            // Destructor must not throw; silently ignore teardown errors.
        }
    }

    Impl(const Impl&) = delete;
    auto operator=(const Impl&) -> Impl& = delete;
    Impl(Impl&&) noexcept = delete;
    auto operator=(Impl&&) noexcept -> Impl& = delete;

    auto start() -> void {
        if (running_) return;
        if (bitrate_kbps_ <= 0) {
            throw std::runtime_error(
                "GstStreamer: bitrate must be positive (got " +
                std::to_string(bitrate_kbps_) + ")");
        }

        // Build pipeline string
        std::ostringstream pipeline_str;
        pipeline_str << "appsrc name=src is-live=true block=false format=time ! "
                     << "videoconvert ! "
                     << "x264enc tune=zerolatency bitrate=" << bitrate_kbps_ << " ! "
                     << "h264parse ! "
                     << "rtspclientsink location=" << rtsp_url_;

        GError* error = nullptr;
        pipeline_ = gst_parse_launch(pipeline_str.str().c_str(), &error);
        if (error != nullptr) {
            std::string msg(error->message);
            g_error_free(error);
            throw std::runtime_error("GstStreamer: failed to parse pipeline: " + msg);
        }

        // Get appsrc element
        appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline_), "src");
        if (appsrc_ == nullptr) {
            gst_object_unref(pipeline_);
            pipeline_ = nullptr;
            throw std::runtime_error("GstStreamer: failed to get appsrc element 'src'");
        }

        // Set caps on appsrc: BGR format matching OpenCV default
        std::ostringstream caps_str;
        caps_str << "video/x-raw,format=BGR,width=" << width_ << ",height=" << height_
                 << ",framerate=" << fps_ << "/1";

        GstCaps* caps = gst_caps_from_string(caps_str.str().c_str());
        g_object_set(G_OBJECT(appsrc_), "caps", caps, nullptr);
        gst_caps_unref(caps);

        // Add bus watch for error logging
        bus_watch_id_ = gst_bus_add_watch(
            gst_pipeline_get_bus(GST_PIPELINE(pipeline_)), on_bus_message, this);

        // Set pipeline to PLAYING
        GstStateChangeReturn ret =
            gst_element_set_state(pipeline_, GST_STATE_PLAYING);
        if (ret == GST_STATE_CHANGE_FAILURE) {
            g_source_remove(bus_watch_id_);
            bus_watch_id_ = 0;
            gst_object_unref(appsrc_);
            appsrc_ = nullptr;
            gst_object_unref(pipeline_);
            pipeline_ = nullptr;
            throw std::runtime_error("GstStreamer: failed to set pipeline to PLAYING");
        }

        running_ = true;

        // Launch GMainLoop thread
        loop_ = g_main_loop_new(nullptr, FALSE);
        loop_thread_ = std::thread([this]() { g_main_loop_run(loop_); });

        // Launch push thread
        push_thread_ = std::thread(&Impl::push_thread_func, this);
    }

    auto stop() -> void {
        if (!running_) return;

        running_ = false;
        queue_cv_.notify_all();

        if (push_thread_.joinable()) {
            push_thread_.join();
        }

        if (appsrc_ != nullptr) {
            gst_app_src_end_of_stream(GST_APP_SRC(appsrc_));
        }

        if (loop_ != nullptr) {
            g_main_loop_quit(loop_);
        }
        if (loop_thread_.joinable()) {
            loop_thread_.join();
        }

        // Remove bus watch after loop thread stops — no more callbacks will fire
        if (bus_watch_id_ != 0) {
            g_source_remove(bus_watch_id_);
            bus_watch_id_ = 0;
        }

        if (pipeline_ != nullptr) {
            gst_element_set_state(pipeline_, GST_STATE_NULL);
            gst_object_unref(pipeline_);
            pipeline_ = nullptr;
        }

        if (appsrc_ != nullptr) {
            gst_object_unref(appsrc_);
            appsrc_ = nullptr;
        }

        if (loop_ != nullptr) {
            g_main_loop_unref(loop_);
            loop_ = nullptr;
        }
    }

    auto push(const cv::Mat& frame) -> void {
        if (!running_) return;
        std::lock_guard lock(queue_mutex_);
        if (queue_.size() >= kMaxQueueSize) {
            queue_.pop_front();  // Drop oldest
        }
        auto mat = std::make_shared<cv::Mat>(frame.clone());
        queue_.push_back(std::move(mat));
        queue_cv_.notify_one();
    }

private:
    static constexpr std::size_t kMaxQueueSize = 5;

    static auto on_bus_message(GstBus* /*bus*/, GstMessage* msg, gpointer user_data) -> gboolean {
        auto* self = static_cast<Impl*>(user_data);
        switch (GST_MESSAGE_TYPE(msg)) {
            case GST_MESSAGE_ERROR: {
                GError* err = nullptr;
                gchar* debug = nullptr;
                gst_message_parse_error(msg, &err, &debug);
                std::cerr << "GstStreamer error: " << err->message << "\n"
                          << "Debug: " << (debug != nullptr ? debug : "(none)") << std::endl;
                g_error_free(err);
                g_free(debug);
                break;
            }
            case GST_MESSAGE_WARNING: {
                GError* warn = nullptr;
                gchar* debug = nullptr;
                gst_message_parse_warning(msg, &warn, &debug);
                std::cerr << "GstStreamer warning: " << warn->message << "\n"
                          << "Debug: " << (debug != nullptr ? debug : "(none)") << std::endl;
                g_error_free(warn);
                g_free(debug);
                break;
            }
            case GST_MESSAGE_EOS:
                std::cerr << "GstStreamer: end of stream" << std::endl;
                break;
            default:
                break;
        }
        (void)self;  // Reserved for future use
        return TRUE;
    }

    auto push_one_frame(const std::shared_ptr<cv::Mat>& mat, int64_t& frame_index) -> void {
        // GstBuffer wraps cv::Mat data directly (zero-copy).
        // The shared_ptr in the free function keeps the data alive until
        // GStreamer releases the buffer.
        auto* holder = new std::shared_ptr<cv::Mat>(mat);
        GstBuffer* buffer = gst_buffer_new_wrapped_full(
            GST_MEMORY_FLAG_READONLY,
            mat->data,
            static_cast<gsize>(mat->total() * mat->elemSize()),
            0,
            static_cast<gsize>(mat->total() * mat->elemSize()),
            holder,
            [](gpointer d) { delete static_cast<std::shared_ptr<cv::Mat>*>(d); });

        if (buffer == nullptr) return;

        GST_BUFFER_PTS(buffer) =
            static_cast<GstClockTime>(frame_index) * GST_SECOND /
            static_cast<GstClockTime>(fps_);
        ++frame_index;

        gst_app_src_push_buffer(GST_APP_SRC(appsrc_), buffer);
    }

    auto push_thread_func() -> void {
        int64_t frame_index = 0;

        while (true) {
            {
                std::unique_lock lock(queue_mutex_);
                queue_cv_.wait(lock, [this] { return !queue_.empty() || !running_; });

                while (!queue_.empty()) {
                    auto mat = std::move(queue_.front());
                    queue_.pop_front();
                    lock.unlock();

                    push_one_frame(mat, frame_index);

                    lock.lock();
                }

                if (!running_) break;
            }
        }

        // Flush any remaining queued frames on shutdown
        std::lock_guard lock(queue_mutex_);
        while (!queue_.empty()) {
            auto mat = std::move(queue_.front());
            queue_.pop_front();
            push_one_frame(mat, frame_index);
        }
    }

    // Configuration
    int width_;
    int height_;
    int fps_;
    int bitrate_kbps_;
    std::string rtsp_url_;

    // GStreamer state
    GstElement* pipeline_ = nullptr;
    GstElement* appsrc_ = nullptr;
    GMainLoop* loop_ = nullptr;
    guint bus_watch_id_ = 0;

    // Threading
    std::thread loop_thread_;
    std::thread push_thread_;
    std::atomic<bool> running_{false};

    // Frame queue
    std::deque<std::shared_ptr<cv::Mat>> queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
};

// ── GstStreamer public API (thin wrappers around Impl) ──

GstStreamer::GstStreamer(int width, int height, int fps, int bitrate_kbps,
                         std::string rtsp_url)
    : impl_(std::make_unique<Impl>(width, height, fps, bitrate_kbps,
                                   std::move(rtsp_url))) {}

GstStreamer::~GstStreamer() = default;

auto GstStreamer::start() -> void { impl_->start(); }

auto GstStreamer::stop() -> void { impl_->stop(); }

auto GstStreamer::push(const cv::Mat& frame) -> void { impl_->push(frame); }

}  // namespace omr_vision::camera

#include "omr_lio/lio_node.hpp"

#include <cmath>
#include <csignal>
#include <omp.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <unistd.h>

#include <fstream>
#include <iostream>
#include <thread>

#include <Eigen/Core>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#define INIT_TIME (0.1)
#define LASER_POINT_COV (0.001)
#define PUBFRAME_PERIOD (20)
#ifndef ROOT_DIR
#define ROOT_DIR "./"
#endif

namespace omr_lio {

LioNode::LioNode(const rclcpp::NodeOptions& options) : rclcpp::Node("lio_node", options) {
    declare_params();
    load_params();

    // ── Publishers ──
    rclcpp::QoS qos_reliable(100);
    rclcpp::QoS qos_sensor = rclcpp::SensorDataQoS();

    pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("/lio/odom", qos_reliable);
    pub_cloud_registered_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", qos_reliable);
    pub_cloud_registered_body_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/cloud_registered_body", qos_reliable);
    pub_laser_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/laser_map", rclcpp::QoS(10).transient_local());
    pub_path_ = this->create_publisher<nav_msgs::msg::Path>("/lio_path", qos_reliable);

    // ── Subscribers ──
    sub_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        lid_topic_, qos_sensor,
        [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) { lidar_callback(msg); });
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, qos_sensor,
        [this](sensor_msgs::msg::Imu::SharedPtr msg) { imu_callback(msg); });

    // ── TF broadcaster ──
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    // ── Filters ──
    downSizeFilterSurf_.setLeafSize(filter_size_surf_min_, filter_size_surf_min_,
                                    filter_size_surf_min_);
    downSizeFilterMap_.setLeafSize(filter_size_map_min_, filter_size_map_min_,
                                   filter_size_map_min_);

    // ── IMU processor setup ──
    Lidar_T_wrt_IMU_ << VEC_FROM_ARRAY(extrinT_);
    Lidar_R_wrt_IMU_ << MAT_FROM_ARRAY(extrinR_);
    p_imu_->set_param(Lidar_T_wrt_IMU_, Lidar_R_wrt_IMU_, V3D(gyr_cov_, gyr_cov_, gyr_cov_),
                      V3D(acc_cov_, acc_cov_, acc_cov_), V3D(b_gyr_cov_, b_gyr_cov_, b_gyr_cov_),
                      V3D(b_acc_cov_, b_acc_cov_, b_acc_cov_));

    // ── Path header ──
    path_.header.stamp = this->now();
    path_.header.frame_id = "map";

    RCLCPP_INFO(this->get_logger(), "LioNode initialized. LiDAR type: %d", p_pre_->lidar_type);
    RCLCPP_INFO(this->get_logger(), "Topics: lidar=%s, imu=%s", lid_topic_.c_str(),
                imu_topic_.c_str());
}

LioNode::~LioNode() {
    flg_exit_ = true;
    sig_buffer_.notify_all();
    if (pcd_save_en_ && pcl_wait_save_->size() > 0) {
        save_map();
    }
}

// ── Config ──

void LioNode::declare_params() {
    // publish
    this->declare_parameter<bool>("publish.path_en", true);
    this->declare_parameter<bool>("publish.scan_publish_en", true);
    this->declare_parameter<bool>("publish.dense_publish_en", true);
    this->declare_parameter<bool>("publish.scan_bodyframe_pub_en", true);
    // common
    this->declare_parameter<std::string>("common.lid_topic", "/livox/lidar");
    this->declare_parameter<std::string>("common.imu_topic", "/livox/imu");
    this->declare_parameter<bool>("common.time_sync_en", false);
    this->declare_parameter<double>("common.time_offset_lidar_to_imu", 0.0);
    // mapping
    this->declare_parameter<int>("max_iteration", 4);
    this->declare_parameter<std::string>("map_file_path", "");
    this->declare_parameter<double>("filter_size_surf", 0.5);
    this->declare_parameter<double>("filter_size_map", 0.5);
    this->declare_parameter<double>("cube_side_length", 200.0);
    this->declare_parameter<double>("mapping.det_range", 300.0);
    this->declare_parameter<double>("mapping.fov_degree", 180.0);
    this->declare_parameter<double>("mapping.gyr_cov", 0.1);
    this->declare_parameter<double>("mapping.acc_cov", 0.1);
    this->declare_parameter<double>("mapping.b_gyr_cov", 0.0001);
    this->declare_parameter<double>("mapping.b_acc_cov", 0.0001);
    this->declare_parameter<bool>("mapping.extrinsic_est_en", true);
    this->declare_parameter<std::vector<double>>("mapping.extrinsic_T",
                                                 std::vector<double>{0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>(
        "mapping.extrinsic_R", std::vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
    // preprocess
    this->declare_parameter<double>("preprocess.blind", 0.01);
    this->declare_parameter<int>("preprocess.lidar_type", AVIA);
    this->declare_parameter<int>("preprocess.scan_line", 16);
    this->declare_parameter<int>("preprocess.timestamp_unit", US);
    this->declare_parameter<int>("preprocess.scan_rate", 10);
    this->declare_parameter<int>("point_filter_num", 2);
    this->declare_parameter<bool>("feature_extract_enable", false);
    // pcd_save
    this->declare_parameter<bool>("pcd_save.pcd_save_en", false);
    this->declare_parameter<int>("pcd_save.interval", -1);
}

void LioNode::load_params() {
    this->get_parameter<bool>("publish.path_en", path_en_);
    this->get_parameter<bool>("publish.scan_publish_en", scan_pub_en_);
    this->get_parameter<bool>("publish.dense_publish_en", dense_pub_en_);
    this->get_parameter<bool>("publish.scan_bodyframe_pub_en", scan_body_pub_en_);
    this->get_parameter<std::string>("common.lid_topic", lid_topic_);
    this->get_parameter<std::string>("common.imu_topic", imu_topic_);
    this->get_parameter<bool>("common.time_sync_en", time_sync_en_);
    this->get_parameter<double>("common.time_offset_lidar_to_imu", time_diff_lidar_to_imu_);
    this->get_parameter<int>("max_iteration", num_max_iterations_);
    this->get_parameter<std::string>("map_file_path", map_file_path_);
    this->get_parameter<double>("filter_size_surf", filter_size_surf_min_);
    this->get_parameter<double>("filter_size_map", filter_size_map_min_);
    this->get_parameter<double>("cube_side_length", cube_len_);
    this->get_parameter<double>("mapping.det_range", det_range_);
    this->get_parameter<double>("mapping.fov_degree", fov_deg_);
    this->get_parameter<double>("mapping.gyr_cov", gyr_cov_);
    this->get_parameter<double>("mapping.acc_cov", acc_cov_);
    this->get_parameter<double>("mapping.b_gyr_cov", b_gyr_cov_);
    this->get_parameter<double>("mapping.b_acc_cov", b_acc_cov_);
    this->get_parameter<bool>("mapping.extrinsic_est_en", extrinsic_est_en_);
    this->get_parameter<std::vector<double>>("mapping.extrinsic_T", extrinT_);
    this->get_parameter<std::vector<double>>("mapping.extrinsic_R", extrinR_);
    this->get_parameter<double>("preprocess.blind", p_pre_->blind);
    this->get_parameter<int>("preprocess.lidar_type", p_pre_->lidar_type);
    this->get_parameter<int>("preprocess.scan_line", p_pre_->N_SCANS);
    this->get_parameter<int>("preprocess.timestamp_unit", p_pre_->time_unit);
    this->get_parameter<int>("preprocess.scan_rate", p_pre_->SCAN_RATE);
    this->get_parameter<int>("point_filter_num", p_pre_->point_filter_num);
    this->get_parameter<bool>("feature_extract_enable", p_pre_->feature_enabled);
    this->get_parameter<bool>("pcd_save.pcd_save_en", pcd_save_en_);
    this->get_parameter<int>("pcd_save.interval", pcd_save_interval_);
}

// ── Callbacks ──

void LioNode::lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_buffer_);
    scan_count_++;
    double msg_sec = rclcpp::Time(msg->header.stamp).seconds();
    if (msg_sec < last_timestamp_lidar_) {
        RCLCPP_ERROR(this->get_logger(), "lidar loop back, clear buffer");
        lidar_buffer_.clear();
    }

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre_->process(msg, ptr);
    lidar_buffer_.push_back(ptr);
    time_buffer_.push_back(msg_sec);
    last_timestamp_lidar_ = msg_sec;
    sig_buffer_.notify_all();
}

void LioNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg_in) {
    publish_count_++;
    sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));

    double msg_sec = rclcpp::Time(msg_in->header.stamp).seconds();

    if (std::abs(timediff_lidar_wrt_imu_) > 0.1 && time_sync_en_) {
        double corrected_sec = timediff_lidar_wrt_imu_ + msg_sec;
        uint32_t sec = static_cast<uint32_t>(corrected_sec);
        uint32_t nsec = static_cast<uint32_t>((corrected_sec - sec) * 1e9);
        msg->header.stamp.sec = sec;
        msg->header.stamp.nanosec = nsec;
    }

    double corrected_sec = msg_sec - time_diff_lidar_to_imu_;
    uint32_t sec = static_cast<uint32_t>(corrected_sec);
    uint32_t nsec = static_cast<uint32_t>((corrected_sec - sec) * 1e9);
    msg->header.stamp.sec = sec;
    msg->header.stamp.nanosec = nsec;

    double timestamp = corrected_sec;

    std::lock_guard<std::mutex> lock(mtx_buffer_);

    if (timestamp < last_timestamp_imu_) {
        RCLCPP_WARN(this->get_logger(), "imu loop back, clear buffer");
        imu_buffer_.clear();
    }

    last_timestamp_imu_ = timestamp;
    imu_buffer_.push_back(msg);
    sig_buffer_.notify_all();
}

// ── Main processing loop ──

void LioNode::run() {
    rclcpp::Rate rate(5000);
    RCLCPP_INFO(this->get_logger(), "LIO processing loop started");

    while (rclcpp::ok() && !flg_exit_) {
        rclcpp::spin_some(this->get_node_base_interface());

        if (sync_packages(Measures_)) {
            double t00 = omp_get_wtime();

            if (flg_first_scan_) {
                first_lidar_time_ = Measures_.lidar_beg_time;
                p_imu_->first_lidar_time = first_lidar_time_;
                flg_first_scan_ = false;
                continue;
            }

            if (time_sync_en_ && !timediff_set_flg_ &&
                std::abs(last_timestamp_lidar_ - last_timestamp_imu_) > 1 && !imu_buffer_.empty()) {
                timediff_set_flg_ = true;
                timediff_lidar_wrt_imu_ = last_timestamp_lidar_ + 0.1 - last_timestamp_imu_;
                printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu_);
            }

            p_imu_->Process(Measures_, kf_, feats_undistort_);

            if (feats_undistort_->empty() || feats_undistort_ == nullptr) {
                RCLCPP_WARN(this->get_logger(), "No point, skip this scan!");
                continue;
            }

            state_point_ = kf_.get_x();
            pos_lid_ = state_point_.pos + state_point_.rot.matrix() * state_point_.offset_T_L_I;

            flg_EKF_inited_ =
                (Measures_.lidar_beg_time - first_lidar_time_) < INIT_TIME ? false : true;

            lasermap_fov_segment();

            downSizeFilterSurf_.setInputCloud(feats_undistort_);
            downSizeFilterSurf_.filter(*feats_down_body_);
            feats_down_size_ = feats_down_body_->points.size();

            if (feats_down_size_ < 5) {
                RCLCPP_WARN(this->get_logger(), "No point, skip this scan!");
                continue;
            }

            if (ikdtree_.Root_Node == nullptr) {
                ikdtree_.set_downsample_param(filter_size_map_min_);
                feats_down_world_->resize(feats_down_size_);
                for (int i = 0; i < feats_down_size_; i++) {
                    pointBodyToWorld(&(feats_down_body_->points[i]),
                                     &(feats_down_world_->points[i]));
                }
                ikdtree_.Build(feats_down_world_->points);
                continue;
            }

            if (0) {  // if(1) to see map points
                PointVector().swap(ikdtree_.PCL_Storage);
                ikdtree_.flatten(ikdtree_.Root_Node, ikdtree_.PCL_Storage, NOT_RECORD);
                featsFromMap_->clear();
                featsFromMap_->points = ikdtree_.PCL_Storage;
            }

            /*** iterated state estimation ***/
            Nearest_Points_.resize(feats_down_size_);
            kf_.update_iterated_dyn_share_modified(LASER_POINT_COV, feats_down_body_, ikdtree_,
                                                   Nearest_Points_, num_max_iterations_,
                                                   extrinsic_est_en_);

            state_point_ = kf_.get_x();
            pos_lid_ = state_point_.pos + state_point_.rot.matrix() * state_point_.offset_T_L_I;

            /******* Publish odometry *******/
            publish_odometry();
            publish_tf();

            /*** add the feature points to map kdtree ***/
            feats_down_world_->resize(feats_down_size_);
            map_incremental();

            /******* Publish points *******/
            if (path_en_) publish_path();
            if (scan_pub_en_ || pcd_save_en_) publish_cloud();
            if (scan_pub_en_ && scan_body_pub_en_) publish_frame_body();

            double t11 = omp_get_wtime();
            RCLCPP_DEBUG(this->get_logger(), "feats_down_size: %d  mapping time: %.2f ms",
                         feats_down_size_, (t11 - t00) * 1000);
        }

        rate.sleep();
    }

    RCLCPP_INFO(this->get_logger(), "LIO processing loop stopped");
}

// ── Data synchronization ──

bool LioNode::sync_packages(MeasureGroup& meas) {
    if (lidar_buffer_.empty() || imu_buffer_.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if (!lidar_pushed_) {
        meas.lidar = lidar_buffer_.front();
        meas.lidar_beg_time = time_buffer_.front();
        if (meas.lidar->points.size() <= 5) {
            lidar_end_time_ = meas.lidar_beg_time + lidar_mean_scantime_;
            RCLCPP_WARN(this->get_logger(), "Too few input point cloud!");
        } else if (meas.lidar->points.back().curvature / double(1000) <
                   0.5 * lidar_mean_scantime_) {
            lidar_end_time_ = meas.lidar_beg_time + lidar_mean_scantime_;
        } else {
            scan_num_++;
            lidar_end_time_ =
                meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
            lidar_mean_scantime_ +=
                (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime_) /
                scan_num_;
        }

        meas.lidar_end_time = lidar_end_time_;
        lidar_pushed_ = true;
    }

    if (last_timestamp_imu_ < lidar_end_time_) {
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = rclcpp::Time(imu_buffer_.front()->header.stamp).seconds();
    meas.imu.clear();
    while ((!imu_buffer_.empty()) && (imu_time < lidar_end_time_)) {
        imu_time = rclcpp::Time(imu_buffer_.front()->header.stamp).seconds();
        if (imu_time > lidar_end_time_) break;
        meas.imu.push_back(imu_buffer_.front());
        imu_buffer_.pop_front();
    }

    lidar_buffer_.pop_front();
    time_buffer_.pop_front();
    lidar_pushed_ = false;
    return true;
}

// ── Coordinate transforms ──

void LioNode::pointBodyToWorld(PointType const* pi, PointType* po) {
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point_.rot.matrix() *
                     (state_point_.offset_R_L_I.matrix() * p_body + state_point_.offset_T_L_I) +
                 state_point_.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void LioNode::RGBpointBodyLidarToIMU(PointType const* pi, PointType* po) {
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point_.offset_R_L_I.matrix() * p_body_lidar + state_point_.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

// ── FOV segment ──

void LioNode::lasermap_fov_segment() {
    cub_needrm_.clear();
    kdtree_delete_counter_ = 0;

    V3D pos_LiD = pos_lid_;
    if (!Localmap_Initialized_) {
        for (int i = 0; i < 3; i++) {
            LocalMap_Points_.vertex_min[i] = pos_LiD(i) - cube_len_ / 2.0;
            LocalMap_Points_.vertex_max[i] = pos_LiD(i) + cube_len_ / 2.0;
        }
        Localmap_Initialized_ = true;
        return;
    }

    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++) {
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points_.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points_.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= 1.5f * det_range_ ||
            dist_to_map_edge[i][1] <= 1.5f * det_range_)
            need_move = true;
    }
    if (!need_move) return;

    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points_;
    float mov_dist = std::max((cube_len_ - 2.0 * 1.5f * det_range_) * 0.5 * 0.9,
                              double(det_range_ * (1.5f - 1)));
    for (int i = 0; i < 3; i++) {
        tmp_boxpoints = LocalMap_Points_;
        if (dist_to_map_edge[i][0] <= 1.5f * det_range_) {
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points_.vertex_max[i] - mov_dist;
            cub_needrm_.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= 1.5f * det_range_) {
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points_.vertex_min[i] + mov_dist;
            cub_needrm_.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points_ = New_LocalMap_Points;

    PointVector points_history;
    ikdtree_.acquire_removed_points(points_history);

    if (cub_needrm_.size() > 0) kdtree_delete_counter_ = ikdtree_.Delete_Point_Boxes(cub_needrm_);
}

// ── Map update ──

void LioNode::map_incremental() {
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size_);
    PointNoNeedDownsample.reserve(feats_down_size_);
    for (int i = 0; i < feats_down_size_; i++) {
        pointBodyToWorld(&(feats_down_body_->points[i]), &(feats_down_world_->points[i]));

        if (!Nearest_Points_[i].empty() && flg_EKF_inited_) {
            const PointVector& points_near = Nearest_Points_[i];
            bool need_add = true;
            PointType mid_point;
            mid_point.x = floor(feats_down_world_->points[i].x / filter_size_map_min_) *
                              filter_size_map_min_ +
                          0.5 * filter_size_map_min_;
            mid_point.y = floor(feats_down_world_->points[i].y / filter_size_map_min_) *
                              filter_size_map_min_ +
                          0.5 * filter_size_map_min_;
            mid_point.z = floor(feats_down_world_->points[i].z / filter_size_map_min_) *
                              filter_size_map_min_ +
                          0.5 * filter_size_map_min_;
            float dist = calc_dist(feats_down_world_->points[i], mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min_ &&
                fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min_ &&
                fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min_) {
                PointNoNeedDownsample.push_back(feats_down_world_->points[i]);
                continue;
            }
            for (int j = 0; j < NUM_MATCH_POINTS; j++) {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[j], mid_point) < dist) {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world_->points[i]);
        } else {
            PointToAdd.push_back(feats_down_world_->points[i]);
        }
    }

    ikdtree_.Add_Points(PointToAdd, true);
    ikdtree_.Add_Points(PointNoNeedDownsample, false);
    add_point_size_ = PointToAdd.size() + PointNoNeedDownsample.size();
}

// ── Publishing ──

template <typename T>
void set_posestamp(T& out, const state_ikfom& state) {
    out.pose.position.x = state.pos(0);
    out.pose.position.y = state.pos(1);
    out.pose.position.z = state.pos(2);

    auto q_ = Eigen::Quaterniond(state.rot.matrix());
    out.pose.orientation.x = q_.coeffs()[0];
    out.pose.orientation.y = q_.coeffs()[1];
    out.pose.orientation.z = q_.coeffs()[2];
    out.pose.orientation.w = q_.coeffs()[3];
}

void LioNode::publish_odometry() {
    odomAftMapped_.header.frame_id = "map";
    odomAftMapped_.child_frame_id = "odom";
    odomAftMapped_.header.stamp = this->now();
    set_posestamp(odomAftMapped_.pose, state_point_);
    pub_odom_->publish(odomAftMapped_);

    auto P = kf_.get_P();
    for (int i = 0; i < 6; i++) {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped_.pose.covariance[i * 6 + 0] = P(k, 3);
        odomAftMapped_.pose.covariance[i * 6 + 1] = P(k, 4);
        odomAftMapped_.pose.covariance[i * 6 + 2] = P(k, 5);
        odomAftMapped_.pose.covariance[i * 6 + 3] = P(k, 0);
        odomAftMapped_.pose.covariance[i * 6 + 4] = P(k, 1);
        odomAftMapped_.pose.covariance[i * 6 + 5] = P(k, 2);
    }
}

void LioNode::publish_tf() {
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = odomAftMapped_.header.stamp;
    transform_stamped.header.frame_id = "map";
    transform_stamped.child_frame_id = "odom";
    transform_stamped.transform.translation.x = odomAftMapped_.pose.pose.position.x;
    transform_stamped.transform.translation.y = odomAftMapped_.pose.pose.position.y;
    transform_stamped.transform.translation.z = odomAftMapped_.pose.pose.position.z;
    transform_stamped.transform.rotation.x = odomAftMapped_.pose.pose.orientation.x;
    transform_stamped.transform.rotation.y = odomAftMapped_.pose.pose.orientation.y;
    transform_stamped.transform.rotation.z = odomAftMapped_.pose.pose.orientation.z;
    transform_stamped.transform.rotation.w = odomAftMapped_.pose.pose.orientation.w;
    tf_broadcaster_->sendTransform(transform_stamped);
}

void LioNode::publish_cloud() {
    if (scan_pub_en_) {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en_ ? feats_undistort_ : feats_down_body_);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++) {
            pointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
        }

        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = this->now();
        laserCloudmsg.header.frame_id = "map";
        pub_cloud_registered_->publish(laserCloudmsg);
        publish_count_ -= PUBFRAME_PERIOD;
    }

    /**************** save map ****************/
    if (pcd_save_en_) {
        int size = feats_undistort_->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++) {
            pointBodyToWorld(&feats_undistort_->points[i], &laserCloudWorld->points[i]);
        }

        static int scan_wait_num = 0;
        scan_wait_num++;
        if (scan_wait_num % 4 == 0) *pcl_wait_save_ += *laserCloudWorld;

        if (pcl_wait_save_->size() > 0 && pcd_save_interval_ > 0 &&
            scan_wait_num >= pcd_save_interval_) {
            pcd_index_++;
            std::string all_points_dir(std::string(ROOT_DIR) + "PCD/scans_" +
                                       std::to_string(pcd_index_) + ".pcd");
            pcl::PCDWriter pcd_writer;
            std::cout << "current scan saved to /PCD/" << all_points_dir << std::endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save_);
            pcl_wait_save_->clear();
            scan_wait_num = 0;
        }
    }
}

void LioNode::publish_frame_body() {
    int size = feats_undistort_->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++) {
        RGBpointBodyLidarToIMU(&feats_undistort_->points[i], &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = this->now();
    laserCloudmsg.header.frame_id = "odom";
    pub_cloud_registered_body_->publish(laserCloudmsg);
    publish_count_ -= PUBFRAME_PERIOD;
}

void LioNode::publish_map() {
    sensor_msgs::msg::PointCloud2 laserCloudMsg;
    pcl::toROSMsg(*featsFromMap_, laserCloudMsg);
    laserCloudMsg.header.stamp = this->now();
    laserCloudMsg.header.frame_id = "map";
    pub_laser_map_->publish(laserCloudMsg);
}

void LioNode::publish_path() {
    set_posestamp(msg_body_pose_, state_point_);
    msg_body_pose_.header.stamp = this->now();
    msg_body_pose_.header.frame_id = "map";

    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0) {
        path_.poses.push_back(msg_body_pose_);
        pub_path_->publish(path_);
    }
}

void LioNode::save_map() {
    PointVector().swap(ikdtree_.PCL_Storage);
    ikdtree_.flatten(ikdtree_.Root_Node, ikdtree_.PCL_Storage, NOT_RECORD);
    featsFromMap_->clear();
    featsFromMap_->points = ikdtree_.PCL_Storage;

    std::string file_name = std::string("GlobalMap_ikdtree.pcd");
    std::string all_points_dir(ROOT_DIR + std::string("PCD/") + file_name);
    pcl::PCDWriter pcd_writer;
    std::cout << "Saving map to " << all_points_dir << std::endl;
    pcd_writer.writeBinary(all_points_dir, *featsFromMap_);
}

}  // namespace omr_lio

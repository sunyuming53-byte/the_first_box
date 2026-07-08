#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl/filters/voxel_grid.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "omr_lio/IMU_Processing.hpp"
#include "omr_lio/common_lib.h"
#include "omr_lio/esekfom.hpp"
#include "omr_lio/preprocess.h"
#include <ikd-Tree/ikd_Tree.h>

namespace omr_lio {

class LioNode : public rclcpp::Node {
public:
    explicit LioNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~LioNode();

    void run();

private:
    // ── Callbacks ──
    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr& msg);

    // ── Main processing ──
    void process_loop();
    bool sync_packages(MeasureGroup& meas);

    // ── Algorithm helpers ──
    void pointBodyToWorld(PointType const* pi, PointType* po);
    void RGBpointBodyLidarToIMU(PointType const* pi, PointType* po);
    void lasermap_fov_segment();
    void map_incremental();

    // ── Publishing helpers ──
    void publish_odometry();
    void publish_cloud();
    void publish_frame_body();
    void publish_map();
    void publish_path();
    void publish_tf();
    void save_map();

    // ── Configuration ──
    void declare_params();
    void load_params();

    // ── Threading ──
    std::mutex mtx_buffer_;
    std::condition_variable sig_buffer_;
    std::atomic<bool> flg_exit_{false};

    // ── Publishers ──
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_registered_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_registered_body_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_laser_map_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;

    // ── Subscribers ──
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;

    // ── TF ──
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // ── Config params ──
    bool path_en_{true};
    bool scan_pub_en_{true};
    bool dense_pub_en_{true};
    bool scan_body_pub_en_{true};
    bool pcd_save_en_{false};
    bool time_sync_en_{false};
    bool extrinsic_est_en_{true};
    int num_max_iterations_{4};
    int pcd_save_interval_{-1};
    std::string map_file_path_;
    std::string lid_topic_{"/livox/lidar"};
    std::string imu_topic_{"/livox/imu"};
    double time_diff_lidar_to_imu_{0.0};
    double filter_size_corner_min_{0.5};
    double filter_size_surf_min_{0.5};
    double filter_size_map_min_{0.5};
    double cube_len_{200.0};
    float det_range_{300.0f};
    double fov_deg_{180.0};
    double gyr_cov_{0.1};
    double acc_cov_{0.1};
    double b_gyr_cov_{0.0001};
    double b_acc_cov_{0.0001};
    std::vector<double> extrinT_{0.0, 0.0, 0.0};
    std::vector<double> extrinR_{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

    // ── Algorithm state (all globals from laserMapping.cpp) ──
    int add_point_size_{0};
    int kdtree_delete_counter_{0};
    float res_last_[100000]{0.0};
    double last_timestamp_lidar_{0.0};
    double last_timestamp_imu_{-1.0};
    double lidar_end_time_{0.0};
    double first_lidar_time_{0.0};
    int scan_count_{0};
    int publish_count_{0};
    int feats_down_size_{0};
    int pcd_index_{0};
    double lidar_mean_scantime_{0.0};
    int scan_num_{0};
    double timediff_lidar_wrt_imu_{0.0};
    bool timediff_set_flg_{false};
    bool lidar_pushed_{false};
    bool flg_first_scan_{true};
    bool flg_EKF_inited_{false};

    // ── Buffers ──
    std::deque<double> time_buffer_;
    std::deque<PointCloudXYZI::Ptr> lidar_buffer_;
    std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_buffer_;

    // ── Point cloud storage ──
    PointCloudXYZI::Ptr featsFromMap_{std::make_shared<PointCloudXYZI>()};
    PointCloudXYZI::Ptr feats_undistort_{std::make_shared<PointCloudXYZI>()};
    PointCloudXYZI::Ptr feats_down_body_{std::make_shared<PointCloudXYZI>()};
    PointCloudXYZI::Ptr feats_down_world_{std::make_shared<PointCloudXYZI>()};
    PointCloudXYZI::Ptr pcl_wait_pub_{std::make_shared<PointCloudXYZI>(500000, 1)};
    PointCloudXYZI::Ptr pcl_wait_save_{std::make_shared<PointCloudXYZI>()};

    // ── ikd-Tree ──
    KD_TREE<PointType> ikdtree_;
    std::vector<BoxPointType> cub_needrm_;
    std::vector<PointVector> Nearest_Points_;
    BoxPointType LocalMap_Points_{};
    bool Localmap_Initialized_{false};

    // ── Filters ──
    pcl::VoxelGrid<PointType> downSizeFilterSurf_;
    pcl::VoxelGrid<PointType> downSizeFilterMap_;

    // ── ESKF ──
    esekfom::esekf kf_;
    state_ikfom state_point_;
    Eigen::Vector3d pos_lid_{Eigen::Vector3d::Zero()};

    // ── Extrinsics ──
    V3D Lidar_T_wrt_IMU_{Zero3d};
    M3D Lidar_R_wrt_IMU_{Eye3d};

    // ── Odometry/path messages ──
    nav_msgs::msg::Path path_;
    nav_msgs::msg::Odometry odomAftMapped_;
    geometry_msgs::msg::PoseStamped msg_body_pose_;

    // ── Preprocessor + IMU processor ──
    std::shared_ptr<Preprocess> p_pre_{std::make_shared<Preprocess>()};
    std::shared_ptr<ImuProcess> p_imu_{std::make_shared<ImuProcess>()};

    // ── Current measurement ──
    MeasureGroup Measures_;
};

}  // namespace omr_lio

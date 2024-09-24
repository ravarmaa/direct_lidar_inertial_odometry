/***********************************************************
 *                                                         *
 * Copyright (c)                                           *
 *                                                         *
 * The Verifiable & Control-Theoretic Robotics (VECTR) Lab *
 * University of California, Los Angeles                   *
 *                                                         *
 * Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez   *
 * Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu         *
 *                                                         *
 ***********************************************************/

#include "dlio/dlio.h"

class dlio::OdomNode {

public:

  OdomNode(ros::NodeHandle node_handle);
  ~OdomNode();

  void start();

private:

  struct State;
  struct ImuMeas;
  struct Lidar;

  void getParams();

  void callbackPointCloud(const sensor_msgs::PointCloud2ConstPtr& pc0, const sensor_msgs::PointCloud2ConstPtr& pc1);
  void callbackImu(const sensor_msgs::Imu::ConstPtr& imu);
  void asyncCallBack(const sensor_msgs::PointCloud2ConstPtr& pc, Lidar& lidar);
  void lidar0cb(const sensor_msgs::PointCloud2ConstPtr& pc);
  void lidar1cb(const sensor_msgs::PointCloud2ConstPtr& pc);

  void publishPose(const ros::TimerEvent& e);

  void publishToROS(Eigen::Matrix4f T_cloud);
  void publishCloud(Eigen::Matrix4f T_cloud, Lidar& lidar);
  void publishKeyframe(std::pair<std::pair<Eigen::Vector3f, Eigen::Quaternionf>,
                       pcl::PointCloud<PointType>::ConstPtr> kf, ros::Time timestamp);

  void getScanFromROS(const sensor_msgs::PointCloud2ConstPtr& pc, Lidar& lidar);
  void preprocessPoints(Lidar& lidar);
  void deskewPointcloud(Lidar& lidar);
  void initializeInputTarget();
  void setInputSource();

  pcl::PointCloud<PointType>::Ptr mergePointClouds(
    const pcl::PointCloud<PointType>::ConstPtr& scan1,
    const pcl::PointCloud<PointType>::ConstPtr& scan2);

  void initializeDLIO();

  void getNextPose();
  bool imuMeasFromTimeRange(double start_time, double end_time,
                            boost::circular_buffer<ImuMeas>::reverse_iterator& begin_imu_it,
                            boost::circular_buffer<ImuMeas>::reverse_iterator& end_imu_it);
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
    integrateImu(double start_time, Eigen::Quaternionf q_init, Eigen::Vector3f p_init, Eigen::Vector3f v_init,
                 const std::vector<double>& sorted_timestamps);
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
    integrateImuInternal(Eigen::Quaternionf q_init, Eigen::Vector3f p_init, Eigen::Vector3f v_init,
                         const std::vector<double>& sorted_timestamps,
                         boost::circular_buffer<ImuMeas>::reverse_iterator begin_imu_it,
                         boost::circular_buffer<ImuMeas>::reverse_iterator end_imu_it);
  void propagateGICP();

  void propagateState();
  void updateState();

  void setAdaptiveParams();
  void setKeyframeCloud();

  void computeMetrics();
  void computeSpaciousness();
  void computeDensity();

  sensor_msgs::Imu::Ptr transformImu(const sensor_msgs::Imu::ConstPtr& imu);

  void updateKeyframes();
  void computeConvexHull();
  void computeConcaveHull();
  void pushSubmapIndices(std::vector<float> dists, int k, std::vector<int> frames);
  void buildSubmap(State vehicle_state);
  void buildKeyframesAndSubmap(State vehicle_state);
  void pauseSubmapBuildIfNeeded();

  void debug();

  ros::NodeHandle nh;
  ros::Timer publish_timer;

  // Subscribers
  ros::Subscriber imu_sub;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> SyncPolicy;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync;

  // Publishers
  ros::Publisher odom_pub;
  ros::Publisher pose_pub;
  ros::Publisher path_pub;

  ros::Publisher kf_pose_pub;
  ros::Publisher kf_cloud_pub;
  

  // ROS Msgs
  nav_msgs::Odometry odom_ros;
  geometry_msgs::PoseStamped pose_ros;
  nav_msgs::Path path_ros;
  geometry_msgs::PoseArray kf_pose_ros;

  // Flags
  std::atomic<bool> dlio_initialized;
  std::atomic<bool> first_valid_scan;
  std::atomic<bool> first_imu_received;
  std::atomic<bool> imu_calibrated;
  std::atomic<bool> submap_hasChanged;
  std::atomic<bool> gicp_hasConverged;

  // Threads
  std::thread publish_thread;
  std::thread publish_keyframe_thread;
  std::thread metrics_thread;
  std::thread debug_thread;

  // Trajectory
  std::vector<std::pair<Eigen::Vector3f, Eigen::Quaternionf>> trajectory;
  double length_traversed;

  // Keyframes
  std::vector<std::pair<std::pair<Eigen::Vector3f, Eigen::Quaternionf>,
                        pcl::PointCloud<PointType>::ConstPtr>> keyframes;
  std::vector<ros::Time> keyframe_timestamps;
  std::vector<std::shared_ptr<const nano_gicp::CovarianceList>> keyframe_normals;
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> keyframe_transformations;
  std::mutex keyframes_mutex;

  // Frames
  std::string odom_frame;
  std::string baselink_frame;
  std::string imu_frame;

  // Preprocessing
  pcl::CropBox<PointType> crop;
  pcl::VoxelGrid<PointType> voxel;

  // Keyframes
  pcl::PointCloud<PointType>::ConstPtr keyframe_cloud;
  int num_processed_keyframes;

  pcl::ConvexHull<PointType> convex_hull;
  pcl::ConcaveHull<PointType> concave_hull;
  std::vector<int> keyframe_convex;
  std::vector<int> keyframe_concave;

  // Submap
  pcl::PointCloud<PointType>::ConstPtr submap_cloud;
  std::shared_ptr<const nano_gicp::CovarianceList> submap_normals;
  std::shared_ptr<const nanoflann::KdTreeFLANN<PointType>> submap_kdtree;

  std::vector<int> submap_kf_idx_curr;
  std::vector<int> submap_kf_idx_prev;

  bool new_submap_is_ready;
  std::future<void> submap_future;
  std::condition_variable submap_build_cv;
  bool main_loop_running;
  std::mutex main_loop_running_mutex;

  
  std::vector<double> comp_times;
  std::vector<double> imu_rates;
  std::vector<double> lidar_rates;

  double first_scan_stamp;
  double elapsed_time;

  // GICP
  nano_gicp::NanoGICP<PointType, PointType> gicp;
  nano_gicp::NanoGICP<PointType, PointType> gicp_temp;

  // Transformations
  Eigen::Quaternionf q_final;

  Eigen::Vector3f origin;

  struct SE3 {
      Eigen::Vector3f t;
      Eigen::Matrix3f R;
  };

  SE3 baselink2imu;
  Eigen::Matrix4f baselink2imu_T;

  // IMU
  ros::Time imu_stamp;
  double first_imu_stamp;
  double prev_imu_stamp;
  double imu_dp, imu_dq_deg;

  struct ImuMeas {
    double stamp;
    double dt; // defined as the difference between the current and the previous measurement
    Eigen::Vector3f ang_vel;
    Eigen::Vector3f lin_accel;
  }; ImuMeas imu_meas;

  boost::circular_buffer<ImuMeas> imu_buffer;
  std::mutex mtx_imu;
  std::condition_variable cv_imu_stamp;

  static bool comparatorImu(ImuMeas m1, ImuMeas m2) {
    return (m1.stamp < m2.stamp);
  };

  // Geometric Observer
  struct Geo {
    bool first_opt_done;
    std::mutex mtx;
    double dp;
    double dq_deg;
    Eigen::Vector3f prev_p;
    Eigen::Quaternionf prev_q;
    Eigen::Vector3f prev_vel;
  }; Geo geo;

  // State Vector
  struct ImuBias {
    Eigen::Vector3f gyro;
    Eigen::Vector3f accel;
  };

  struct Frames {
    Eigen::Vector3f b;
    Eigen::Vector3f w;
  };

  struct Velocity {
    Frames lin;
    Frames ang;
  };

  struct State {
    Eigen::Vector3f p; // position in world frame
    Eigen::Quaternionf q; // orientation in world frame
    Velocity v;
    ImuBias b; // imu biases in body frame
  }; State state;

  struct Pose {
    Eigen::Vector3f p; // position in world frame
    Eigen::Quaternionf q; // orientation in world frame
  };


  int using_lidar;

  Pose imuPose;
  Pose lidarPose; // world-->baselink in reference lidar's time
  Eigen::Matrix4f T, T_prior, T_corr; // ICP transformations world-->baselink

  // int using_lidar;
  struct Lidar {
    int id;
    Eigen::Matrix4f baselink2lidar_T;
    SE3 baselink2lidar;

    Eigen::Matrix4f integrated_T;

    State current_state;
    State previous_state;

    // Timestamps
    ros::Time scan_header_stamp;
    double scan_stamp;
    double prev_scan_stamp;
    double scan_dt;

    ros::Time debug_time;

    std::string frame;
    // Point Clouds
    pcl::PointCloud<PointType>::ConstPtr original_scan;
    pcl::PointCloud<PointType>::ConstPtr deskewed_scan;
    pcl::PointCloud<PointType>::ConstPtr current_scan;

    bool deskew_;
    bool time_offset_;

    std::atomic<bool> deskew_status;
    std::atomic<int> deskew_size;

    ros::Publisher deskewed_pub;

    message_filters::Subscriber<sensor_msgs::PointCloud2> synced_sub;
    ros::Subscriber async_sub;
    // ros::Subscriber sub;

    // Sensor Type
    dlio::SensorType sensor;
  }; 

  Lidar lidar0;
  Lidar lidar1;
  pcl::PointCloud<PointType>::ConstPtr combined_scan;


  // Metrics
  struct Metrics {
    std::vector<float> spaciousness;
    std::vector<float> density;
  }; Metrics metrics;

  std::string cpu_type;
  std::vector<double> cpu_percents;
  clock_t lastCPU, lastSysCPU, lastUserCPU;
  int numProcessors;

  // Parameters
  std::string version_;
  int num_threads_;

  

  double gravity_;

  bool adaptive_params_;

  double obs_submap_thresh_;
  double obs_keyframe_thresh_;
  double obs_keyframe_lag_;

  double keyframe_thresh_dist_;
  double keyframe_thresh_rot_;

  int submap_knn_;
  int submap_kcv_;
  int submap_kcc_;
  double submap_concave_alpha_;

  bool densemap_filtered_;
  bool wait_until_move_;

  double crop_size_;

  bool vf_use_;
  double vf_res_;

  bool imu_calibrate_;
  bool calibrate_gyro_;
  bool calibrate_accel_;
  bool gravity_align_;
  double imu_calib_time_;
  int imu_buffer_size_;
  Eigen::Matrix3f imu_accel_sm_;

  int gicp_min_num_points_;
  int gicp_k_correspondences_;
  double gicp_max_corr_dist_;
  int gicp_max_iter_;
  double gicp_transformation_ep_;
  double gicp_rotation_ep_;
  double gicp_init_lambda_factor_;

  double geo_Kp_;
  double geo_Kv_;
  double geo_Kq_;
  double geo_Kab_;
  double geo_Kgb_;
  double geo_abias_max_;
  double geo_gbias_max_;

};

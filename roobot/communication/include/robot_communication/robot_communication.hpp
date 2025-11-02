#ifndef ROBOT_COMMUNICATION
#define ROBOT_COMMUNICATION

#include <netinet/in.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Transform.h>

#include <cstdint>
#include <memory>
#include <pcl/impl/point_types.hpp>
#include <queue>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/int8.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace robot_communication {
class RobotCommunicationNode : public rclcpp::Node {
 public:
  RobotCommunicationNode(const rclcpp::NodeOptions& options);
  ~RobotCommunicationNode() override;

 private:
  int port;
  std::string ip;
  int sockfd;
  struct sockaddr_in server_addr;

  int robot_id;

  std::thread send_thread_;
  std::thread recv_thread_;
  std::thread parse_buffer_thread_;
  std::thread tf_update_thread_;

  struct SendBuffer {
    int id = -1;
    std::vector<uint8_t> buffer;
    int msg_type = -1;
  };
  std::queue<SendBuffer> send_buffer_queue;
  std::queue<std::vector<uint8_t>> recv_buffer_queue;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  Eigen::Matrix4d odom_to_local_matrix;
  Eigen::Matrix4d local_to_global_matrix;
  Eigen::Matrix4d camera_to_base_matrix;

  void InitMapTF();
  void InitClient();

  void NetworkSendThread();
  void NetworkRecvThread();
  void TFUpdateThread();
  void PrepareBuffer(const SendBuffer& prepare_buffer);
  void ParseBufferThread();

  void RegisteredScanCallBack(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr registered_scan_msg);
  void MapCallBack(
    const nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_msg);
  void RealsensePointCallBack(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr
      realsense_pointcloud_msg);
  void OdometryCallBack(
    const nav_msgs::msg::Odometry::ConstSharedPtr odometry_msg);
  void ImageCallBack(const sensor_msgs::msg::Image::ConstSharedPtr image_msg);
  void PlanCallBack(const nav_msgs::msg::Path::ConstSharedPtr path_msg);

  void WayPointCallBack(
    const geometry_msgs::msg::PointStamped::ConstSharedPtr way_point_msg);
  
  void SendNavigationStatus(int8_t status);
  void SendNavigationPath(const nav_msgs::msg::Path& path);

  template <class T>
  std::vector<uint8_t> SerializeMsg(const T& msg);
  template <class T>
  T DeserializeMsg(const std::vector<uint8_t>& data);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
    registered_scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
    way_point_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
    realsense_pointcloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;

  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav2_client_;

  // 地图增量传输相关
  nav_msgs::msg::OccupancyGrid last_map_;     // 上次发送的完整地图
  rclcpp::Time last_map_send_time_;           // 上次发送地图的时间（增量或完整）
  rclcpp::Time last_full_map_send_time_;      // 上次发送完整地图的时间
  double map_send_interval_ = 0.2;            // 增量地图发送间隔（秒）
  double full_map_send_interval_ = 5.0;      // 完整地图发送间隔（秒）
  bool first_map_sent_ = false;               // 是否已发送过完整地图
  
  // 导航状态跟踪
  int8_t last_nav_status_ = -1;               // 上次发送的导航状态（-1表示未初始化）
  
  // 目标点去重
  geometry_msgs::msg::PointStamped last_received_waypoint_;  // 上次接收的目标点
  bool has_received_waypoint_ = false;                       // 是否接收过目标点
  static constexpr double WAYPOINT_DUPLICATE_THRESHOLD = 0.01;  // 去重阈值（米）
  
  // 路径发送频率控制
  nav_msgs::msg::Path last_sent_path_;                       // 上次发送的路径
  rclcpp::Time last_path_send_time_;                         // 上次发送路径的时间
  bool has_sent_path_ = false;                               // 是否发送过路径
  static constexpr double PATH_SEND_INTERVAL = 1.0;          // 路径发送最小间隔（秒）
  static constexpr double PATH_CHANGE_THRESHOLD = 0.1;       // 路径变化阈值（米）
};
}  // namespace robot_communication

#endif
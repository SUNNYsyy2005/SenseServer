#ifndef ROBOT_COMMUNICATION
#define ROBOT_COMMUNICATION

#include <arpa/inet.h>
#include <netinet/in.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <memory>
#include <tf2_ros/transform_broadcaster.h>  // 添加：TransformBroadcaster 声明
#include <pcl/impl/point_types.hpp>
#include <queue>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos_event.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <tf2_ros/transform_broadcaster.h>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/int8.hpp"

#define MAX_ROBOT_COUNT 5

namespace robot_communication {
class RobotCommunicationNode : public rclcpp::Node {
 public:
  RobotCommunicationNode(const rclcpp::NodeOptions& options);
  ~RobotCommunicationNode() override;

 private:
  int port;
  std::string ip;
  int sockfd;
  struct sockaddr_in server_addr, saved_client_addr[MAX_ROBOT_COUNT];

  int robot_count = 3;
  bool update_timestamp_on_receive = true;  // 控制是否将接收消息的时间戳更新为接收时间

  std::thread send_thread_;
  std::thread recv_thread_;
  std::thread parse_buffer_thread_[MAX_ROBOT_COUNT];

  struct SendBuffer {
    int id = -1;
    std::vector<uint8_t> buffer;
    int msg_type = -1;
  };
  std::queue<SendBuffer> send_buffer_queue;
  std::queue<std::vector<uint8_t>> recv_buffer_queue[MAX_ROBOT_COUNT];

  void InitServer();

  void NetworkSendThread();
  void NetworkRecvThread();
  void PrepareBuffer(const SendBuffer& buffer);
  void ParseBufferThread(const int robot_id);

  void WayPointCallBack(
    const geometry_msgs::msg::PointStamped::ConstSharedPtr way_point_msg,
    const int robot_id);

  template <class T>
  std::vector<uint8_t> SerializeMsg(const T& msg);
  template <class T>
  T DeserializeMsg(const std::vector<uint8_t>& data);

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    registered_scan_pub_[MAX_ROBOT_COUNT];
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    realsense_pointcloud_pub_[MAX_ROBOT_COUNT];
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
    map_pub_[MAX_ROBOT_COUNT];
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
    image_pub_[MAX_ROBOT_COUNT];
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr
    nav2_status_pub_[MAX_ROBOT_COUNT];
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr
    nav2_path_pub_[MAX_ROBOT_COUNT];

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
    way_point_sub_[MAX_ROBOT_COUNT];

  // 用于发送接收到的 transform
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // 地图增量传输相关
  nav_msgs::msg::OccupancyGrid cached_maps_[MAX_ROBOT_COUNT];  // 缓存的完整地图
  bool map_initialized_[MAX_ROBOT_COUNT] = {false};            // 地图是否已初始化
};
}  // namespace robot_communication

#endif
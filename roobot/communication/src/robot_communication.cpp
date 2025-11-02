#include <arpa/inet.h>
#include <netinet/in.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rmw/qos_profiles.h>
#include <rmw/rmw.h>
#include <rmw/types.h>
#include <sys/socket.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/convert.h>
#include <tf2/time.h>
#include <tf2/transform_datatypes.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/utilities.hpp>
#include <string>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <vector>

#include "robot_communication/robot_communication.hpp"

#define MAX_PACKET_SIZE 64000
#define BUFFER_SIZE 65535
#define MAX_BUFFER_QUEUE_SIZE 256

#define LOCAL_MAP_FRAME "map"
#define MAP_FRAME "global_map"
#define CAMERA_FRAME "camera_depth_optical_frame"
#define BASE_FRAME "base_link"

namespace robot_communication {
RobotCommunicationNode::RobotCommunicationNode(
  const rclcpp::NodeOptions& options)
    : Node("robot_communication", options),
      last_map_send_time_(this->get_clock()->now()),
      last_full_map_send_time_(this->get_clock()->now()),
      last_path_send_time_(this->get_clock()->now()) {
  this->declare_parameter<int>("robot_id", 0);
  this->declare_parameter<int>("network_port", 12130);
  this->declare_parameter<std::string>("network_ip", "192.168.31.207");
  this->declare_parameter<double>("map_send_interval", 5.0);
  this->declare_parameter<double>("full_map_send_interval", 30.0);

  this->get_parameter("robot_id", robot_id);
  this->get_parameter("network_port", port);
  this->get_parameter("network_ip", ip);
  this->get_parameter("map_send_interval", map_send_interval_);
  this->get_parameter("full_map_send_interval", full_map_send_interval_);

  RCLCPP_INFO(this->get_logger(), "Map send interval: %.1f seconds (delta), %.1f seconds (full)", 
              map_send_interval_, full_map_send_interval_);

  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  qos.best_effort();

  registered_scan_sub_ =
    this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/lidar/pointcloud", qos,
      std::bind(&RobotCommunicationNode::RegisteredScanCallBack, this,
                std::placeholders::_1));
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "map", qos,
    std::bind(&RobotCommunicationNode::MapCallBack, this,
              std::placeholders::_1));
  way_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "way_point", qos,
    std::bind(&RobotCommunicationNode::WayPointCallBack, this,
              std::placeholders::_1));
  realsense_pointcloud_sub_ =
    this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "camera/camera/depth/color/points", qos,
      std::bind(&RobotCommunicationNode::RealsensePointCallBack, this,
                std::placeholders::_1));
  odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "Odometry", qos,
    std::bind(&RobotCommunicationNode::OdometryCallBack, this,
              std::placeholders::_1));
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "camera/camera/depth/color/image_raw", qos,
    std::bind(&RobotCommunicationNode::ImageCallBack, this,
              std::placeholders::_1));
  plan_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "plan", qos,
    std::bind(&RobotCommunicationNode::PlanCallBack, this,
              std::placeholders::_1));

  this->nav2_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    this,
    "navigate_to_pose");

  InitMapTF();
  InitClient();
}

RobotCommunicationNode::~RobotCommunicationNode() {
  if (send_thread_.joinable()) {
    send_thread_.join();
  }
  if (recv_thread_.joinable()) {
    recv_thread_.join();
  }
  if (parse_buffer_thread_.joinable()) {
    parse_buffer_thread_.join();
  }
  if (tf_update_thread_.joinable()) {
    tf_update_thread_.join();
  }
  close(sockfd);
}

// initialize the tf from local_map to map
void RobotCommunicationNode::InitMapTF() {
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  geometry_msgs::msg::TransformStamped local_to_global_transform,
    camera_to_base_transform;
/*   while (rclcpp::ok()) {
    try {
      local_to_global_transform = tf_buffer_->lookupTransform(
        MAP_FRAME, LOCAL_MAP_FRAME, tf2::TimePointZero,
        tf2::durationFromSec(5.0));
      local_to_global_matrix =
        tf2::transformToEigen(local_to_global_transform.transform)
          .matrix()
          .cast<double>();
      break;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
    }
  } */
  try {
    camera_to_base_transform = tf_buffer_->lookupTransform(
      BASE_FRAME, CAMERA_FRAME, tf2::TimePointZero, tf2::durationFromSec(5.0));
    camera_to_base_matrix =
      tf2::transformToEigen(camera_to_base_transform.transform)
        .matrix()
        .cast<double>();
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "Could not transform camera!");
  }
  tf_update_thread_ =
    std::thread(&RobotCommunicationNode::TFUpdateThread, this);
}

// initialize the socket client
void RobotCommunicationNode::InitClient() {
  if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    RCLCPP_ERROR(this->get_logger(), "Socket creation failed!");
    return;
  }

  memset(&server_addr, 0, sizeof(server_addr));

  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(port);
  server_addr.sin_addr.s_addr = inet_addr(ip.c_str());

  send_thread_ = std::thread(&RobotCommunicationNode::NetworkSendThread, this);
  recv_thread_ = std::thread(&RobotCommunicationNode::NetworkRecvThread, this);
  parse_buffer_thread_ =
    std::thread(&RobotCommunicationNode::ParseBufferThread, this);

  RCLCPP_INFO(this->get_logger(), "Client start! robot_id: %d, Target ip: %s, port: %d",
              robot_id, ip.c_str(), port);
}

void RobotCommunicationNode::WayPointCallBack(
  const geometry_msgs::msg::PointStamped::ConstSharedPtr way_point_msg) {
  RCLCPP_DEBUG(this->get_logger(), 
              "Received waypoint packet: x=%.2f, y=%.2f, z=%.2f, frame_id='%s'",
              way_point_msg->point.x, way_point_msg->point.y, way_point_msg->point.z,
              way_point_msg->header.frame_id.c_str());
  
  // 目标点去重：检查是否与上次接收的目标点相同
  if (has_received_waypoint_) {
    double dx = way_point_msg->point.x - last_received_waypoint_.point.x;
    double dy = way_point_msg->point.y - last_received_waypoint_.point.y;
    double dz = way_point_msg->point.z - last_received_waypoint_.point.z;
    double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
    
    if (distance < WAYPOINT_DUPLICATE_THRESHOLD) {
      RCLCPP_DEBUG(this->get_logger(), 
                  "Duplicate waypoint detected (distance=%.4fm < %.4fm), ignoring",
                  distance, WAYPOINT_DUPLICATE_THRESHOLD);
      return;  // 忽略重复的目标点
    }
  }
  
  // 缓存当前目标点
  last_received_waypoint_ = *way_point_msg;
  has_received_waypoint_ = true;
  
  RCLCPP_INFO(this->get_logger(), 
              "NEW waypoint accepted: x=%.2f, y=%.2f, z=%.2f, frame_id='%s'",
              way_point_msg->point.x, way_point_msg->point.y, way_point_msg->point.z,
              way_point_msg->header.frame_id.c_str());
  
  try {
    geometry_msgs::msg::PointStamped local_point = tf_buffer_->transform(
        *way_point_msg, LOCAL_MAP_FRAME, tf2::durationFromSec(10.0));
    
    RCLCPP_INFO(this->get_logger(), 
                "Transformed to local frame: x=%.2f, y=%.2f, z=%.2f",
                local_point.point.x, local_point.point.y, local_point.point.z);
    
    geometry_msgs::msg::PoseStamped local_pose;
    local_pose.header = local_point.header;
    local_pose.pose.position = local_point.point;
    local_pose.pose.orientation.w = 1.0;  // 设置默认方向

    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose = local_pose;

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    
    send_goal_options.goal_response_callback =
        [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr & goal_handle) {
            if (!goal_handle) {
                RCLCPP_ERROR(this->get_logger(), "目标被拒绝 (Goal REJECTED)");
                SendNavigationStatus(0);  // UNKNOWN
            } else {
                RCLCPP_INFO(this->get_logger(), "目标已接受 (Goal ACCEPTED)，等待结果");
                SendNavigationStatus(1);  // ACCEPTED
            }
        };

    send_goal_options.feedback_callback =
        [this](
            rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
            const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback) {
            // 只在状态变化时发送 EXECUTING 状态
            if (last_nav_status_ != 2) {
                SendNavigationStatus(2);  // EXECUTING
            }
            
            // 如果反馈中包含当前规划的路径，也可以发送
            // 注意：NavigateToPose 的 Feedback 中包含 distance_remaining 等信息
            RCLCPP_DEBUG(this->get_logger(), "Distance remaining: %.2f", 
                        feedback->distance_remaining);
        };

    send_goal_options.result_callback =
        [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result) {
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this->get_logger(), "✅ 导航成功! (Result: SUCCEEDED)");
                    SendNavigationStatus(4);  // SUCCEEDED
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(), "❌ 导航被中止 (Result: ABORTED)");
                    SendNavigationStatus(6);  // ABORTED
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_WARN(this->get_logger(), "⚠️ 导航取消 (Result: CANCELED)");
                    SendNavigationStatus(5);  // CANCELED
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "❓ 未知导航结果 (Result code: %d)", static_cast<int>(result.code));
                    SendNavigationStatus(0);  // UNKNOWN
                    break;
            }
        };

    RCLCPP_INFO(this->get_logger(), "Sending goal to Nav2...");
    this->nav2_client_->async_send_goal(goal_msg, send_goal_options);
    
  } catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR(this->get_logger(), 
                 "TF transform failed: %s. Cannot navigate to waypoint.", ex.what());
  } catch (const std::exception& ex) {
    RCLCPP_ERROR(this->get_logger(), 
                 "Exception in WayPointCallBack: %s", ex.what());
  }
}void RobotCommunicationNode::RegisteredScanCallBack(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr registered_scan_msg) {
  // Some lidar drivers (e.g., Livox) don't provide an 'intensity' field,
  // causing pcl::fromROSMsg<PointXYZI> to fail. We don't need to touch the
  // fields here; just forward and normalize the frame_id.
  sensor_msgs::msg::PointCloud2 totalRegisteredScan = *registered_scan_msg;
  totalRegisteredScan.header.frame_id = LOCAL_MAP_FRAME;

  std::vector<uint8_t> data_buffer =
    SerializeMsg<sensor_msgs::msg::PointCloud2>(totalRegisteredScan);
  SendBuffer prepare_buffer = {robot_id, data_buffer, 0};
  PrepareBuffer(prepare_buffer);
}

void RobotCommunicationNode::MapCallBack(
  const nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_msg) {
  
  // 验证地图数据的有效性
  if (map_msg->info.width == 0 || map_msg->info.height == 0) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Received invalid map dimensions (w=%u, h=%u), skipping",
                         map_msg->info.width, map_msg->info.height);
    return;
  }
  
  size_t expected_size = map_msg->info.width * map_msg->info.height;
  size_t actual_size = map_msg->data.size();
  if (expected_size != actual_size) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Map data size mismatch. Expected %zu (w=%u * h=%u), got %zu",
                         expected_size, map_msg->info.width, map_msg->info.height, actual_size);
    return;
  }
  
  rclcpp::Time now = this->get_clock()->now();
  double time_since_last_send = (now - last_map_send_time_).seconds();
  double time_since_last_full_send = (now - last_full_map_send_time_).seconds();
  
  // 检查是否到达发送间隔
  if (time_since_last_send < map_send_interval_) {
    return;
  }
  
  nav_msgs::msg::OccupancyGrid current_map = *map_msg;
  current_map.header.frame_id = LOCAL_MAP_FRAME;
  
  bool should_send_full_map = false;
  std::string full_map_reason = "";
  
  // 判断是否需要发送完整地图
  if (!first_map_sent_) {
    should_send_full_map = true;
    full_map_reason = "first time";
  } else if (last_map_.info.width != current_map.info.width ||
             last_map_.info.height != current_map.info.height) {
    should_send_full_map = true;
    full_map_reason = "size changed";
  } else if (time_since_last_full_send >= full_map_send_interval_) {
    should_send_full_map = true;
    full_map_reason = "periodic refresh";
  }
  
  // 如果需要发送完整地图
  if (should_send_full_map) {
    RCLCPP_DEBUG(this->get_logger(), 
                "Sending full map (reason: %s, size: %dx%d, %zu bytes)",
                full_map_reason.c_str(),
                current_map.info.width, current_map.info.height, 
                current_map.data.size());
    
    std::vector<uint8_t> data_buffer =
      SerializeMsg<nav_msgs::msg::OccupancyGrid>(current_map);
    SendBuffer prepare_buffer = {robot_id, data_buffer, 3};
    PrepareBuffer(prepare_buffer);
    
    last_map_ = current_map;
    first_map_sent_ = true;
    last_map_send_time_ = now;
    last_full_map_send_time_ = now;
    return;
  }
  
  // 计算增量地图
  nav_msgs::msg::OccupancyGrid delta_map;
  delta_map.header = current_map.header;
  delta_map.info = current_map.info;
  
  std::vector<int32_t> changed_indices;
  std::vector<int8_t> changed_values;
  
  for (size_t i = 0; i < current_map.data.size(); ++i) {
    if (current_map.data[i] != last_map_.data[i]) {
      changed_indices.push_back(static_cast<int32_t>(i));
      changed_values.push_back(current_map.data[i]);
    }
  }
  
  // 如果没有变化，不发送地图
  if (changed_indices.empty()) {
    RCLCPP_DEBUG(this->get_logger(), "No map changes detected, skipping transmission");
    last_map_send_time_ = now;  // 更新发送时间，避免频繁检查
    return;
  }
  
  // 如果变化超过50%，发送完整地图
  double change_ratio = static_cast<double>(changed_indices.size()) / current_map.data.size();
  if (change_ratio > 0.5) {
    RCLCPP_DEBUG(this->get_logger(), 
                "Change ratio %.1f%% too high, sending full map",
                change_ratio * 100.0);
    
    std::vector<uint8_t> data_buffer =
      SerializeMsg<nav_msgs::msg::OccupancyGrid>(current_map);
    SendBuffer prepare_buffer = {robot_id, data_buffer, 3};
    PrepareBuffer(prepare_buffer);
    
    last_map_ = current_map;
    last_map_send_time_ = now;
    last_full_map_send_time_ = now;
    return;
  }
  
  // 发送增量地图
  // 构造特殊格式：使用负宽度表示这是增量地图
  // 注意：需要保存原始宽度，因为负值会影响序列化
  uint32_t original_width = delta_map.info.width;
  
  // 使用负宽度作为标记，高度字段存储原始宽度
  delta_map.info.width = static_cast<uint32_t>(-static_cast<int32_t>(changed_indices.size()));
  delta_map.info.height = original_width;  // 临时存储原始宽度
  delta_map.data.clear();
  
  // 数据格式：[index1, value1, index2, value2, ...]
  for (size_t i = 0; i < changed_indices.size(); ++i) {
    // 将 int32_t 拆分为 4 个 int8_t 存储
    int32_t idx = changed_indices[i];
    delta_map.data.push_back(static_cast<int8_t>(idx & 0xFF));
    delta_map.data.push_back(static_cast<int8_t>((idx >> 8) & 0xFF));
    delta_map.data.push_back(static_cast<int8_t>((idx >> 16) & 0xFF));
    delta_map.data.push_back(static_cast<int8_t>((idx >> 24) & 0xFF));
    delta_map.data.push_back(changed_values[i]);
  }
  
  size_t delta_size = delta_map.data.size();
  size_t full_size = current_map.data.size();
  double compression_ratio = 100.0 * (1.0 - static_cast<double>(delta_size) / full_size);
  
  RCLCPP_DEBUG(this->get_logger(), 
              "Sending delta map: %zu changes (%.1f%%), bandwidth saved: %.1f%%, next full map in: %.1fs",
              changed_indices.size(), change_ratio * 100.0, compression_ratio,
              full_map_send_interval_ - time_since_last_full_send);
  
  std::vector<uint8_t> data_buffer =
    SerializeMsg<nav_msgs::msg::OccupancyGrid>(delta_map);
  SendBuffer prepare_buffer = {robot_id, data_buffer, 3};
  PrepareBuffer(prepare_buffer);
  
  last_map_ = current_map;
  last_map_send_time_ = now;
}

void RobotCommunicationNode::RealsensePointCallBack(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr
    realsense_pointcloud_msg) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_tmp(
    new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_tmp2(
    new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_result(
    new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::fromROSMsg(*realsense_pointcloud_msg, *pointcloud_tmp);

  try {
    pcl::transformPointCloud(*pointcloud_tmp, *pointcloud_tmp2,
                             camera_to_base_matrix);
    pcl::transformPointCloud(*pointcloud_tmp2, *pointcloud_result,
                             odom_to_local_matrix);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_INFO(this->get_logger(), "%s", ex.what());
    return;
  }

  sensor_msgs::msg::PointCloud2 totalRegisteredScan;
  pcl::toROSMsg(*pointcloud_result, totalRegisteredScan);
  totalRegisteredScan.header.stamp = realsense_pointcloud_msg->header.stamp;
  // Output in local map frame only
  totalRegisteredScan.header.frame_id = LOCAL_MAP_FRAME;

  std::vector<uint8_t> data_buffer =
    SerializeMsg<sensor_msgs::msg::PointCloud2>(totalRegisteredScan);
  SendBuffer prepare_buffer = {robot_id, data_buffer, 1};
  PrepareBuffer(prepare_buffer);
}

void RobotCommunicationNode::OdometryCallBack(
  const nav_msgs::msg::Odometry::ConstSharedPtr odometry_msg) {
  geometry_msgs::msg::TransformStamped odom_to_local_transform;
  odom_to_local_transform.transform.rotation =
    odometry_msg->pose.pose.orientation;
  odom_to_local_transform.transform.translation.x =
    odometry_msg->pose.pose.position.x;
  odom_to_local_transform.transform.translation.y =
    odometry_msg->pose.pose.position.y;
  odom_to_local_transform.transform.translation.z =
    odometry_msg->pose.pose.position.z;

  odom_to_local_matrix =
    tf2::transformToEigen(odom_to_local_transform.transform)
      .matrix()
      .cast<double>();
}

void RobotCommunicationNode::ImageCallBack(
  const sensor_msgs::msg::Image::ConstSharedPtr image_msg) {
  std::vector<uint8_t> data_buffer =
    SerializeMsg<sensor_msgs::msg::Image>(*image_msg);
  SendBuffer prepare_buffer = {robot_id, data_buffer, 4};
  PrepareBuffer(prepare_buffer);
}

void RobotCommunicationNode::PlanCallBack(
  const nav_msgs::msg::Path::ConstSharedPtr path_msg) {
  // 当收到新的规划路径时，发送到服务器
  SendNavigationPath(*path_msg);
}

void RobotCommunicationNode::SendNavigationStatus(int8_t status) {
  // 只在状态改变时发送
  if (status == last_nav_status_) {
    return;
  }
  
  // 发送导航状态到服务器，msg_type = 5
  std_msgs::msg::Int8 status_msg;
  status_msg.data = status;
  
  std::vector<uint8_t> data_buffer =
    SerializeMsg<std_msgs::msg::Int8>(status_msg);
  SendBuffer prepare_buffer = {robot_id, data_buffer, 5};
  PrepareBuffer(prepare_buffer);
  
  const char* status_names[] = {"UNKNOWN", "ACCEPTED", "EXECUTING", "CANCELING", 
                                "SUCCEEDED", "CANCELED", "ABORTED"};
  if (status >= 0 && status <= 6) {
    RCLCPP_INFO(this->get_logger(), "Navigation status changed: %s (%d)", 
                status_names[status], status);
  }
  
  last_nav_status_ = status;
}

void RobotCommunicationNode::SendNavigationPath(const nav_msgs::msg::Path& path) {
  // 如果路径为空，不发送
  if (path.poses.empty()) {
    RCLCPP_DEBUG(this->get_logger(), "Empty path, skipping send");
    return;
  }
  
  rclcpp::Time now = this->get_clock()->now();
  double time_since_last_send = (now - last_path_send_time_).seconds();
  
  // 检查发送间隔
  if (has_sent_path_ && time_since_last_send < PATH_SEND_INTERVAL) {
    RCLCPP_DEBUG(this->get_logger(), 
                "Path send interval not reached (%.2fs < %.2fs), skipping",
                time_since_last_send, PATH_SEND_INTERVAL);
    return;
  }
  
  // 检查路径是否有显著变化
  if (has_sent_path_) {
    // 比较路径长度
    if (path.poses.size() == last_sent_path_.poses.size()) {
      // 路径长度相同，检查路径点是否有显著变化
      bool has_significant_change = false;
      
      // 采样检查：检查起点、中点、终点
      std::vector<size_t> check_indices = {
        0,                                    // 起点
        path.poses.size() / 2,               // 中点
        path.poses.size() - 1                // 终点
      };
      
      for (size_t idx : check_indices) {
        if (idx < path.poses.size() && idx < last_sent_path_.poses.size()) {
          double dx = path.poses[idx].pose.position.x - 
                      last_sent_path_.poses[idx].pose.position.x;
          double dy = path.poses[idx].pose.position.y - 
                      last_sent_path_.poses[idx].pose.position.y;
          double dz = path.poses[idx].pose.position.z - 
                      last_sent_path_.poses[idx].pose.position.z;
          double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
          
          if (distance > PATH_CHANGE_THRESHOLD) {
            has_significant_change = true;
            RCLCPP_DEBUG(this->get_logger(), 
                        "Path point %zu changed by %.2fm", idx, distance);
            break;
          }
        }
      }
      
      if (!has_significant_change) {
        RCLCPP_DEBUG(this->get_logger(), 
                    "Path unchanged (checked %zu key points), skipping",
                    check_indices.size());
        return;
      }
    } else {
      RCLCPP_DEBUG(this->get_logger(), 
                  "Path length changed: %zu -> %zu poses",
                  last_sent_path_.poses.size(), path.poses.size());
    }
  }
  
  // 发送导航路径到服务器，msg_type = 6
  std::vector<uint8_t> data_buffer =
    SerializeMsg<nav_msgs::msg::Path>(path);
  SendBuffer prepare_buffer = {robot_id, data_buffer, 6};
  PrepareBuffer(prepare_buffer);
  
  // 更新缓存
  last_sent_path_ = path;
  last_path_send_time_ = now;
  has_sent_path_ = true;
  
  RCLCPP_INFO(this->get_logger(), "Sent NEW navigation path with %zu poses", 
              path.poses.size());
}

void RobotCommunicationNode::TFUpdateThread() {
  while (rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 10Hz
    
    auto now = this->get_clock()->now();
    
    // 1. 发送 odom -> base_link 变换
    // 这是里程计提供的实时位姿变换
    Eigen::Affine3d odom_to_base(odom_to_local_matrix);
    geometry_msgs::msg::TransformStamped odom_to_base_transform = 
      tf2::eigenToTransform(odom_to_base);
    odom_to_base_transform.header.frame_id = "odom";
    odom_to_base_transform.child_frame_id = BASE_FRAME;  // "base_link"
    odom_to_base_transform.header.stamp = now;
    
    std::vector<uint8_t> odom_base_buffer =
      SerializeMsg<geometry_msgs::msg::TransformStamped>(odom_to_base_transform);
    SendBuffer odom_base_msg = {robot_id, odom_base_buffer, 2};
    PrepareBuffer(odom_base_msg);
    
    // 2. 查询并发送 map -> odom 变换
    // 这是 AMCL 或其他定位系统提供的校正变换
    try {
      geometry_msgs::msg::TransformStamped map_to_odom_transform = 
        tf_buffer_->lookupTransform(
          LOCAL_MAP_FRAME,  // "map"
          "odom",
          tf2::TimePointZero);  // 获取最新可用的变换
      
      map_to_odom_transform.header.stamp = now;  // 更新时间戳
      
      std::vector<uint8_t> map_odom_buffer =
        SerializeMsg<geometry_msgs::msg::TransformStamped>(map_to_odom_transform);
      SendBuffer map_odom_msg = {robot_id, map_odom_buffer, 7};  // 使用新的消息类型 7
      PrepareBuffer(map_odom_msg);
    } catch (const tf2::TransformException& ex) {
      // 如果查询失败（例如 AMCL 还未初始化），记录警告但不中断
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Could not get map->odom transform: %s", ex.what());
    }
  }
}

void RobotCommunicationNode::NetworkSendThread() {
  while (rclcpp::ok()) {
    if (send_buffer_queue.empty()) {
      std::this_thread::sleep_for(std::chrono::nanoseconds(10));
      continue;
    }
    SendBuffer s_buffer = send_buffer_queue.front();
    send_buffer_queue.pop();
    if (sendto(sockfd, s_buffer.buffer.data(), s_buffer.buffer.size(), 0,
               (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Send failed!");
    }
  }
}

void RobotCommunicationNode::NetworkRecvThread() {
  int n, len = sizeof(server_addr);
  while (rclcpp::ok()) {
    std::vector<uint8_t> buffer_tmp(BUFFER_SIZE);
    n = recvfrom(sockfd, buffer_tmp.data(), BUFFER_SIZE, MSG_WAITFORONE,
                 (struct sockaddr*)&server_addr, (socklen_t*)&len);
    if (n < 0) {
      continue;
    }
    buffer_tmp.resize(n);
    if (recv_buffer_queue.size() >= MAX_BUFFER_QUEUE_SIZE) {
      continue;
    }
    recv_buffer_queue.push(buffer_tmp);
  }
}

void RobotCommunicationNode::PrepareBuffer(const SendBuffer& prepare_buffer) {
  const int total_packet =
    (prepare_buffer.buffer.size() + MAX_PACKET_SIZE - 1) / MAX_PACKET_SIZE;
  for (int i = 0; i < total_packet; i++) {
    uint8_t id = prepare_buffer.id;
    uint8_t type = prepare_buffer.msg_type;
    uint16_t idx = i;
    uint8_t max_idx = total_packet;
    std::vector<uint8_t> header(sizeof(uint32_t) + sizeof(uint8_t));
    std::memcpy(header.data(), &id, sizeof(id));
    std::memcpy(header.data() + sizeof(uint8_t), &type, sizeof(type));
    std::memcpy(header.data() + sizeof(uint16_t), &idx, sizeof(idx));
    std::memcpy(header.data() + sizeof(uint32_t), &max_idx, sizeof(max_idx));
    std::vector<uint8_t> packet;
    packet.insert(packet.end(), header.begin(), header.end());
    packet.insert(
      packet.end(), prepare_buffer.buffer.begin() + i * MAX_PACKET_SIZE,
      i == total_packet - 1
        ? prepare_buffer.buffer.end()
        : prepare_buffer.buffer.begin() + (i + 1) * MAX_PACKET_SIZE);
    SendBuffer s_buffer;
    s_buffer.buffer = packet;
    send_buffer_queue.push(s_buffer);
  }
}

void RobotCommunicationNode::ParseBufferThread() {
  int packet_idx = 0;
  int packet_type = -1;
  std::vector<uint8_t> buffer;
  while (rclcpp::ok()) {
    if (recv_buffer_queue.empty()) {
      std::this_thread::sleep_for(std::chrono::nanoseconds(10));
      continue;
    }
    std::vector<uint8_t> buffer_tmp = recv_buffer_queue.front();
    recv_buffer_queue.pop();

    uint8_t id, type, max_idx;
    uint16_t idx;
    std::memcpy(&id, buffer_tmp.data(), sizeof(id));
    std::memcpy(&type, buffer_tmp.data() + sizeof(uint8_t), sizeof(type));
    std::memcpy(&idx, buffer_tmp.data() + sizeof(uint16_t), sizeof(idx));
    std::memcpy(&max_idx, buffer_tmp.data() + sizeof(uint32_t),
                sizeof(max_idx));

    if (packet_type == -1) {
      packet_type = type;
    } else if (packet_type != type) {
      packet_idx = 0;
      packet_type = -1;
      buffer = std::vector<uint8_t>(0);
    }

    if (idx == 0) {
      packet_idx = 0;
      buffer = std::vector<uint8_t>(0);
    } else if (packet_idx != idx) {
      packet_idx = 0;
      packet_type = -1;
      buffer = std::vector<uint8_t>(0);
      continue;
    }

    packet_idx++;
    if (packet_idx == 1) {
      buffer.insert(buffer.begin(),
                    buffer_tmp.begin() + sizeof(uint32_t) + sizeof(uint8_t),
                    buffer_tmp.end());
    } else {
      buffer.insert(buffer.end(),
                    buffer_tmp.begin() + sizeof(uint32_t) + sizeof(uint8_t),
                    buffer_tmp.end());
    }

    if (packet_idx != max_idx) {
      continue;
    }
    try {
      if (type == 0) {  // Way Point
        RCLCPP_INFO(this->get_logger(), "Received waypoint packet from network");
        geometry_msgs::msg::PointStamped way_point =
          DeserializeMsg<geometry_msgs::msg::PointStamped>(buffer);
        WayPointCallBack(
          std::make_shared<geometry_msgs::msg::PointStamped>(way_point));
      }
    } catch (const std::exception& ex) {
      RCLCPP_ERROR(this->get_logger(), 
                   "Exception in ParseBufferThread: %s", ex.what());
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), 
                   "Unknown exception in ParseBufferThread");
    }

    packet_idx = 0;
    packet_type = -1;
    buffer = std::vector<uint8_t>(0);
  }
}

// Serialization
template <class T>
std::vector<uint8_t> RobotCommunicationNode::SerializeMsg(const T& msg) {
  rclcpp::SerializedMessage serialized_msg;
  rclcpp::Serialization<T> serializer;
  serializer.serialize_message(&msg, &serialized_msg);

  std::vector<uint8_t> buffer_tmp(serialized_msg.size());
  std::memcpy(buffer_tmp.data(),
              serialized_msg.get_rcl_serialized_message().buffer,
              serialized_msg.size());

  return buffer_tmp;
}

// Deserialization
template <class T>
T RobotCommunicationNode::DeserializeMsg(const std::vector<uint8_t>& data) {
  rclcpp::SerializedMessage serialized_msg;
  rclcpp::Serialization<T> serializer;

  serialized_msg.reserve(data.size());
  std::memcpy(serialized_msg.get_rcl_serialized_message().buffer, data.data(),
              data.size());
  serialized_msg.get_rcl_serialized_message().buffer_length = data.size();

  T msg;
  serializer.deserialize_message(&serialized_msg, &msg);

  return msg;
}

}  // namespace robot_communication

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(robot_communication::RobotCommunicationNode)
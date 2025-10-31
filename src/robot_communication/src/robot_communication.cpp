#include <arpa/inet.h>
#include <netinet/in.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rmw/qos_profiles.h>
#include <rmw/types.h>
#include <sys/socket.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/convert.h>
#include <tf2/time.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>  // already present

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp/time.hpp>  // 添加：提供 rclcpp::Time::to_msg()
#include <string>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <vector>

#include "robot_communication/robot_communication.hpp"

#define MAX_PACKET_SIZE 64000
#define BUFFER_SIZE 65535
#define MAX_BUFFER_QUEUE_SIZE 256

namespace robot_communication {
RobotCommunicationNode::RobotCommunicationNode(
  const rclcpp::NodeOptions &options)
    : Node("robot_communication", options) {
  this->declare_parameter<int>("robot_count", 3);
  this->declare_parameter<int>("network_port", 12130);
  this->declare_parameter<std::string>("network_ip", "192.168.31.207");
  this->declare_parameter<bool>("update_timestamp_on_receive", true);

  this->get_parameter("robot_count", robot_count);
  this->get_parameter("network_port", port);
  this->get_parameter("network_ip", ip);
  this->get_parameter("update_timestamp_on_receive", update_timestamp_on_receive);
  
  RCLCPP_INFO(this->get_logger(), "Timestamp update on receive: %s", 
              update_timestamp_on_receive ? "enabled" : "disabled");

  rclcpp::QoS qos(rclcpp::KeepLast(10));
  qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
  qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  qos.history(rclcpp::HistoryPolicy::KeepLast);

  for (int i = 1; i <= robot_count; i++) {
    registered_scan_pub_[i] =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/robot_" + std::to_string(i) + "/total_registered_scan", 5);
    map_pub_[i] =
      this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/robot_" + std::to_string(i) + "/map", qos);
    realsense_pointcloud_pub_[i] =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/robot_" + std::to_string(i) + "/realsense_pointcloud", 5);
    image_pub_[i] =
      this->create_publisher<sensor_msgs::msg::Image>(
        "/robot_" + std::to_string(i) + "/image_raw", 5);
    nav2_status_pub_[i] =
      this->create_publisher<std_msgs::msg::Int8>(
        "/robot_" + std::to_string(i) + "/nav2_status", 5);
    nav2_path_pub_[i] =
      this->create_publisher<nav_msgs::msg::Path>(
        "/robot_" + std::to_string(i) + "/nav2_path", 5);
    way_point_sub_[i] =
      this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/robot_" + std::to_string(i) + "/way_point", 2,
        [this, i](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
          WayPointCallBack(msg, i);
        });
  }

  // initialize class member TransformBroadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  // 初始化缓存的导航状态（所有机器人初始化为 UNKNOWN）
  for (int i = 0; i < MAX_ROBOT_COUNT; i++) {
    cached_nav_status_[i].data = 0;  // UNKNOWN
  }

  // 创建定时器，以 5Hz (0.2秒) 频率发布缓存的导航状态
  status_publish_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(200),
    std::bind(&RobotCommunicationNode::PublishCachedStatusCallback, this));

  // 创建目标点重传定时器
  waypoint_resend_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(WAYPOINT_RESEND_INTERVAL_MS),
    std::bind(&RobotCommunicationNode::WaypointResendCallback, this));

  InitServer();
}

RobotCommunicationNode::~RobotCommunicationNode() {
  if (send_thread_.joinable()) {
    send_thread_.join();
  }
  if (recv_thread_.joinable()) {
    recv_thread_.join();
  }
  for (int i = 1; i <= robot_count; i++) {
    if (parse_buffer_thread_[i].joinable()) {
      parse_buffer_thread_[i].join();
    }
  }
  close(sockfd);
}

// initialize the socket server
void RobotCommunicationNode::InitServer() {
  if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    RCLCPP_ERROR(this->get_logger(), "Socket creation failed!");
    return;
  }

  memset(&server_addr, 0, sizeof(server_addr));
  memset(&saved_client_addr, 0, sizeof(saved_client_addr));

  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(port);
  server_addr.sin_addr.s_addr = inet_addr(ip.c_str());

  if (bind(sockfd, (const struct sockaddr *)&server_addr, sizeof(server_addr)) <
      0) {
    RCLCPP_ERROR(this->get_logger(), "Bind failed!");
    close(sockfd);
    return;
  }

  send_thread_ = std::thread(&RobotCommunicationNode::NetworkSendThread, this);
  for (int i = 1; i <= robot_count; i++) {
    parse_buffer_thread_[i] =
      std::thread(&RobotCommunicationNode::ParseBufferThread, this, i);
  }
  recv_thread_ = std::thread(&RobotCommunicationNode::NetworkRecvThread, this);
  RCLCPP_INFO(this->get_logger(), "Server start at ip: %s, port: %d",
              ip.c_str(), port);
}

void RobotCommunicationNode::WayPointCallBack(
  const geometry_msgs::msg::PointStamped::ConstSharedPtr way_point_msg,
  const int robot_id) {
  // 记录接收到新目标点
  RCLCPP_INFO(this->get_logger(), 
              "========================================");
  RCLCPP_INFO(this->get_logger(), 
              "🎯 NEW WAYPOINT RECEIVED for robot_%d", robot_id);
  RCLCPP_INFO(this->get_logger(), 
              "   Position: (%.3f, %.3f, %.3f)", 
              way_point_msg->point.x, 
              way_point_msg->point.y, 
              way_point_msg->point.z);
  RCLCPP_INFO(this->get_logger(), 
              "   Frame: %s -> map (auto-converted)", 
              way_point_msg->header.frame_id.c_str());

  // Check if we have a saved address for this robot
  if (saved_client_addr[robot_id].sin_addr.s_addr == 0) {
    RCLCPP_WARN(this->get_logger(), 
                "   ⚠️  Robot_%d not connected yet, waypoint cached for later delivery", 
                robot_id);
  } else {
    // 记录目标客户端地址
    char addr_str[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &(saved_client_addr[robot_id].sin_addr), addr_str, INET_ADDRSTRLEN);
    uint16_t target_port = ntohs(saved_client_addr[robot_id].sin_port);
    RCLCPP_INFO(this->get_logger(), 
                "   📡 Target: %s:%u", addr_str, target_port);
  }

  // Create a copy and force frame_id to be "map"
  geometry_msgs::msg::PointStamped msg = *way_point_msg;
  msg.header.frame_id = "map";

  // 重置导航状态缓存为 UNKNOWN（防止旧状态干扰新目标点判断）
  cached_nav_status_[robot_id].data = 0;  // 0 = UNKNOWN
  RCLCPP_INFO(this->get_logger(), 
              "   🔄 Reset nav status cache to UNKNOWN");

  // 缓存目标点，用于重传
  {
    std::lock_guard<std::mutex> lock(waypoint_mutex_[robot_id]);
    cached_waypoints_[robot_id].waypoint = msg;
    cached_waypoints_[robot_id].retry_count = 0;
    cached_waypoints_[robot_id].has_waypoint = true;
    cached_waypoints_[robot_id].last_send_time = std::chrono::steady_clock::now();
  }

  // 立即发送一次
  std::vector<uint8_t> data_buffer =
    SerializeMsg<geometry_msgs::msg::PointStamped>(msg);
  SendBuffer prepare_buffer = {robot_id, data_buffer, 0};
  PrepareBuffer(prepare_buffer);
  
  RCLCPP_INFO(this->get_logger(), 
              "   ✅ Waypoint sent (size: %zu bytes, will retry up to %d times if needed)", 
              data_buffer.size(), MAX_WAYPOINT_RETRY);
  RCLCPP_INFO(this->get_logger(), 
              "========================================");
}

void RobotCommunicationNode::NetworkSendThread() {
  while (rclcpp::ok()) {
    SendBuffer s_buffer;
    {
      std::lock_guard<std::mutex> lock(send_buffer_mutex_);
      if (send_buffer_queue.empty()) {
        // 在锁外 sleep 以避免长时间持有锁
      } else {
        s_buffer = send_buffer_queue.front();
        send_buffer_queue.pop();
      }
    }
    
    // 如果队列为空（id为-1表示没有取到数据），则短暂sleep
    if (s_buffer.id == -1) {
      std::this_thread::sleep_for(std::chrono::nanoseconds(10));
      continue;
    }
    if (saved_client_addr[s_buffer.id].sin_addr.s_addr == 0) {
      RCLCPP_WARN(this->get_logger(), "No saved address for robot_%d, skipping send", s_buffer.id);
      continue;
    }
    
    // Log the target address
    char addr_str[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &(saved_client_addr[s_buffer.id].sin_addr), addr_str, INET_ADDRSTRLEN);
    uint16_t target_port = ntohs(saved_client_addr[s_buffer.id].sin_port);
    RCLCPP_DEBUG(this->get_logger(), "Sending to robot_%d at %s:%d, buffer size: %zu", 
                s_buffer.id, addr_str, target_port, s_buffer.buffer.size());
    
    ssize_t sent = sendto(sockfd, s_buffer.buffer.data(), s_buffer.buffer.size(), 0,
               (const struct sockaddr *)&saved_client_addr[s_buffer.id],
               sizeof(saved_client_addr[s_buffer.id]));
    if (sent < 0) {
      RCLCPP_ERROR(this->get_logger(), "Send failed for robot_%d! errno: %d (%s)", 
                   s_buffer.id, errno, strerror(errno));
    } else {
      RCLCPP_DEBUG(this->get_logger(), "Successfully sent %zd bytes to robot_%d", sent, s_buffer.id);
    }
  }
}

void RobotCommunicationNode::NetworkRecvThread() {
  struct sockaddr_in client_addr;
  memset(&client_addr, 0, sizeof(client_addr));
  int n, len = sizeof(client_addr);
  while (rclcpp::ok()) {
    std::vector<uint8_t> buffer_tmp(BUFFER_SIZE);
    n = recvfrom(sockfd, buffer_tmp.data(), BUFFER_SIZE, MSG_WAITFORONE,
                 (struct sockaddr *)&client_addr, (socklen_t *)&len);
    if (n < 0) {
      continue;
    }
    buffer_tmp.resize(n);

    uint8_t id;
    std::memcpy(&id, buffer_tmp.data(), sizeof(id));
    
    // 检查地址是否发生变化并记录
    if (saved_client_addr[id].sin_addr.s_addr != 0) {
      // 已有地址，检查是否变化
      if (saved_client_addr[id].sin_addr.s_addr != client_addr.sin_addr.s_addr ||
          saved_client_addr[id].sin_port != client_addr.sin_port) {
        
        // 记录地址变化
        char old_addr_str[INET_ADDRSTRLEN];
        char new_addr_str[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &(saved_client_addr[id].sin_addr), old_addr_str, INET_ADDRSTRLEN);
        inet_ntop(AF_INET, &(client_addr.sin_addr), new_addr_str, INET_ADDRSTRLEN);
        uint16_t old_port = ntohs(saved_client_addr[id].sin_port);
        uint16_t new_port = ntohs(client_addr.sin_port);
        
        RCLCPP_WARN(this->get_logger(), 
                    "Robot_%u address changed: %s:%u -> %s:%u",
                    id, old_addr_str, old_port, new_addr_str, new_port);
      }
    } else {
      // 第一次接收到该机器人的消息
      char addr_str[INET_ADDRSTRLEN];
      inet_ntop(AF_INET, &(client_addr.sin_addr), addr_str, INET_ADDRSTRLEN);
      uint16_t port = ntohs(client_addr.sin_port);
      
      RCLCPP_INFO(this->get_logger(), 
                  "Robot_%u connected from %s:%u",
                  id, addr_str, port);
    }
    
    // 始终更新为最新地址
    saved_client_addr[id] = client_addr;
    
    // 使用互斥锁保护接收队列
    {
      std::lock_guard<std::mutex> lock(recv_buffer_mutex_[id]);
      if (recv_buffer_queue[id].size() >= MAX_BUFFER_QUEUE_SIZE) {
        continue;
      }
      recv_buffer_queue[id].push(buffer_tmp);
    }
  }
}

void RobotCommunicationNode::PrepareBuffer(const SendBuffer &prepare_buffer) {
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
    s_buffer.id = prepare_buffer.id;
    
    // 使用互斥锁保护队列操作
    {
      std::lock_guard<std::mutex> lock(send_buffer_mutex_);
      send_buffer_queue.push(s_buffer);
    }
  }
}

void RobotCommunicationNode::ParseBufferThread(const int robot_id) {
  int packet_idx = 0;
  int packet_type = -1;
  std::vector<uint8_t> buffer;
  rclcpp::Time last_transform_time(0, 0, RCL_ROS_TIME); // Track last transform time
  rclcpp::Time last_transform_log_time(0, 0, RCL_ROS_TIME); // Track last transform log time

  while (rclcpp::ok()) {
    std::vector<uint8_t> buffer_tmp;
    {
      std::lock_guard<std::mutex> lock(recv_buffer_mutex_[robot_id]);
      if (recv_buffer_queue[robot_id].empty()) {
        // 在锁外 sleep 以避免长时间持有锁
      } else {
        buffer_tmp = recv_buffer_queue[robot_id].front();
        recv_buffer_queue[robot_id].pop();
      }
    }
    
    // 如果队列为空，则短暂sleep
    if (buffer_tmp.empty()) {
      std::this_thread::sleep_for(std::chrono::nanoseconds(10));
      continue;
    }

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
      if (type == 0) {  // PointCloud2
        sensor_msgs::msg::PointCloud2 totalRegisteredScan =
          DeserializeMsg<sensor_msgs::msg::PointCloud2>(buffer);
        
        // 更新时间戳为接收时间
        if (update_timestamp_on_receive) {
          uint64_t now_ns = this->get_clock()->now().nanoseconds();
          totalRegisteredScan.header.stamp.sec = static_cast<uint32_t>(now_ns / 1000000000ULL);
          totalRegisteredScan.header.stamp.nanosec = static_cast<uint32_t>(now_ns % 1000000000ULL);
        }
        
        // Prefix frame_id with robot_{id}/ before publishing
        const std::string prefix = "robot_" + std::to_string(id) + "/";
        if (!totalRegisteredScan.header.frame_id.empty() &&
            totalRegisteredScan.header.frame_id.rfind(prefix, 0) != 0) {
          totalRegisteredScan.header.frame_id = prefix + totalRegisteredScan.header.frame_id;
        }
        registered_scan_pub_[id]->publish(totalRegisteredScan);
      } else if (type == 1) {  // Realsense PointCloud2
        sensor_msgs::msg::PointCloud2 realsense_pointcloud =
          DeserializeMsg<sensor_msgs::msg::PointCloud2>(buffer);
        
        // 更新时间戳为接收时间
        if (update_timestamp_on_receive) {
          uint64_t now_ns = this->get_clock()->now().nanoseconds();
          realsense_pointcloud.header.stamp.sec = static_cast<uint32_t>(now_ns / 1000000000ULL);
          realsense_pointcloud.header.stamp.nanosec = static_cast<uint32_t>(now_ns % 1000000000ULL);
        }
        
        const std::string prefix = "robot_" + std::to_string(id) + "/";
        if (!realsense_pointcloud.header.frame_id.empty() &&
            realsense_pointcloud.header.frame_id.rfind(prefix, 0) != 0) {
          realsense_pointcloud.header.frame_id = prefix + realsense_pointcloud.header.frame_id;
        }
        realsense_pointcloud_pub_[id]->publish(realsense_pointcloud);
      } else if (type == 2) {  // Transform
        geometry_msgs::msg::TransformStamped transformStamped =
          DeserializeMsg<geometry_msgs::msg::TransformStamped>(buffer);

        // 更新时间戳为接收时间（如果启用）
        if (update_timestamp_on_receive || 
            (transformStamped.header.stamp.sec == 0 && transformStamped.header.stamp.nanosec == 0)) {
          uint64_t now_ns = this->get_clock()->now().nanoseconds();
          transformStamped.header.stamp.sec = static_cast<uint32_t>(now_ns / 1000000000ULL);
          transformStamped.header.stamp.nanosec = static_cast<uint32_t>(now_ns % 1000000000ULL);
          if (!update_timestamp_on_receive) {
            RCLCPP_WARN(this->get_logger(),
                        "TransformStamped received without timestamp, adding current time");
          }
        }

        // 可选：如果 frame_id 或 child_frame_id 为空，记录警告（便于排查）
        if (transformStamped.header.frame_id.empty() ||
            transformStamped.child_frame_id.empty()) {
          RCLCPP_WARN(this->get_logger(),
                      "Received TransformStamped with empty frame ids (parent='%s', child='%s')",
                      transformStamped.header.frame_id.c_str(),
                      transformStamped.child_frame_id.c_str());
        }

        // Prefix parent/child frames with robot_{id}/ before broadcasting TF
        const std::string prefix = "robot_" + std::to_string(id) + "/";
        if (!transformStamped.header.frame_id.empty() &&
            transformStamped.header.frame_id.rfind(prefix, 0) != 0) {
          transformStamped.header.frame_id = prefix + transformStamped.header.frame_id;
        }
        if (!transformStamped.child_frame_id.empty() &&
            transformStamped.child_frame_id.rfind(prefix, 0) != 0) {
          transformStamped.child_frame_id = prefix + transformStamped.child_frame_id;
        }

        // 提高发送频率：检查是否超过阈值时间间隔（5Hz = 0.2秒）
        rclcpp::Time now = this->get_clock()->now();
        if ((now - last_transform_time).seconds() >= 0.2) {
          if (tf_broadcaster_) {
            tf_broadcaster_->sendTransform(transformStamped);
            
            // 降低日志输出频率到 0.5Hz（2秒间隔）
            if ((now - last_transform_log_time).seconds() >= 2.0) {
              RCLCPP_DEBUG(this->get_logger(),
                          "Sent transform (robot id=%u) parent='%s' child='%s' time=%u.%u",
                          id,
                          transformStamped.header.frame_id.c_str(),
                          transformStamped.child_frame_id.c_str(),
                          transformStamped.header.stamp.sec,
                          transformStamped.header.stamp.nanosec);
              last_transform_log_time = now;
            }
          } else {
            RCLCPP_WARN(this->get_logger(), "tf_broadcaster_ is not initialized");
          }
          last_transform_time = now;
        }
      } else if (type == 3) {
        nav_msgs::msg::OccupancyGrid map =
          DeserializeMsg<nav_msgs::msg::OccupancyGrid>(buffer);
        
        // 更新时间戳为接收时间
        if (update_timestamp_on_receive) {
          uint64_t now_ns = this->get_clock()->now().nanoseconds();
          map.header.stamp.sec = static_cast<uint32_t>(now_ns / 1000000000ULL);
          map.header.stamp.nanosec = static_cast<uint32_t>(now_ns % 1000000000ULL);
        }
        
        const std::string prefix = "robot_" + std::to_string(id) + "/";
        if (!map.header.frame_id.empty() &&
            map.header.frame_id.rfind(prefix, 0) != 0) {
          map.header.frame_id = prefix + map.header.frame_id;
        }
        
        // 检查是否为增量地图（width的最高位为1表示这是负数，即增量地图）
        int32_t signed_width = static_cast<int32_t>(map.info.width);
        if (signed_width < 0) {
          // 这是增量地图
          int32_t num_changes = -signed_width;
          
          if (!map_initialized_[id]) {
            RCLCPP_WARN(this->get_logger(), 
                        "Received delta map for robot_%u but no base map exists, ignoring", id);
            packet_idx = 0;
            packet_type = -1;
            buffer = std::vector<uint8_t>(0);
            continue;
          }
          
          // 从height字段恢复原始宽度，从缓存恢复高度
          uint32_t original_width = map.info.height;  // 客户端临时存储在height中
          map.info.width = original_width;
          map.info.height = cached_maps_[id].info.height;
          
          // 解析增量数据并应用到缓存的地图
          nav_msgs::msg::OccupancyGrid updated_map = cached_maps_[id];
          updated_map.header = map.header;  // 更新时间戳
          
          size_t data_idx = 0;
          int changes_applied = 0;
          while (data_idx + 4 < map.data.size()) {
            // 读取 4 字节索引
            int32_t idx = static_cast<int32_t>(
              static_cast<uint8_t>(map.data[data_idx]) |
              (static_cast<uint8_t>(map.data[data_idx + 1]) << 8) |
              (static_cast<uint8_t>(map.data[data_idx + 2]) << 16) |
              (static_cast<uint8_t>(map.data[data_idx + 3]) << 24)
            );
            int8_t value = map.data[data_idx + 4];
            
            if (idx >= 0 && idx < static_cast<int32_t>(updated_map.data.size())) {
              updated_map.data[idx] = value;
              changes_applied++;
            }
            
            data_idx += 5;
          }
          
          RCLCPP_DEBUG(this->get_logger(), 
                      "Applied %d delta changes to robot_%u map (expected %d)",
                      changes_applied, id, num_changes);
          
          // 更新缓存并发布
          cached_maps_[id] = updated_map;
          map_pub_[id]->publish(updated_map);
          
        } else {
          // 这是完整地图
          RCLCPP_DEBUG(this->get_logger(), 
                      "Received and published full map for robot_%u (size: %dx%d)",
                      id, map.info.width, map.info.height);
          
          cached_maps_[id] = map;
          map_initialized_[id] = true;
          map_pub_[id]->publish(map);
        }
      } else if (type == 4) {
        sensor_msgs::msg::Image image =
          DeserializeMsg<sensor_msgs::msg::Image>(buffer);
        
        // 更新时间戳为接收时间
        if (update_timestamp_on_receive) {
          uint64_t now_ns = this->get_clock()->now().nanoseconds();
          image.header.stamp.sec = static_cast<uint32_t>(now_ns / 1000000000ULL);
          image.header.stamp.nanosec = static_cast<uint32_t>(now_ns % 1000000000ULL);
        }
        
        const std::string prefix = "robot_" + std::to_string(id) + "/";
        if (!image.header.frame_id.empty() &&
            image.header.frame_id.rfind(prefix, 0) != 0) {
          image.header.frame_id = prefix + image.header.frame_id;
        }
        image_pub_[id]->publish(image);
      } else if (type == 5) {  // Nav2 Status
        std_msgs::msg::Int8 nav2_status =
          DeserializeMsg<std_msgs::msg::Int8>(buffer);
        
        // 只更新缓存，不立即发布，由定时器统一发布
        cached_nav_status_[id] = nav2_status;
        
        const char* status_names[] = {"UNKNOWN", "ACCEPTED", "EXECUTING", "CANCELING", 
                                      "SUCCEEDED", "CANCELED", "ABORTED"};
        int8_t status = nav2_status.data;
        if (status >= 0 && status <= 6) {
          RCLCPP_INFO(this->get_logger(), 
                      "Updated cached status for robot_%u: %s (%d)",
                      id, status_names[status], status);
        }
        
        // 如果状态变为 ACCEPTED、EXECUTING 或 ABORTED，说明目标点已被接收，清除缓存的重传任务
        // ACCEPTED=1: 目标已接受
        // EXECUTING=2: 正在执行
        // ABORTED=6: 已中止（说明收到了目标但执行失败）
        if (status == 1 || status == 2 || status == 6) {
          std::lock_guard<std::mutex> lock(waypoint_mutex_[id]);
          if (cached_waypoints_[id].has_waypoint) {
            // 记录之前缓存的目标点坐标
            const auto& wp = cached_waypoints_[id].waypoint;
            RCLCPP_INFO(this->get_logger(), 
                        "✅ Robot_%u acknowledged waypoint (%.2f, %.2f, %.2f) due to status=%s, "
                        "stopping retries (retry_count=%u)",
                        id, wp.point.x, wp.point.y, wp.point.z,
                        status_names[status], cached_waypoints_[id].retry_count);
            cached_waypoints_[id].has_waypoint = false;  // 停止重传
          }
        }
      } else if (type == 6) {  // Nav2 Path
        nav_msgs::msg::Path nav2_path =
          DeserializeMsg<nav_msgs::msg::Path>(buffer);
        
        // 更新时间戳为接收时间
        if (update_timestamp_on_receive) {
          uint64_t now_ns = this->get_clock()->now().nanoseconds();
          nav2_path.header.stamp.sec = static_cast<uint32_t>(now_ns / 1000000000ULL);
          nav2_path.header.stamp.nanosec = static_cast<uint32_t>(now_ns % 1000000000ULL);
        }
        
        // Prefix frame_id
        const std::string prefix = "robot_" + std::to_string(id) + "/";
        if (!nav2_path.header.frame_id.empty() &&
            nav2_path.header.frame_id.rfind(prefix, 0) != 0) {
          nav2_path.header.frame_id = prefix + nav2_path.header.frame_id;
        }
        
        nav2_path_pub_[id]->publish(nav2_path);
        RCLCPP_DEBUG(this->get_logger(), 
                    "Received and published Nav2 path for robot_%u (%zu poses)",
                    id, nav2_path.poses.size());
      }
    } catch (...) {
    }

    packet_idx = 0;
    packet_type = -1;
    buffer = std::vector<uint8_t>(0);
  }
}

// Serialization
template <class T>
std::vector<uint8_t> RobotCommunicationNode::SerializeMsg(const T &msg) {
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
T RobotCommunicationNode::DeserializeMsg(const std::vector<uint8_t> &data) {
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

// 定时发布缓存的导航状态
void RobotCommunicationNode::PublishCachedStatusCallback() {
  for (int i = 1; i <= robot_count; i++) {
    nav2_status_pub_[i]->publish(cached_nav_status_[i]);
  }
}

// 目标点重传回调
void RobotCommunicationNode::WaypointResendCallback() {
  auto now = std::chrono::steady_clock::now();
  
  // 收集需要重传的目标点
  std::vector<std::pair<int, geometry_msgs::msg::PointStamped>> waypoints_to_send;
  
  for (int i = 1; i <= robot_count; i++) {
    std::lock_guard<std::mutex> lock(waypoint_mutex_[i]);
    
    // 检查是否有待发送的目标点
    if (!cached_waypoints_[i].has_waypoint) {
      continue;
    }
    
    // 检查是否超过最大重传次数
    if (cached_waypoints_[i].retry_count >= MAX_WAYPOINT_RETRY) {
      RCLCPP_WARN(this->get_logger(), 
                  "Waypoint for robot_%d reached max retries (%d), giving up. Point: (%.2f, %.2f, %.2f)",
                  i, MAX_WAYPOINT_RETRY,
                  cached_waypoints_[i].waypoint.point.x,
                  cached_waypoints_[i].waypoint.point.y,
                  cached_waypoints_[i].waypoint.point.z);
      cached_waypoints_[i].has_waypoint = false;  // 放弃重传
      continue;
    }
    
    // 检查是否到了重传时间
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
      now - cached_waypoints_[i].last_send_time
    ).count();
    
    if (elapsed < WAYPOINT_RESEND_INTERVAL_MS) {
      continue;  // 还没到重传时间
    }
    
    // 检查客户端地址是否可用
    if (saved_client_addr[i].sin_addr.s_addr == 0) {
      RCLCPP_DEBUG(this->get_logger(), "No saved address for robot_%d, skipping waypoint resend", i);
      continue;
    }
    
    // 只有retry_count > 0时才是重传（第一次发送在WayPointCallBack中完成）
    if (cached_waypoints_[i].retry_count > 0) {
      // 收集需要发送的目标点
      waypoints_to_send.push_back({i, cached_waypoints_[i].waypoint});
      
      RCLCPP_INFO(this->get_logger(), 
                  "🔄 Resending waypoint to robot_%d (retry %d/%d): (%.2f, %.2f, %.2f)",
                  i, cached_waypoints_[i].retry_count, MAX_WAYPOINT_RETRY,
                  cached_waypoints_[i].waypoint.point.x,
                  cached_waypoints_[i].waypoint.point.y,
                  cached_waypoints_[i].waypoint.point.z);
    }
    
    // 更新重传状态
    cached_waypoints_[i].retry_count++;
    cached_waypoints_[i].last_send_time = now;
  }
  
  // 在锁外发送（避免死锁）
  for (const auto& [robot_id, waypoint] : waypoints_to_send) {
    if (saved_client_addr[robot_id].sin_addr.s_addr != 0) {
      std::vector<uint8_t> data_buffer =
        SerializeMsg<geometry_msgs::msg::PointStamped>(waypoint);
      SendBuffer prepare_buffer = {robot_id, data_buffer, 0};
      PrepareBuffer(prepare_buffer);
      
      char addr_str[INET_ADDRSTRLEN];
      inet_ntop(AF_INET, &(saved_client_addr[robot_id].sin_addr), addr_str, INET_ADDRSTRLEN);
      uint16_t target_port = ntohs(saved_client_addr[robot_id].sin_port);
      RCLCPP_INFO(this->get_logger(), 
                  "   📤 Waypoint resent to %s:%u (size: %zu bytes)",
                  addr_str, target_port, data_buffer.size());
    }
  }
}

}  // namespace robot_communication

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(robot_communication::RobotCommunicationNode)
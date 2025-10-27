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

  for (int i = 0; i < robot_count; i++) {
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
    way_point_sub_[i] =
      this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/robot_" + std::to_string(i) + "/way_point", 2,
        [this, i](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
          WayPointCallBack(msg, i);
        });
  }

  // initialize class member TransformBroadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  InitServer();
}

RobotCommunicationNode::~RobotCommunicationNode() {
  if (send_thread_.joinable()) {
    send_thread_.join();
  }
  if (recv_thread_.joinable()) {
    recv_thread_.join();
  }
  for (int i = 0; i < robot_count; i++) {
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
  for (int i = 0; i < robot_count; i++) {
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
  RCLCPP_INFO(this->get_logger(), "Sending waypoint message for robot_%d: x=%.2f, y=%.2f, z=%.2f", 
              robot_id, way_point_msg->point.x, way_point_msg->point.y, way_point_msg->point.z);

  // Remove prefix `robot_{id}/` from frame_id before sending to robot
  geometry_msgs::msg::PointStamped msg = *way_point_msg;
  const std::string prefix = "robot_" + std::to_string(robot_id) + "/";
  if (!msg.header.frame_id.empty()) {
    if (msg.header.frame_id.rfind(prefix, 0) == 0) { // starts with prefix
      msg.header.frame_id = msg.header.frame_id.substr(prefix.size());
    }
  }

  std::vector<uint8_t> data_buffer =
    SerializeMsg<geometry_msgs::msg::PointStamped>(*way_point_msg);
  SendBuffer prepare_buffer = {robot_id, data_buffer, 0};
  PrepareBuffer(prepare_buffer);
}

void RobotCommunicationNode::NetworkSendThread() {
  while (rclcpp::ok()) {
    if (send_buffer_queue.empty()) {
      std::this_thread::sleep_for(std::chrono::nanoseconds(10));
      continue;
    }
    SendBuffer s_buffer = send_buffer_queue.front();
    send_buffer_queue.pop();
    if (saved_client_addr[s_buffer.id].sin_addr.s_addr == 0) {
      continue;
    }
    if (sendto(sockfd, s_buffer.buffer.data(), s_buffer.buffer.size(), 0,
               (const struct sockaddr *)&saved_client_addr[s_buffer.id],
               sizeof(saved_client_addr[s_buffer.id])) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Send failed!");
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
    saved_client_addr[id] = client_addr;
    if (recv_buffer_queue[id].size() >= MAX_BUFFER_QUEUE_SIZE) {
      continue;
    }
    recv_buffer_queue[id].push(buffer_tmp);
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
    send_buffer_queue.push(s_buffer);
  }
}

void RobotCommunicationNode::ParseBufferThread(const int robot_id) {
  int packet_idx = 0;
  int packet_type = -1;
  std::vector<uint8_t> buffer;
  rclcpp::Time last_transform_time(0, 0, RCL_ROS_TIME); // Track last transform time
  rclcpp::Time last_transform_log_time(0, 0, RCL_ROS_TIME); // Track last transform log time

  while (rclcpp::ok()) {
    if (recv_buffer_queue[robot_id].empty()) {
      std::this_thread::sleep_for(std::chrono::nanoseconds(10));
      continue;
    }
    std::vector<uint8_t> buffer_tmp = recv_buffer_queue[robot_id].front();
    recv_buffer_queue[robot_id].pop();

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
              RCLCPP_INFO(this->get_logger(),
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
        map_pub_[id]->publish(map);
        RCLCPP_INFO(this->get_logger(), "Received and published map for robot_%u", id);
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
}  // namespace robot_communication

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(robot_communication::RobotCommunicationNode)
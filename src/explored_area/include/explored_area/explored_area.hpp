#ifndef EXPLORED_AREA
#define EXPLORED_AREA

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/impl/point_types.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>

#define MAX_ROBOT_COUNT 5

namespace explored_area {
class ExploredAreaNode : public rclcpp::Node {
 public:
  ExploredAreaNode(const rclcpp::NodeOptions& options);

 private:
  int robot_count = 3;
  int exploredAreaDisplayCount = 0;
  int exploredAreaDisplayInterval = 1;
  double exploredVolumeVoxelSize = 0.05;
  double exploredAreaVoxelSize = 0.05;
  float exploredVolume = 0;
  float travelingDis = 0;

  pcl::VoxelGrid<pcl::PointXYZI> exploredVolumeDwzFilter;
  pcl::VoxelGrid<pcl::PointXYZI> exploredAreaDwzFilter;
  pcl::VoxelGrid<pcl::PointXYZRGB> exploredRGBVolumeDwzFilter;
  pcl::VoxelGrid<pcl::PointXYZRGB> exploredRGBAreaDwzFilter;

  pcl::PointCloud<pcl::PointXYZI>::Ptr exploredVolumeCloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr exploredVolumeCloud2;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr exploredRGBVolumeCloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr exploredRGBVolumeCloud2;
  pcl::PointCloud<pcl::PointXYZI>::Ptr exploredAreaCloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr exploredAreaCloud2;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr exploredRGBAreaCloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr exploredRGBAreaCloud2;

  void RegisteredScanCallBack(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr registered_scan_msg);
  void RealsenseScanCallBack(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr realsense_scan_msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
    registered_scan_sub_[MAX_ROBOT_COUNT];
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
    realsense_scan_sub_[MAX_ROBOT_COUNT];

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    explored_area_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    explored_rgb_area_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr explored_volume_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr traveling_dis_pub_;
};

}  // namespace explored_area

#endif
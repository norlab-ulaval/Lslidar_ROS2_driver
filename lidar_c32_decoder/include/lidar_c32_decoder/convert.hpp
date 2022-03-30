/*
 * The driver is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the driver.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef CONVERT_HPP_
#define CONVERT_HPP_

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "rawdata.hpp"
#include "sensor_msgs/msg/time_reference.hpp"
#include "pcl_conversions/pcl_conversions.h"

namespace lidar_c32_decoder
{
class Convert : public rclcpp::Node
{
	
public:
  Convert();
  Convert(const rclcpp::NodeOptions& options);

  ~Convert()
  {
  }

private:

  void processScan(const lidar_c32_msgs::msg::LidarC32ScanUnified::UniquePtr scanMsg);
  void timeSync(const sensor_msgs::msg::TimeReference::SharedPtr time_msg);
  /// Pointer to dynamic reconfigure service srv_
  
  // Publish scan Data
    void publishScan(lidar_c32_msgs::msg::LidarC32Sweep::SharedPtr sweep_data, int scan_num);
	
  std::shared_ptr<lidar_rawdata::RawData> data_;
  rclcpp::Subscription<lidar_c32_msgs::msg::LidarC32ScanUnified>::SharedPtr packet_sub_;
  rclcpp::Subscription<sensor_msgs::msg::TimeReference>::SharedPtr sync_sub_;
  rclcpp::Time global_time;
  //lidar_c32_msgs::LidarC32ScanUnifiedPtr scan_recv;
  lidar_c32_msgs::msg::LidarC32Sweep::SharedPtr sweep_data;
  bool scan_start;
  bool publish_scan;
  int scan_num;
  std::string scan_frame_id;
  bool time_synchronization_;
  size_t scan_nums;
  //ros::Publisher output_;
  //ros::Publisher scan_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
};

}  // namespace lidar_c32_decoder
#endif

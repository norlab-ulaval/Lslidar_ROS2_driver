/*
 * This file is part of lidar_c32 driver.
 *
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

#ifndef _C32_DRIVER_H_
#define _C32_DRIVER_H_

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "diagnostic_updater/publisher.hpp"
//#include <pcl/point_types.h>
//#include <pcl_ros/impl/transforms.hpp>
//#include "pcl_conversions/pcl_conversions.h"
#include "lidar_c32_msgs/msg/lidar_c32_scan_unified.hpp"
#include "input.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace lidar_c32_driver
{
class lidarDriver : public rclcpp::Node
{
public:
  /**
 * @brief lidarDriver
 * @param node          raw packet output topic
 * @param private_nh    通过这个节点传参数
 */
  lidarDriver();
  lidarDriver(const rclcpp::NodeOptions& options);
  ~lidarDriver();
  
  bool poll(void);
  void difopPoll(void);

  void pollThread(void);
  void difopPollThread(void);
private:
  /// Callback for skip num for time synchronization
  //void skipNumCallback(const std_msgs::msg::Int32::SharedPtr skip_num);

  // configuration parameters
  struct
  {
    std::string frame_id;  ///< tf frame ID
    std::string model;     ///< device model name
    int npackets;          ///< number of packets to collect
    double rpm;            ///< device rotation rate (RPMs)
    double time_offset;    ///< time in seconds added to each  time stamp
    int cut_angle;
    int return_mode;     //return wave number
    int degree_mode;
  } config_;

  std::shared_ptr<Input> msop_input_;
  std::shared_ptr<Input> difop_input_;
  rclcpp::Publisher<lidar_c32_msgs::msg::LidarC32ScanUnified>::SharedPtr msop_output_;
  rclcpp::Publisher<lidar_c32_msgs::msg::LidarC32Packet>::SharedPtr difop_output_;
  rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr output_sync_;
  
  // Converter convtor_
  //diagnostic_updater::Updater diagnostics_;
  //std::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic_;
  std::thread difop_thread_;
  std::thread poll_thread_;
  
  // add for time synchronization
  bool time_synchronization_;
  bool scan_fill;
  unsigned char packetTimeStamp[10];
  uint64_t pointcloudTimeStamp;
  uint64_t GPSStableTS;
  uint64_t GPSCountingTS;
  uint64_t last_FPGA_ts;
  uint64_t GPS_ts;
  int cnt_gps_ts;
  
	int last_difop_num;	
	int current_difop_num;	
	int last_msop_num;	
	int current_msop_num;	
	int last_lvds;	
	int current_lvds;
		
	rclcpp::Time timeStamp;
	lidar_c32_msgs::msg::LidarC32ScanUnified scan_start;
};

}  // namespace lidar_driver

#endif

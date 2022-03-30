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

#include "lidar_c32_decoder/convert.hpp"

namespace lidar_c32_decoder
{
Convert::Convert() : Convert(rclcpp::NodeOptions()) {}

/** @brief Constructor. */																										 
Convert::Convert(const rclcpp::NodeOptions& options) : Node("cloud_node", options), data_(new lidar_rawdata::RawData(this))
{
  scan_nums = 0;
  scan_start = true;
  //scan_recv.reset(new lidar_c32_msgs::LidarC32ScanUnified);
  //scan_recv->packets.resize(1000);

  data_->loadConfigFile();  // load lidar parameters
  
  std::string output_points_topic = std::string("lidar_point_cloud");
  std::string input_packets_topic = std::string("lidar_packet");
  scan_num = 1;
  publish_scan = false;
  scan_frame_id = std::string("laser_link");
  this->declare_parameter("output_points_topic");
  this->declare_parameter("input_packets_topic");
  this->declare_parameter("scan_num");
  this->declare_parameter("publish_scan");
  this->declare_parameter("scan_frame_id");
  
  this->get_parameter("output_points_topic", output_points_topic);
  this->get_parameter("input_packets_topic", input_packets_topic);
  this->get_parameter("scan_num", scan_num);
  this->get_parameter("publish_scan", publish_scan);
  this->get_parameter("scan_frame_id", scan_frame_id);
	
  output_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_points_topic, 10);
  scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 100);
  packet_sub_ = this->create_subscription<lidar_c32_msgs::msg::LidarC32ScanUnified>(input_packets_topic, 10, std::bind(&Convert::processScan, this, std::placeholders::_1));

	//if (time_synchronization_)
	{
		sync_sub_ = this->create_subscription<sensor_msgs::msg::TimeReference>("sync_header", 10, std::bind(&Convert::timeSync, this, std::placeholders::_1));
		//sync_sub_ = node.subscribe("sync_header", 10, &Convert::timeSync, (Convert*)this, ros::TransportHints().tcpNoDelay(true));
	}
	  
	sweep_data = lidar_c32_msgs::msg::LidarC32Sweep::UniquePtr(
                    new lidar_c32_msgs::msg::LidarC32Sweep());
}

void Convert::timeSync(const sensor_msgs::msg::TimeReference::SharedPtr time_msg)
{
  global_time = time_msg->header.stamp;
}

void Convert::publishScan(lidar_c32_msgs::msg::LidarC32Sweep::SharedPtr sweep_data, int scan_num)
{
    sensor_msgs::msg::LaserScan::UniquePtr scan(new sensor_msgs::msg::LaserScan);

    if(sweep_data->scans[scan_num].points.size() <= 1)   return;
	
	uint16_t point_num = 2000;  //65000/32
	double angle_base = M_PI*2 / point_num;

    scan->header.frame_id = scan_frame_id;
    scan->header.stamp = sweep_data->header.stamp;  // timestamp will obtained from sweep data stamp

    scan->time_increment = 0.05;  //s 
    scan->scan_time =1/10.0;  //s
    scan->angle_min = M_PI/2;
    scan->angle_max = M_PI*5/2;
    scan->angle_increment = (scan->angle_max - scan->angle_min)/point_num;
	
    scan->range_min = 0.15;
    scan->range_max = 150;
    scan->ranges.reserve(point_num);
    scan->ranges.assign(point_num, std::numeric_limits<float>::infinity());
    scan->intensities.reserve(point_num);
    scan->intensities.assign(point_num, std::numeric_limits<float>::infinity());
		
  for(uint16_t i = 0; i < sweep_data->scans[scan_num].points.size()-1; i++)
  {
	int point_idx = sweep_data->scans[scan_num].points[i].azimuth / angle_base;

	if (point_idx >= point_num)
	  point_idx = 0;
	if (point_idx < 0)
	  point_idx = point_num - 1;

	scan->ranges[point_num - 1-point_idx] = sweep_data->scans[scan_num].points[i].distance;
	scan->intensities[point_num - 1-point_idx] = sweep_data->scans[scan_num].points[i].intensity;
  }
    scan_pub->publish(std::move(scan));
}


/** @brief Callback for raw scan messages. */
void Convert::processScan(const lidar_c32_msgs::msg::LidarC32ScanUnified::UniquePtr scanMsg)
{
  lidar_rawdata::VPointCloud::Ptr outPoints(new lidar_rawdata::VPointCloud);
  sweep_data = std::make_shared<lidar_c32_msgs::msg::LidarC32Sweep>();
  
  //if (time_synchronization_)
  //{
    //outPoints->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
	  sweep_data->header.stamp = scanMsg->header.stamp;
  /*}
  else
  {
	  sweep_data->header.stamp = this->now();
      outPoints->header.stamp = pcl_conversions::toPCL(sweep_data->header).stamp;//.seconds() / 1000ull;
  }	*/
		
  outPoints->header.frame_id = scanMsg->header.frame_id;
  outPoints->clear();
  outPoints->height = 32;
  outPoints->width = 12 * (int)scanMsg->packets.size();
  outPoints->is_dense = false;
  outPoints->resize(outPoints->height * outPoints->width);
  
  // process each packet provided by the driver
  data_->block_num = 0;
  for (size_t i = 0; i < scanMsg->packets.size(); ++i)
  {
    data_->unpack(scanMsg->packets[i], outPoints, sweep_data, i);
  }
  
  if(scan_num < 0)  scan_num = 0;
  else if (scan_num > 31)  scan_num = 31;
	  
  if(publish_scan)
		publishScan(sweep_data, scan_num);
					
  sensor_msgs::msg::PointCloud2 outMsg;
  pcl::toROSMsg(*outPoints, outMsg);
  outMsg.header.stamp = scanMsg->header.stamp;
  output_->publish(outMsg);
}
}  // namespace lidar_c32_decoder

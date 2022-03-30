/*
 * This file is part of lslidar_n301 driver.
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

#ifndef LSLIDAR_N301_DRIVER_H
#define LSLIDAR_N301_DRIVER_H

#include <unistd.h>
#include <stdio.h>
#include <netinet/in.h>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "diagnostic_updater/publisher.hpp"
#include "lslidar_n301_msgs/msg/lslidar_n301_packet.hpp"
#include "std_msgs/msg/byte.hpp"

namespace lslidar_n301_driver {

static uint16_t PACKET_SIZE = 1206;

class LslidarN301Driver: public rclcpp::Node {
public:
	LslidarN301Driver();
	LslidarN301Driver(const rclcpp::NodeOptions& options);
	~LslidarN301Driver();

    bool initialize();
    bool polling();
	void difopPoll();
	void serialPoll();

    typedef std::shared_ptr<LslidarN301Driver> LslidarN301DriverPtr;
    typedef std::shared_ptr<const LslidarN301Driver> LslidarN301DriverConstPtr;

private:

    bool loadParameters();
    bool createRosIO();
    bool openUDPPort();
    int getPacket(lslidar_n301_msgs::msg::LslidarN301Packet::UniquePtr& msg);
	//int getDifopPacket(lslidar_n301_msgs::msg::LslidarN301Packet::UniquePtr& msg);
	bool SendPacketToLidar(bool power_switch);
	
    // Ethernet relate variables
    std::string device_ip_string;
    in_addr device_ip;
    int UDP_PORT_NUMBER;
    int socket_id;
	int socket_id_difop;
	bool first_time;
	bool add_multicast;
	bool serial_switch;
	unsigned char difop_data[1206];
	std::string group_ip;
	std::string serial_device;
	
    // ROS related variables
    //ros::NodeHandle nh;
    //ros::NodeHandle pnh;

    std::string frame_id;
	
	rclcpp::Publisher<lslidar_n301_msgs::msg::LslidarN301Packet>::SharedPtr packet_pub;
	rclcpp::Publisher<std_msgs::msg::Byte>::SharedPtr type_pub;
	rclcpp::Publisher<lslidar_n301_msgs::msg::LslidarN301Packet>::SharedPtr difop_output_;

	//rclcpp::TimerBase::SharedPtr timer_;
	std::thread difop_thread;
	std::thread serial_thread;
	
    // Diagnostics updater
    diagnostic_updater::Updater diagnostics;
    std::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic;
    double diag_min_freq;
    double diag_max_freq;

};

//typedef LslidarN301Driver::LslidarN301DriverPtr LslidarN301DriverPtr;
//typedef LslidarN301Driver::LslidarN301DriverConstPtr LslidarN301DriverConstPtr;

} // namespace lslidar_driver

#endif // _LSLIDAR_N301_DRIVER_H_

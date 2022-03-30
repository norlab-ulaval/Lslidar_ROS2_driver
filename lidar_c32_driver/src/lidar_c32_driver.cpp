/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009-2012 Austin Robot Technology, Jack O'Quin
 *	Copyright (C) 2017 Robosense, Tony Zhang
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver implementation for the RILIDAR 3D LIDARs
 */
#include "lidar_c32_driver/lidar_c32_driver.h"

namespace lidar_c32_driver
{
static const unsigned int POINTS_ONE_CHANNEL_PER_SECOND65 = 1693;  //65000/384
static const unsigned int POINTS_ONE_CHANNEL_PER_SECOND64 = 1700;  //64000/384

lidarDriver::lidarDriver() : lidarDriver(rclcpp::NodeOptions()) {}
lidarDriver::lidarDriver(const rclcpp::NodeOptions& options) : 
    Node("lidar_node", options)//,  diagnostics_(this)
{
    //std::string tf_prefix = tf::getPrefixParam(private_nh);
    //ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
    //config_.frame_id = tf::resolve(tf_prefix, config_.frame_id);
	config_.frame_id = std::string("lidar");
	config_.model = std::string("C32");
	config_.degree_mode = 1;
	config_.return_mode = 1;
	config_.rpm = 300.0;

	this->declare_parameter("frame_id");
	this->declare_parameter("model");
	this->declare_parameter("degree_mode");
	this->declare_parameter("return_mode");
	this->declare_parameter("rpm");
	this->get_parameter("frame_id", config_.frame_id);
	this->get_parameter("model", config_.model);
	this->get_parameter("degree_mode", config_.degree_mode);
	this->get_parameter("return_mode", config_.return_mode);
	this->get_parameter("rpm", config_.rpm);

    double packet_rate;  // packet frequency (Hz)
	double frequency = (config_.rpm / 60.0);  // expected Hz rate
	
    if(1 == config_.degree_mode){
      packet_rate = POINTS_ONE_CHANNEL_PER_SECOND65 * config_.return_mode;
    }else if(2 == config_.degree_mode){
      packet_rate = POINTS_ONE_CHANNEL_PER_SECOND64 * config_.return_mode;
    }

    config_.npackets = (int)ceil(packet_rate / frequency);
	this->declare_parameter("npackets");
	this->get_parameter("npackets", config_.npackets);
	RCLCPP_INFO(this->get_logger(), "[driver] publishing %d packets per scan", config_.npackets);
    printf("frequency = %f\n", frequency);
    //std::string dump_file;
    //this->get_parameter_or("pcap", dump_file, std::string(""));

    int msop_udp_port = (int)MSOP_DATA_PORT_NUMBER;
	int difop_udp_port = (int)DIFOP_DATA_PORT_NUMBER;
	this->declare_parameter("msop_port");
	this->declare_parameter("difop_port");
	this->get_parameter("msop_port", msop_udp_port);
	this->get_parameter("difop_port", difop_udp_port);
  
	scan_start = lidar_c32_msgs::msg::LidarC32ScanUnified();
	scan_start.packets.resize(1);
	
    // open rslidar input device or file
   /* if (dump_file != "")  // have PCAP file?
    {
        // read data from packet capture file
        msop_input_.reset(new lidar_c32_driver::InputPCAP(this, msop_udp_port, packet_rate, dump_file));
        difop_input_.reset(new lidar_c32_driver::InputPCAP(this, difop_udp_port, packet_rate, dump_file));
    }
    else
    {*/
        // read data from live socket
        msop_input_.reset(new lidar_c32_driver::InputSocket(this, msop_udp_port));
        difop_input_.reset(new lidar_c32_driver::InputSocket(this, difop_udp_port));
   // }

    // raw packet output topic
	std::string output_packets_topic = std::string("lidar_packet");
	std::string output_difop_topic = std::string("lidar_packet_difop");
	time_synchronization_ = false;
	this->declare_parameter("output_packets_topic");
	this->declare_parameter("output_difop_topic");
	this->declare_parameter("time_synchronization");
	this->get_parameter("output_packets_topic", output_packets_topic);
	this->get_parameter("output_difop_topic", output_difop_topic);
	this->get_parameter("time_synchronization", time_synchronization_);
	
	msop_output_ = this->create_publisher<lidar_c32_msgs::msg::LidarC32ScanUnified>(output_packets_topic, 10);
	difop_output_ = this->create_publisher<lidar_c32_msgs::msg::LidarC32Packet>(output_difop_topic, 10);
    
	difop_thread_ = std::thread(std::bind(&lidarDriver::difopPollThread, this));
	poll_thread_ = std::thread(std::bind(&lidarDriver::pollThread, this));

    if (time_synchronization_)
    {
        output_sync_ = this->create_publisher<sensor_msgs::msg::TimeReference>("sync_header", 1);
    }
	scan_fill = false;
}


lidarDriver::~lidarDriver()
{
    if (difop_thread_.joinable()) {
    difop_thread_.join();
  }
  if (poll_thread_.joinable()) {
    poll_thread_.join();
  }
}

void lidarDriver::pollThread(void)
{
  while (rclcpp::ok())
  {
    this->poll();
  }
}

void lidarDriver::difopPollThread(void)
{
  // reading and publishing scans as fast as possible.
  while (rclcpp::ok())
  {
    this->difopPoll();
  }
}

/** poll the device
 *
 *  @returns true unless end of file reached
 */
bool lidarDriver::poll(void)
{
    // Allocate a new shared pointer for zero-copy sharing with other nodelets.
    lidar_c32_msgs::msg::LidarC32ScanUnified::UniquePtr scan(new lidar_c32_msgs::msg::LidarC32ScanUnified());
	
    // Since the rslidar delivers data at a very high rate, keep
    // reading and publishing scans as fast as possible.
    int mode = config_.return_mode;
    int packet_rate;
	int azi1, azi2;
	
    if (difop_input_->getUpdateFlag())
    {
      if(1 == config_.degree_mode){
        packet_rate = POINTS_ONE_CHANNEL_PER_SECOND65 * config_.return_mode;
      }else if(2 == config_.degree_mode){
        packet_rate = POINTS_ONE_CHANNEL_PER_SECOND64 * config_.return_mode;
      }

      config_.rpm = difop_input_->getRpm();
	  if(config_.rpm < 200)  config_.rpm = 600;
      config_.npackets = ceil(packet_rate*60/config_.rpm)*mode ;
	  config_.npackets = config_.npackets + config_.npackets/10;//Increase the  size to find the next 0 degree packet
      difop_input_->clearUpdateFlag();
    }

    scan->packets.resize(config_.npackets);
	
	//Find the 0 degree packet, in order to fix the 0 degree angle position
	if(scan_fill)
	{
		scan->packets[0] = scan_start.packets[0];
	}
	else
	{
		while (true)
		{
			while (true)
			{
				int rc = msop_input_->getPacket(scan->packets[0], config_.time_offset, "msop_pkg");
				if (rc == 0)
					break;  
				if (rc < 0)
					return false;  
			}
			 azi1 = 256 * scan->packets[0].data[3]  + scan->packets[0].data[2] ; 
			 azi2 = 256 * scan->packets[0].data[1103]  + scan->packets[0].data[1102] ; 
			if(azi1 > 35000 && azi2 < 1000)  	break;
		}
	}
	scan_fill = false;
	
    // use in standard behaviour only
    for (int i = 1; i < config_.npackets ; ++i)
    {
        while (true)
        {
            // keep reading until full packet received
            int rc = msop_input_->getPacket(scan->packets[i], config_.time_offset, "msop_pkg");
            if (rc == 0)
                break;  // got a full packet?
            if (rc < 0)
                return false;  // end of file reached?
        }
		
		//packet loss test code
		/*time_t curTime = time(NULL);
		struct tm *curTm = localtime(&curTime);
		char bufTime[30] = {0};
		sprintf(bufTime, "%d-%d-%d %d:%d:%d", curTm->tm_year + 1900, curTm->tm_mon + 1,
				curTm->tm_mday, curTm->tm_hour, curTm->tm_min, curTm->tm_sec);
		current_msop_num = 256 * scan->packets[i].data[1205]  + scan->packets[i].data[1204] ; 
		if(current_msop_num - last_msop_num != 1 && current_msop_num != 0)
		{
			RCLCPP_WARN(this->get_logger(), "time: %s",bufTime);
			RCLCPP_WARN(this->get_logger(), "msop_num: %d :: %d",last_msop_num, current_msop_num);
		}
		last_msop_num = current_msop_num; */
			
       azi1 = 256 * scan->packets[i].data[3]  + scan->packets[i].data[2] ; 
       azi2 = 256 * scan->packets[i].data[1103]  + scan->packets[i].data[1102] ; 
		 
		 //scan_fill:   The last packet in the previous frame acts as the first packet in the next frame
		if((azi1 > 35000 && azi2 < 1000) ||  (azi1 < 500  &&  i  > config_.npackets / 2))
		{
			 scan_fill = true;
			 scan_start.packets[0] = scan->packets[i];
			 break;
		}
    }

    if (time_synchronization_)
    {
        sensor_msgs::msg::TimeReference sync_header;

        // it is already the msop msg
        // use the first packets
        lidar_c32_msgs::msg::LidarC32Packet pkt = scan->packets[0];
        uint64_t packet_timestamp;
        packet_timestamp = (pkt.data[1200]  +
                pkt.data[1201] * pow(2, 8) +
                pkt.data[1202] * pow(2, 16) +
                pkt.data[1203] * pow(2, 24)) * 1e3; //ns

        timeStamp = rclcpp::Time(GPSCountingTS, packet_timestamp);// s,ns
        sync_header.header.stamp = timeStamp;
        output_sync_->publish(sync_header);
    }


    // publish message using time of last packet read
    //  ROS_INFO("Publishing a full scan.");
    if (time_synchronization_)
    {
        scan->header.stamp = timeStamp;
    } else{
        scan->header.stamp = get_clock()->now();//scan->packets.back().stamp;
    }
    scan->header.frame_id = config_.frame_id;
    msop_output_->publish(std::move(scan));

    return true;
}

void lidarDriver::difopPoll(void)
{
    // reading and publishing scans as fast as possible.
    lidar_c32_msgs::msg::LidarC32Packet::UniquePtr difop_packet_ptr(new lidar_c32_msgs::msg::LidarC32Packet);
	lidar_c32_msgs::msg::LidarC32Packet difop_packet_msg;
	int rc = difop_input_->getPacket(difop_packet_msg, config_.time_offset, "difop_pkg");
	if (rc == 0)
	{
		/*time_t curTime = time(NULL);
		struct tm *curTm = localtime(&curTime);
		char bufTime[30] = {0};
		sprintf(bufTime, "%d-%d-%d %d:%d:%d", curTm->tm_year + 1900, curTm->tm_mon + 1,
				curTm->tm_mday, curTm->tm_hour, curTm->tm_min, curTm->tm_sec);
				
		current_lvds = difop_packet_ptr->data[1184] * 256 + difop_packet_ptr->data[1185];
		current_difop_num = difop_packet_ptr->data[1146] * 256 + difop_packet_ptr->data[1147];
		if(current_difop_num - last_difop_num != 1 || last_lvds != current_lvds)
		{
			RCLCPP_INFO(this->get_logger(), "time: %s",bufTime);
			RCLCPP_INFO(this->get_logger(), "difop_num: %d :: %d",last_difop_num, current_difop_num);
			RCLCPP_INFO(this->get_logger(), "lvds: %d :: %d",last_lvds, current_lvds);
		}
		last_lvds = current_lvds;
		last_difop_num = current_difop_num;*/
		*difop_packet_ptr = difop_packet_msg;
		RCLCPP_DEBUG(this->get_logger(), "Publishing a difop data.");
		difop_output_->publish(std::move(difop_packet_ptr));
		if(difop_packet_msg.data[1202] == 0x03){  
				this->packetTimeStamp[4] = difop_packet_msg.data[57];
				this->packetTimeStamp[5] = difop_packet_msg.data[56];
				this->packetTimeStamp[6] = difop_packet_msg.data[55];
				this->packetTimeStamp[7] = difop_packet_msg.data[54];
				this->packetTimeStamp[8] = difop_packet_msg.data[53];
				this->packetTimeStamp[9] = difop_packet_msg.data[52];
		}
		else if(difop_packet_msg.data[1202] >= 0x02 && difop_packet_msg.data[1203] >= 0x70){
				this->packetTimeStamp[4] = difop_packet_msg.data[57];
				this->packetTimeStamp[5] = difop_packet_msg.data[56];
				this->packetTimeStamp[6] = difop_packet_msg.data[55];
				this->packetTimeStamp[7] = difop_packet_msg.data[54];
				this->packetTimeStamp[8] = difop_packet_msg.data[53];
				this->packetTimeStamp[9] = difop_packet_msg.data[52];
		}
		else{
				this->packetTimeStamp[4] = difop_packet_msg.data[41];
				this->packetTimeStamp[5] = difop_packet_msg.data[40];
				this->packetTimeStamp[6] = difop_packet_msg.data[39];
				this->packetTimeStamp[7] = difop_packet_msg.data[38];
				this->packetTimeStamp[8] = difop_packet_msg.data[37];
				this->packetTimeStamp[9] = difop_packet_msg.data[36];
		}
		struct tm cur_time;
		memset(&cur_time, 0, sizeof(cur_time));
		cur_time.tm_sec = this->packetTimeStamp[4]+1;
		cur_time.tm_min = this->packetTimeStamp[5];
		cur_time.tm_hour = this->packetTimeStamp[6];
		cur_time.tm_mday = this->packetTimeStamp[7];
		cur_time.tm_mon = this->packetTimeStamp[8]-1;
		cur_time.tm_year = this->packetTimeStamp[9]+2000-1900;
		this->pointcloudTimeStamp = mktime(&cur_time);
		if (GPSCountingTS != this->pointcloudTimeStamp)
		{
			cnt_gps_ts = 0;
			GPSCountingTS = this->pointcloudTimeStamp;
		}
		else if (cnt_gps_ts == 3)
		{
			GPSStableTS = GPSCountingTS;
		}
		else
		{
			cnt_gps_ts ++;
		}
	}
	if (rc < 0)
		return;  // end of file reached?
}

// add for time synchronization
}  // namespace lidar_c32_driver

/*******************************************************
@company: Copyright (C) 2021, Leishen Intelligent System
@product: LSM10
@filename: lsm10.cpp
@brief:
@version:       date:       author:     comments:
@v1.0           21-2-4      yao          new
*******************************************************/
#include "lsm10_v2/lsm10.h"
#include <stdio.h>
#include <signal.h> 
#include <thread>
#include <time.h>
#include <sys/time.h>

std::vector<int> split(const std::string &s) {
  std::vector<int> elems;
  std::stringstream ss(s);
  std::string number;
 
  while (std::getline(ss, number, ',')) {
    elems.push_back(atoi(number.c_str()));
  }

  return elems;
}

bool publish_scan;

namespace ls{
LSM10 * LSM10::instance()
{
  static LSM10 obj;
  return &obj;
}

LSM10::LSM10()
{
  int code = 0;
  nh = rclcpp::Node::make_shared("lsm10_node");
  
  initParam();
  pub_ = nh->create_publisher<sensor_msgs::msg::LaserScan>(scan_topic_, 5);
  //device_pub = nh->create_publisher<msg::Difop>("difop_information", 100);

  serial_ = LSIOSR::instance(serial_port_, baud_rate_);
  code = serial_->init();
  if(code != 0)
  {
	  printf("open_port %s ERROR !\n", serial_port_.c_str());
	  rclcpp::shutdown();
	  exit(0);
  }
  printf("open_port %s OK!\n", serial_port_.c_str());

  std::thread  recv_thread_(&LSM10::recvThread,this);
  std::thread  pubscan_thread_(&LSM10::pubScanThread,this);
  recv_thread_.detach();
  pubscan_thread_.detach();
}

LSM10::~LSM10()
{
  is_shutdown_ = true;

  serial_->close();
  serial_ = NULL;
  delete serial_;
  printf("end LSM10::~LSM10()\n");
}

void LSM10::initParam()
{
  scan_topic_ = "/scan";
  frame_id_ = "laser_link";
  serial_port_ = "/dev/ttyUSB0";
  baud_rate_ = 460800;
  min_range = 0.3;
  max_range = 100.0;
  angle_disable_min = 0.0;
  angle_disable_max = 0.0;
  
  nh->declare_parameter("scan_topic");
  nh->get_parameter("scan_topic", scan_topic_);
  nh->declare_parameter("frame_id");
  nh->get_parameter("frame_id", frame_id_);
  nh->declare_parameter("serial_port");
  nh->get_parameter("serial_port", serial_port_);
  nh->declare_parameter("baud_rate");
  nh->get_parameter("baud_rate", baud_rate_);
  nh->declare_parameter("min_range");
  nh->get_parameter("min_range", min_range);
  nh->declare_parameter("max_range");
  nh->get_parameter("max_range", max_range);
  nh->declare_parameter("angle_disable_min");
  nh->get_parameter("angle_disable_min", angle_disable_min);
  nh->declare_parameter("angle_disable_max");
  nh->get_parameter("angle_disable_max", angle_disable_max);
	
  is_shutdown_ = false;

  data_len_ = 180;
  points_size_ = 360 * 42 / 15;
  scan_points_.resize(points_size_);
}

int LSM10::getScan(std::vector<ScanPoint> &points, rclcpp::Time &scan_time, float &scan_duration)
{
  points.assign(scan_points_bak_.begin(), scan_points_bak_.end());
  scan_time = pre_time_;
  scan_duration = 0.1;
  return 1;
}

int LSM10::getVersion(std::string &version)
{
  version = "lsm10_v1_0";
  return 0;
}


void LSM10::recvThread()
{
  char * packet_bytes = new char[data_len_];
  int idx = 0;
  int link_time = 0;
  double degree;
  double last_degree = 0.0;
  
  while(!is_shutdown_&&rclcpp::ok()){
	int count = serial_->read(packet_bytes, 92);
	
	if(count <= 0) 
		link_time++;
	else
		link_time = 0;
	
	if(link_time > 10000)
	{
		serial_->close();
		int ret = serial_->init();
		if(ret < 0)
		{
			RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "serial open fail");
			usleep(200000);
		}
		link_time = 0;
	}

	for (int i = 0; i < count; i++)
	{
		int k = packet_bytes[i];
		k < 0 ? k += 256 : k;
		int y = packet_bytes[i + 1];
		y < 0 ? y += 256 : y;

		int k_1 = packet_bytes[i + 2];
		k_1 < 0 ? k_1 += 256 : k_1;
		int y_1 = packet_bytes[i + 3];
		y_1 < 0 ? y_1 += 256 : y_1;
		
		if (k == 0xA5 && y == 0x5A)					 //应答距离
		{
			if(i != 0)
			{
				memcpy(packet_bytes, packet_bytes + 92 - i, 92 - i);
				serial_->read(packet_bytes + 92 - i, i);
			}
			
			int s = packet_bytes[i + 2];
			s < 0 ? s += 256 : s;
			int z = packet_bytes[i + 3];
			z < 0 ? z += 256 : z;
			
			//if ((s * 256 + z) / 100.f > 360)
			//	degree = 0;
			//else
				degree = (s * 256 + z) / 100.f;

			/*lsm10_v2::difopPtr Difop_data = lsm10_v2::difopPtr(
						new lsm10_v2::difop());
			s = packet_bytes[i + 4];
			s < 0 ? s += 256 : s;
			z = packet_bytes[i + 5];
			z < 0 ? z += 256 : z;
			
			Difop_data->motorspeed = float(2500000.0 / (s * 256 + z));
			device_pub->publish(Difop_data);*/
			
			int invalidValue = 0;
			for (size_t num = 2; num < 86; num+=2)
			{
				int s = packet_bytes[i + num + 4];
				s < 0 ? s += 256 : s;
				int z = packet_bytes[i + num + 5];
				z < 0 ? z += 256 : z;
				
				if ((s * 256 + z) != 0xFFFF)
				{
					scan_points_[idx].range = double(s * 256 + (z)) / 1000.f;
					scan_points_[idx].intensity = 0;
					idx++;
				}
				else
				{
					invalidValue++;
				}
			}

			invalidValue = 42 - invalidValue;

			for (int i = 0; i < invalidValue; i++)
			{
				if ((degree + (15.0 / invalidValue * i)) > 360)
					scan_points_[idx-invalidValue+i].degree = degree + (15.0 / invalidValue * i) - 360;
				else
					scan_points_[idx-invalidValue+i].degree = degree + (15.0 / invalidValue * i);
			}
			
			if (degree < last_degree) 	
			{
				idx = 0;
				
				for(unsigned long k=0;k<scan_points_.size();k++)
				{
					if(angle_disable_min < angle_disable_max)
					{
						if(scan_points_[k].degree > angle_disable_min && scan_points_[k].degree < angle_disable_max)
							scan_points_[k].range = 0;
					}
					else if(angle_disable_min != 0.0 || angle_disable_max!= 0.0)
					{
						if(scan_points_[k].degree > angle_disable_min || scan_points_[k].degree < angle_disable_max)
							scan_points_[k].range = 0;
					}
					
					if(scan_points_[k].range < min_range || scan_points_[k].range > max_range)
						scan_points_[k].range = 0;
				}
				
				scan_points_bak_.resize(scan_points_.size());
				scan_points_bak_.assign(scan_points_.begin(), scan_points_.end());
				for(unsigned long k=0; k<scan_points_.size(); k++)
				{
					scan_points_[k].range = 0;
					scan_points_[k].degree = 0;
				}
				publish_scan = true;
				pre_time_ = time_;
				time_ = nh->now();
			}
			last_degree = degree;
		}	
		else if (k == 0xA5 && y == 0xFF && k_1 == 00 && y_1 == 0x5A)
		{
			/*lsm10_v2::difopPtr Difop_data = lsm10_v2::difopPtr(
						new lsm10_v2::difop());
 
			//温度
			int s = packet_bytes[i + 12];
			s < 0 ? s += 256 : s;
			int z = packet_bytes[i + 13];
			z < 0 ? z += 256 : z;
			Difop_data->Temperature = (float(s * 256 + z) / 4096 * 330 - 50);

			//高压
			s = packet_bytes[i + 14];
			s < 0 ? s += 256 : s;
			z = packet_bytes[i + 15];
			z < 0 ? z += 256 : z;
			Difop_data->HighPressure = (float(s * 256 + z) / 4096 * 3.3 * 101);

			//转速
			s = packet_bytes[i + 16];
			s < 0 ? s += 256 : s;
			z = packet_bytes[i + 17];
			z < 0 ? z += 256 : z;
			Difop_data->MotorSpeed = (float(1000000 / (s * 256 + z) / 24));
			
			device_pub.publish(Difop_data);*/
		}
	}
  }
  if (packet_bytes)
  {
    packet_bytes = NULL;
    delete packet_bytes;
  }
}

void LSM10::pubScanThread()
{
  while (rclcpp::ok() && !is_shutdown_)
  {
	if (!publish_scan)
      continue;
  
    std::vector<ScanPoint> points;
    rclcpp::Time start_time;
    float scan_time = 0.1;
  
    this->getScan(points, start_time, scan_time);
    int count = points.size();
    if (count <= 0)
      continue;

    sensor_msgs::msg::LaserScan msg;
    msg.header.frame_id = frame_id_;
    msg.header.stamp = start_time;
    msg.angle_min = 0.0;
    msg.angle_max = 2 * M_PI;
    msg.angle_increment = (msg.angle_max - msg.angle_min) / count;
    msg.range_min = 0.02;
    msg.range_max = 50;
    msg.ranges.resize(count);
    msg.intensities.resize(count);
    msg.scan_time = scan_time;
    msg.time_increment = scan_time / (double)(count - 1);
	
	for(int k=0; k<count; k++)
	{
		msg.ranges[k] = std::numeric_limits<float>::infinity();
        msg.intensities[k] = 0;
	}
	
	for (int i = 0; i < count; i++) {
		int point_idx = 1008 - points[i].degree * 42 / 15;

		if(point_idx < 0 || point_idx > 1007) 
		{
			continue;
		}

      if (points[i].range == 0.0) {
        msg.ranges[point_idx] = std::numeric_limits<float>::infinity();
        msg.intensities[point_idx] = 0;
      }
      else {
        double dist = points[i].range;
        msg.ranges[point_idx] = (float) dist;
        msg.intensities[point_idx] = points[i].intensity;
      }
    }

    pub_->publish(msg);
	publish_scan = false;
  }
}

}

void handleSig(int signo)
{
  printf("handleSig %d\n", signo);
  rclcpp::shutdown();
  exit(0);
}

int main(int argv, char **argc)
{
  signal(SIGINT, handleSig);
  signal(SIGTERM, handleSig);
  rclcpp::init(argv, argc);
 
  ls::LSM10* lsm10 = ls::LSM10::instance();

  rclcpp::WallRate loop_rate(10);
	while (rclcpp::ok()) {
		rclcpp::spin_some(lsm10->nh);
		loop_rate.sleep();
	}
	rclcpp::shutdown();

	return 0;
}

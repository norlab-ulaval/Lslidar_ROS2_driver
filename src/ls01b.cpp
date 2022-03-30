/*******************************************************
@company: Copyright (C) 2018, Leishen Intelligent System
@product: LS01B
@filename: ls01b.cpp
@brief:
@version:       date:       author:     comments:
@v1.0           18-8-21     fu          new
@v1.5           19-4-16     tongsky     Add service to change resolution online
@v1.5           19-4-18     tongsky     Fix bug when switching resolution
@v2.0           19-05-15    tongsky     Add service to start and stop laser; Modify truncated laser data behavior in a smart way
@v2.1           19-07-29    tongsky     Add angle compensation to fix distortion around near obstacle
*******************************************************/
#ifdef _MSC_VER
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#endif

#include "ls01b_v2/ls01b.h"
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

namespace ls{

LS01B * LS01B::instance()
{
  static LS01B obj;
  return &obj;
}

LS01B::LS01B()
{
	nh = rclcpp::Node::make_shared("ls01b_node");
	initParam();
    pub_ = nh->create_publisher<sensor_msgs::msg::LaserScan>(scan_topic_, 5);
	//pub_ = n_.advertise<sensor_msgs::LaserScan>(scan_topic_, 3);
	
	serial_ = LSIOSR::instance(serial_port_, baud_rate_);
	serial_->init();

	//auto changeRes_client = n_.create_service("resolution_control", &LS01B::resServerCallback, this);
	//auto start_client = n_.create_service("start_control", &LS01B::startCallback, this);
	//changeRes_client = n_.advertiseService("resolution_control", &LS01B::resServerCallback, this);
	//start_client = n_.advertiseService("start_control", &LS01B::startCallback, this);

	std::thread  recv_thread_(&LS01B::recvThread,this);
	std::thread  pubscan_thread_(&LS01B::pubScanThread,this);
	recv_thread_.detach();
	pubscan_thread_.detach();
	//recv_thread_ = new boost::thread(boost::bind(&LS01B::recvThread, this));
	//pubscan_thread_ = new boost::thread(boost::bind(&LS01B::pubScanThread, this));
}

LS01B::~LS01B()
{
  stopRecvData();
  usleep(100000);
  stopRecvData();
  usleep(100000);
  stopScan();
  usleep(100000);
  stopScan();

  is_shutdown_ = true;

  serial_->close();
  serial_ = NULL;
  delete serial_;
  printf("end LS01B::~LS01B()\n");
}

void LS01B::initParam()
{
  scan_topic_ = "/scan";
  frame_id_ = "laser_link";
  serial_port_ = "/dev/ttyUSB0";
  baud_rate_ = 460800;
  resolution_ = 1.0;
  robot_radius_ = 0.2;
  center_x_ = 0.0;
  center_y_ = 0.0;
  rpm_ = 600;
  special_range_ = 0.0;
  flag_angle_compensate = true;
  truncated_mode_ = 0;
  
  int disable_tolerance = 5;
  std::string list_min = "";
  std::string list_max = "";
  std::vector<int> disable_angle_min_range, disable_angle_max_range;
  //auto nh = rclcpp::Node::make_shared("ls01b_node");
  //ros::NodeHandle nh("~");
  //rclcpp::Parameter<std::string>("serial_port",serial_port,"/dev/ttyUSB0");
  //pnh.param<int>("point_num", point_num, 300000);

  nh->declare_parameter("scan_topic");
  nh->get_parameter("scan_topic", scan_topic_);
  nh->declare_parameter("frame_id");
  nh->get_parameter("frame_id", frame_id_);
  nh->declare_parameter("serial_port");
  nh->get_parameter("serial_port", serial_port_);
  nh->declare_parameter("baud_rate");
  nh->get_parameter("baud_rate", baud_rate_);
  nh->declare_parameter("angle_resolution");
  nh->get_parameter("angle_resolution", resolution_);
  nh->declare_parameter("robot_radius");
  nh->get_parameter("robot_radius", robot_radius_);
  nh->declare_parameter("center_x");
  nh->get_parameter("center_x", center_x_);
  nh->declare_parameter("center_y");
  nh->get_parameter("center_y", center_y_);
  nh->declare_parameter("rpm");
  nh->get_parameter("rpm", rpm_);
  nh->declare_parameter("special_range");
  nh->get_parameter("special_range", special_range_);
  nh->declare_parameter("angle_compensate");
  nh->get_parameter("angle_compensate", flag_angle_compensate);
  nh->declare_parameter("disable_tolerance");
  nh->get_parameter("disable_tolerance", disable_tolerance);
  nh->declare_parameter("truncated_mode");
  nh->get_parameter("truncated_mode", truncated_mode_);
  nh->declare_parameter("disable_min");
  nh->get_parameter("disable_min", list_min);
  nh->declare_parameter("disable_max");
  nh->get_parameter("disable_max", list_max);
  
  disable_angle_min_range = split(list_min);
  disable_angle_max_range = split(list_max);

  scan_health_ = 0;
  is_shutdown_ = false;
  is_start_ = false;
  use_angle_ = true;
  start_switch = true;
  data_len_ = 180;
  points_size_ = 360 / resolution_;
  scan_points_.resize(points_size_);
  
  // different method to discard laser data
  //int disable_tolerance;
  for(unsigned i=0; i < disable_angle_min_range.size(); i++)
  {

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"truncated angle min is %d", disable_angle_min_range[i]);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"truncated angle max is %d", disable_angle_max_range[i]);
      disable_angle_min_range_.push_back(disable_angle_min_range[i]);
      disable_angle_max_range_.push_back(disable_angle_max_range[i]);
  }

  if (truncated_mode_ == 1)
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"truncated mode is specific angle ranges");
  else if (truncated_mode_ == 2)
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"truncated mode is radius limits");
  else if (truncated_mode_ == 3)
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"truncated mode is radius and specific angle ranges");
  else
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"truncated mode is disable");

  /*dynamic_reconfigure::Server<ls01b_v2::FilterConfig> *dsrv_;
  dsrv_ = new dynamic_reconfigure::Server<ls01b_v2::FilterConfig>(nh);
  dynamic_reconfigure::Server<ls01b_v2::FilterConfig>::CallbackType cb = boost::bind(&LS01B::DynParamCallback, this, _1, _2);
  dsrv_->setCallback(cb);*/
}

/*void LS01B::DynParamCallback(ls01b_v2::FilterConfig &config, uint32_t level)
{
  RCLCPP_INFO("reaction parameters reconfigure request received!");
  robot_radius_ = config.robot_radius;
  truncated_mode_ = config.truncated_mode;
  center_x_ = config.center_x;
  center_y_ = config.center_y;
  double sr = config.special_range;
  flag_angle_compensate = config.angle_compensate;
  if (sr > 90)
  {
    special_range_ = std::numeric_limits<float>::infinity();
  } 
  else
  {
      special_range_ = sr;
  }
  RCLCPP_INFO("Enable angle compensate or not %d", flag_angle_compensate);
  RCLCPP_INFO("current robot radius is %1.3f, centerx %1.3f, centery %1.3f", config.robot_radius, config.center_x, config.center_y);
  RCLCPP_INFO("Truncated Mode is %d, special range for truncated laser data is %3.3f", truncated_mode_, special_range_);
}*/

bool LS01B::isHealth()
{
  return (scan_health_ < 0) ? false : true;
}

bool LS01B::resetHealth()
{
  scan_health_ = 0;
  return scan_health_;
}

int LS01B::getScan(std::vector<ScanPoint> &points, rclcpp::Time &scan_time, float &scan_duration)
{

  points.assign(scan_points_bak_.begin(), scan_points_bak_.end());
  scan_time = pre_time_;
  scan_duration = 0.1;//(float)(time_ - pre_time_);
  return 1;
}

double LS01B::getRPM()
{
  return real_rpm_;
}

int LS01B::startScan()
{
  printf("startScan\n");
  unsigned char data[3];
  data[0] = 0xa5;
  data[1] = 0x2c;
  data[2] = 0x0;
  int rtn = serial_->send((const char*)data, 2);
  if (rtn < 0)
  {
    printf("start scan error !\n");
    scan_health_ = -1;
    return rtn;
  }
  start_switch = true;
  return rtn;
}

int LS01B::stopScan()
{
  printf("stopScan\n");
  unsigned char data[2];
  data[0] = 0xa5;
  data[1] = 0x25;
  int rtn = serial_->send((const char *)data, 2);
  if (rtn < 0)
  {
    printf("stop scan error !\n");
    scan_health_ = -1;
    return rtn;
  }
  is_start_ = false;
  start_switch = false;
  return rtn;
}

int LS01B::setScanMode(bool is_continuous)
{
  printf("setScanMode is_continuous = %d\n", is_continuous);
  unsigned char data[2];
  data[0] = 0xa5;
  data[1] = 0x0;
  
  if (is_continuous){
    data[1] = 0x20;
  }
  else{
    data[1] = 0x22;
  }

  int rtn = serial_->send((const char *)data, 2);
  if (rtn < 0)
  {
    printf("error !\n");
    scan_health_ = -1;
    return rtn;
  }
  serial_->flushinput();
  is_start_ = true;
  usleep(200000);
  return rtn;
}

int LS01B::stopRecvData()
{
  printf("stopRecvData\n");
  unsigned char data[2];
  data[0] = 0xa5;
  data[1] = 0x21;

  int rtn = serial_->send((const char *)data, 2);
  if (rtn < 0)
  {
    printf("stopRecvData error !\n");
    scan_health_ = -1;
    return rtn;
  }
  is_start_ = false;
  return rtn;
}

int LS01B::switchData(bool use_angle)
{
  printf("switchData use_angle = %d\n", use_angle);
  unsigned char data[4];
  data[0] =  0xa5;
  data[1] = 0x0;
  data[2] = 0x0;
  data[3] = 0x0;

  if (use_angle){
    data[1] = 0x5c;
  }
  else{
    data[1] = 0x50;
  }

  int rtn = serial_->send((const char *)data, 4);
  if (rtn < 0)
  {
    printf("switchData error !\n");
    scan_health_ = -1;
    return rtn;
  }
  is_start_ = false;
  use_angle_ = use_angle;

  return rtn;
}

int LS01B::setMotorSpeed(int rpm)
{
  printf("setMotorSpeed rpm = %d\n", rpm);

  unsigned char data[4];
  data[0] = 0xa5;
  data[1] = 0x26;
  data[3] = (rpm & 0xff);
  data[2] = (rpm >> 8) & 0xff;

  int rtn = serial_->send((const char *)data, 4);
  if (rtn < 0)
  {
    printf("setMotorSpeed error !\n");
    scan_health_ = -1;
    return rtn;
  }
  is_start_ = false;

  return rtn;
}

int LS01B::setResolution(double resolution)
{
  printf("setResolution resolution = %f\n", resolution);

  is_start_ = false;
  unsigned char data[4];
  data[0] = 0xa5;
  data[1] = 0x30;
  data[2] = 0x00;

  if (resolution == 0.25){
    data[3] = 0x19;
    data_len_ = 180;
  }
  else if (resolution == 0.5){
    data[3] = 0x32;
    data_len_ = 90;
  }
  else if (resolution == 1.0){
    data[3] = 0x64;
    data_len_ = 45;
  }
  else
  {
    // resolution error
    scan_health_ = -2;
    return scan_health_;
  }

  resolution_ = resolution;
  points_size_ = 360/resolution;
  scan_points_.resize(points_size_);
  

  int rtn = serial_->send((const char *)data, 4);
  if (rtn < 0)
  {
    printf("setResolution error !\n");
    scan_health_ = -1;
    return rtn;
  }
  serial_->flushinput();
  return rtn;
}

/*bool LS01B::startCallback(AddThreeInts::AddThreeInts &req, AddThreeInts::AddThreeInts &res)
{
  bool cmd = req.status.data;
  if (cmd == start_switch)
  {
    res.success = false;
    if (cmd)
    {
      RCLCPP_INFO("laser already start");
      res.message = "laser already start";
    }
    else
    {
      RCLCPP_INFO("laser already stop");
      res.message = "laser already stop";
    }
  }
  else{
    res.success = true;
    if (cmd){
      startScan();
      usleep(100000);
      startScan();
      usleep(100000);
      setScanMode(true);
      res.message = "bring up laser";
    }
    else
    {
      stopRecvData();
      usleep(100000);
      stopScan();
      res.message = "shutdown laser";
    }
  }
  return true;
}

bool LS01B::resServerCallback(AddThreeInts::AddThreeInts::Request &req,
                   AddThreeInts::AddThreeInts::Response &res)
{
  double resolution = req.AddThreeInts.data;
  if (resolution_ == resolution)
  {
      RCLCPP_INFO("The same resolution, no need to switch");
      res.status = true;
      return true;
  }
  if ((resolution == 0.25) | (resolution == 0.5)  | (resolution == 1.0))
  {
    RCLCPP_WARN("User want to switch to resolution %1.2f degree", resolution);
  }
  else{
    RCLCPP_WARN("Invalid angle resolution to set! Keep it same");
    res.status = false;
    return true;
  }
  stopRecvData();   // stop sending data
  usleep(100);
  setResolution(resolution);
  usleep(100000);
  setResolution(resolution);
  usleep(100000);
  setScanMode(true);
  res.status = true;
  serial_->flushinput();
  return true;
}*/

void LS01B::recvThread()
{
  uint8_t start_count = 0;
  char header[6];
  char * packet_bytes = new char[data_len_];
  if (packet_bytes == NULL)
  {
    packet_bytes = NULL;
  }
  int cnt_switch_res_failed = 0;
  
  //struct timeval t1;
  //gettimeofday(&t1, nullptr);
  
  //boost::posix_time::ptime t1,t2;
  //t1 = boost::posix_time::microsec_clock::universal_time();
  while(!is_shutdown_&&rclcpp::ok()){
    while(!is_start_){
      usleep(100000);
    }

    bool is_health = this->isHealth();
    if (!is_health)
    {
      this->resetHealth();
      serial_ = LSIOSR::instance(serial_port_, baud_rate_);
      usleep(100000);
      run();
      continue;
    }

    int count = serial_->read(&header[start_count], 1);
    if (count<=0)
    {
      RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "read header[%d] error\n", start_count);
      start_count = 0;
      scan_health_ = -3;
      continue;
    }

    if(0 == start_count){
      if (0xA5 == (header[start_count]&0xff)){
        start_count = 1;
      }
      else{
        start_count = 0;
      }
    }
    else if (1 == start_count) {

      if (0x6A == (header[start_count]&0xff) || 0x5A == (header[start_count]&0xff))
      {
        if (0x6A == (header[start_count]&0xff))
        {
          //t2 = boost::posix_time::microsec_clock::universal_time();
		  //boost::posix_time::millisec_posix_time_system_config::time_duration_type t_elapse;
          //t_elapse = t2 - t1;
          //real_rpm_ = 1000000.0 / t_elapse.ticks();

          //t1 = t2;

		  //boost::unique_lock<boost::mutex> lock(mutex_);
          scan_points_bak_.resize(scan_points_.size());
          scan_points_bak_.assign(scan_points_.begin(), scan_points_.end());
          pre_time_ = time_;
          //lock.unlock();
          
          //pubscan_cond_.notify_one();
          time_ = nh->now();
        }

        start_count = 2;
        int count = serial_->read(&header[start_count], 4);
        if (count != 4)
        {
  //       RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "read header error\n");
			start_count = 0;
			scan_health_ = -3;
			continue;
        }

        rpm_ = ((header[2]&0x7f) << 8) + (header[3]&0xff);
        //int flag = ((header[2] & 0x80) == 0) ? 0 : 1;
        int angular_resolution = (header[4]&0xff) >> 1;
        int start_angle = ((header[4] & 0x01) << 8) + (header[5]&0xff);

        if (angular_resolution/100.0 != resolution_)
        {
       RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "angular resolution is not ok yet current res %d", angular_resolution);
          usleep(100000);
          cnt_switch_res_failed++;
          if (cnt_switch_res_failed >= 10)
          {
              double tmp_res = resolution_;
           RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Laser switch resolution failed. Fallback");
              setResolution(angular_resolution/100.0);
              usleep(100000);
              setResolution(tmp_res);
              usleep(100000);
              setScanMode(true);
              usleep(100000);
              cnt_switch_res_failed =0;
          }
          start_count = 0;
          continue;
        }
        count = serial_->read(packet_bytes, data_len_);
        if (count != data_len_)
        {
   //    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "read %d packet error. Only read %d data", data_len_, count);
          start_count = 0;
          if (count == 0)
            scan_health_ = -3;
          continue;
        }

        for (int i = 0; i < data_len_; i = i+3)
        {
          int idx = start_angle / resolution_ + i/3;
          double distance = ((packet_bytes[i+1] & 0xFF) << 8) | (packet_bytes[i+2] & 0xFF);         
          double degree = start_angle + idx * resolution_;

          //boost::unique_lock<boost::mutex> lock(mutex_);
          scan_points_[idx].degree = degree;
          scan_points_[idx].range = distance/1000.0;
          if(use_angle_)
          {
            scan_points_[idx].intensity = 0;
          }
          else
          {
            scan_points_[idx].intensity = packet_bytes[i] & 0xFF;
          }
          //lock.unlock();
        }

        start_count = 0;

      }
      else{
        start_count = 0;
      }

    }
  }

  if (packet_bytes)
  {
    packet_bytes = NULL;
    delete packet_bytes;
  }
}

double LS01B::angle_compensate(double dist)
{
  double offset = (0.02345 / dist) * 57.3;
  return offset;
}

void LS01B::pubScanThread()
{
  bool wait_for_wake = true;
  //boost::unique_lock<boost::mutex> lock(pubscan_mutex_);

  while (rclcpp::ok() && !is_shutdown_)
  {
    while (wait_for_wake)
    {
      //pubscan_cond_.wait(lock);
      wait_for_wake = false;
    }
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
    msg.range_min = 0.15;
    msg.range_max = 16;
    msg.ranges.resize(count);
    msg.intensities.resize(count);
    msg.scan_time = scan_time;
    msg.time_increment = scan_time / (double)(count - 1);

    for (int i = count - 1; i >= 0; i--) {
      if (points[count - i - 1].range == 0.0) {
        msg.ranges[i] = std::numeric_limits<float>::infinity();
        msg.intensities[i] = 0;
      }
      else {
        double dist = points[count - i - 1].range;
        if (flag_angle_compensate) {
          double offset = angle_compensate(dist);
          int actual_index = i;
          int fix_index = int16_t (round(offset / resolution_));
    //     RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "dist is %2.2f, current idx is %d, offset is %2.3f, fix_index is %d",dist, i,  offset, fix_index);

          int j = (i - fix_index);
            if (j > 0) {
              actual_index = j;
            } else {
              actual_index = points_size_ + j;
            }
            msg.ranges[actual_index] = (float) dist;
            if (i == 0)
            {
                msg.ranges[i] = (msg.ranges[1] + msg.ranges[count-1]) / 2.0;
            }
        } else{
          msg.ranges[i] = (float) dist;
        }
        msg.intensities[i] = points[count - i - 1].intensity;
      }

      if ((truncated_mode_ == 1) or (truncated_mode_ == 3))
      {
        for (long unsigned int j = 0; j < disable_angle_max_range_.size(); ++j) {
          if ((i >= (disable_angle_min_range_[j] * count / 360) ) &&
              (i <= (disable_angle_max_range_[j] * count / 360 ))) {
            msg.ranges[i] = special_range_;
            msg.intensities[i] = 0;
          }
        }
      }
      if ((truncated_mode_ == 2) or (truncated_mode_ == 3))
      {
        double point_dist = msg.ranges[i];
        if (point_dist < 1.0 && point_dist > 0.06)
        {
          double x = point_dist * cos(i * resolution_ * M_PI / 180);
          double y = point_dist * sin(i * resolution_ * M_PI / 180);

          double dist2center = sqrt((y - center_y_) * (y - center_y_) + (x - center_x_) * (x - center_x_));
          if (dist2center < robot_radius_)
          {
            msg.ranges[i] = special_range_;
            msg.intensities[i] = 0;
          }
        }
      }

    }
    pub_->publish(msg);
    wait_for_wake = true;
  }
}

void LS01B::run()
{
  setResolution(resolution_);
  usleep(100000);
  setMotorSpeed(rpm_);
  usleep(100000);
  switchData(false);
  usleep(100000);
  startScan();
  usleep(100000);
  startScan();
  usleep(100000);
  setScanMode(true); 
}


}

void handleSig(int signo)
{
  int unused;
  unused = signo;
  printf("handleSig %d\n", unused);
  rclcpp::shutdown();
  exit(0);
}

int main(int argc, char *argv[])
{
	signal(SIGINT, handleSig);
	signal(SIGTERM, handleSig);
	rclcpp::init(argc, argv);

	ls::LS01B* ls01b = ls::LS01B::instance();
	
	rclcpp::WallRate loop_rate(10);
	usleep(100000);
	ls01b->run();
	while (rclcpp::ok()) {
		
		rclcpp::spin_some(ls01b->nh);
		loop_rate.sleep();
	}
	
	rclcpp::shutdown();
	return 0;
}

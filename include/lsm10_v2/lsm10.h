/*******************************************************
@company: Copyright (C) 2021, Leishen Intelligent System
@product: LSM10
@filename: lsm10.cpp
@brief:
@version:       date:       author:     comments:
@v1.0           21-2-4      yao          new
*******************************************************/
#ifndef LSM10_H
#define LSM10_H
//#include <boost/date_time/posix_time/posix_time.hpp>
//#include <boost/thread.hpp>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
//#include "lsm10_v2/msg/difop.hpp"
#include "lsm10_v2/lsiosr.h"

namespace ls {

typedef struct {
    double degree;
    double range;
    double intensity;
} ScanPoint;

class LSM10
{
public:
  LSM10();
  ~LSM10();
    /**
    * 实例化雷达
    * port: 串口号，
    * baud_rate: 波特率 460800
    */
    static LSM10* instance();

    /**
    * 获取雷达数据
    * poins: 雷达点的数据。类型为ScanPoint数组
    */
    int getScan(std::vector<ScanPoint> &points, rclcpp::Time &scan_time, float &scan_duration);

    /**
    * 获取软件版本号
    * version: 返回版本号
    */
    int getVersion(std::string &version);

    int getRate();
	rclcpp::Node::SharedPtr nh = nullptr;
  private:
    void initParam();
    void recvThread();
    void pubScanThread();

    std::vector<ScanPoint> scan_points_;
    std::vector<ScanPoint> scan_points_bak_;

    LSIOSR * serial_;
    /*boost::thread *recv_thread_;
    boost::thread *pubscan_thread_;
    boost::mutex pubscan_mutex_;
    boost::mutex mutex_;
    boost::condition_variable pubscan_cond_;*/

    bool is_shutdown_;    // shutdown recvthread
    int data_len_;
    int points_size_;
	
	double min_range;
    double max_range;
    double angle_disable_min;
    double angle_disable_max;
	
    rclcpp::Time pre_time_;
    rclcpp::Time time_;
	
    std::string serial_port_;
    int baud_rate_;
    std::string scan_topic_;
    std::string frame_id_;
	
	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
	//rclcpp::Publisher<msg::Difop>::SharedPtr device_pub;
};

}
#endif

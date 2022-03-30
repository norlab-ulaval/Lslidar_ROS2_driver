
#ifndef LSLIDAR_Ch_DECODER_LSLIDAR_Ch_DECODER_H
#define LSLIDAR_Ch_DECODER_LSLIDAR_Ch_DECODER_H

#include "rclcpp/rclcpp.hpp"
#include "lslidar_ch_msgs/msg/lslidar_ch_packet.hpp"
#include "lslidar_ch_msgs/msg/lslidar_ch_scan.hpp"
#include "lslidar_ch_msgs/msg/lslidar_ch_scan_unified.hpp"
#include "lslidar_ch_msgs/msg/lslidar_ch_sweep.hpp"
#include "lslidar_ch_msgs/msg/lslidar_ch_point.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include <stdio.h>
#include <string.h>
#include <cmath>
#include <algorithm>
#include <vector>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <chrono>
#include <memory>



namespace lslidar_ch_decoder {

    static const double DISTANCE_RESOLUTION = 0.0000390625; //meters
    static const int POINTS_PER_PACKET = 171;

    static const double scan_laser_altitude[32] = {
            -0.29670597283903605, -0.2792526803190927,
            -0.2617993877991494, -0.24434609527920614,
            -0.22689280275926285, -0.20943951023931956,
            -0.19198621771937624, -0.17453292519943295,
            -0.15707963267948966, -0.13962634015954636,
            -0.12217304763960307, -0.10471975511965978,
            -0.08726646259971647, -0.06981317007977318,
            -0.05235987755982989, -0.03490658503988659,
            -0.017453292519943295, 0.0,
            0.017453292519943295, 0.03490658503988659,
            0.05235987755982989, 0.06981317007977318,
            0.08726646259971647, 0.10471975511965978,
            0.12217304763960307, 0.13962634015954636,
            0.15707963267948966, 0.17453292519943295,
            0.19198621771937624, 0.20943951023931956,
            0.22689280275926285, 0.24434609527920614,
    };
    static const double big_angle[32] = {-17, -16, -15, -14, -13, -12, -11, -10,
                                         -9, -8, -7, -6, -5, -4.125, -4, -3.125,
                                         -3, -2.125, -2, -1.125, -1, -0.125, 0, 0.875,
                                         1, 1.875, 2, 3, 4, 5, 6, 7
    };

    struct PointXYZITM {
        PCL_ADD_POINT4D
        float intensity;
        uint16_t ring;
        double timestamp;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
    } EIGEN_ALIGN16;

    class LslidarChDecoder : public rclcpp::Node {
    public:
        LslidarChDecoder();

        LslidarChDecoder(const rclcpp::NodeOptions &options);

        ~LslidarChDecoder() {};



    private:


        union TwoBytes {
            uint16_t value;
            uint8_t bytes[2];
        };

        union ThreeBytes {
            uint32_t value;
            uint8_t bytes[3];
        };

        struct Point {
            uint8_t vertical_line;
            uint8_t azimuth_1;
            uint8_t azimuth_2;
            uint8_t distance_1;
            uint8_t distance_2;
            uint8_t distance_3;
            uint8_t intensity;
        };
        struct RawPacket {
            Point points[POINTS_PER_PACKET];
            uint32_t time_stamp;
            uint8_t factory[2];
        };
        struct Firing{
            int vertical_line;
            double azimuth;
            double distance;
            float intensity;
        };

        bool initialize();
        bool loadParameters();
        bool createRosIO();
        void packetCallback(const lslidar_ch_msgs::msg::LslidarChPacket::SharedPtr packet_msg);
        int checkPacketValidity(const RawPacket* packet);
        void decodePacket(const RawPacket* packet);

        void publishPointCloud();

        bool isPointInRange(const double &distance){
            return (distance >= min_range && distance <= max_range);
        }

        double verticalLineToAngle(const uint16_t &vertical_line){
            return static_cast<double>((big_angle[vertical_line/4]) * M_PI / 180);
        }

        double min_range;
        double max_range;
        bool use_gps_ts;
        std::string topic_name;
        std::string frame_id;
        rclcpp::Time packet_timeStamp;
        uint64_t packet_end_timeStamp;

        Firing firings[POINTS_PER_PACKET];

        lslidar_ch_msgs::msg::LslidarChScan::UniquePtr sweep_data;

        rclcpp::Subscription<lslidar_ch_msgs::msg::LslidarChPacket>::SharedPtr packet_sub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub;
    };

    typedef PointXYZITM VPoint;
    typedef pcl::PointCloud<VPoint> VPointCloud;



} //end namespace lslidar_ch_decoder
POINT_CLOUD_REGISTER_POINT_STRUCT(lslidar_ch_decoder::PointXYZITM,
                                  (float, x, x)(float, y, y)(float, z, z)
                                          (float, intensity, intensity)
                                          (std::uint16_t, ring, ring)
                                          (double, timestamp, timestamp)
)
#endif //LSLIDAR_Ch_DECODER_LSLIDAR_Ch_DECODER_H

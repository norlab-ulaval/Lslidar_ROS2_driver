/*
 * This file is part of lslidar_ch driver.
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

#ifndef _LS_Ch_DRIVER_H_
#define _LS_Ch_DRIVER_H_

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "diagnostic_updater/publisher.hpp"
#include "lslidar_ch_msgs/msg/lslidar_ch_scan_unified.hpp"
#include "input.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace lslidar_ch_driver {
    class lslidarDriver : public rclcpp::Node {
    public:
        lslidarDriver();

        lslidarDriver(const rclcpp::NodeOptions &options);

        ~lslidarDriver();
        bool initialize();
        void initTimeStamp();
        void getFPGA_GPSTimeStamp(lslidar_ch_msgs::msg::LslidarChPacket::UniquePtr &packet);

        bool poll(void);

        void difopPoll(void);


        void pollThread(void);

        void difopPollThread(void);

    private:
        bool loadParameters();
        bool createRosIO();

        int msop_udp_port;
        int difop_udp_port;

        // configuration parameters
        struct {
            std::string frame_id;  ///< tf frame ID
            std::string model;     ///< device model name
            double rpm;            ///< device rotation rate (RPMs)
            int cut_angle;
            int return_mode;     //return wave number
        } config_;

        std::shared_ptr <Input> msop_input_;
        std::shared_ptr <Input> difop_input_;

        rclcpp::Publisher<lslidar_ch_msgs::msg::LslidarChPacket>::SharedPtr packet_pub;



        std::thread difop_thread_;
        std::thread poll_thread_;

        // add for time synchronization
        bool time_synchronization_;


        unsigned char packetTimeStamp[10];
        uint64_t pointcloudTimeStamp;

        int cnt_gps_ts;
        int socket_id;

        rclcpp::Time timeStamp;
    };

}  // namespace lslidar_driver

#endif

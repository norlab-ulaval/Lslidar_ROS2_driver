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

#include "lslidar_ch_driver/lslidar_ch_driver.h"

namespace lslidar_ch_driver {

    lslidarDriver::lslidarDriver() : lslidarDriver(rclcpp::NodeOptions()) {}

    lslidarDriver::lslidarDriver(const rclcpp::NodeOptions &options) :
            Node("lslidar_node", options), socket_id(-1) {
        config_.frame_id = std::string("lslidar");
        config_.model = std::string("LSCh");

        config_.return_mode = 1;
        config_.rpm = 600.0;


        double frequency = (config_.rpm / 60.0);  // expected Hz rate



        RCLCPP_INFO(this->get_logger(), "[driver] return mode = %d ", config_.return_mode);

        printf("frequency = %f\n", frequency);
        this->initialize();

    }


    lslidarDriver::~lslidarDriver() {
        if (difop_thread_.joinable()) {
            difop_thread_.join();
        }
        if (poll_thread_.joinable()) {
            poll_thread_.join();
        }
        (void) close(socket_id);
    }

    void lslidarDriver::pollThread(void) {
        while (rclcpp::ok()) {
            this->poll();
        }
    }

    void lslidarDriver::difopPollThread(void) {
        // reading and publishing scans as fast as possible.
        while (rclcpp::ok()) {
            this->difopPoll();
        }
    }

    bool lslidarDriver::loadParameters() {

        this->declare_parameter("msop_port",2368);
        this->declare_parameter("difop_port",2369);
        this->get_parameter("msop_port", msop_udp_port);
        this->get_parameter("difop_port", difop_udp_port);

        this->declare_parameter("frame_id");
        this->declare_parameter("model");
        this->declare_parameter("return_mode");
        this->declare_parameter("rpm");

        this->get_parameter("frame_id", config_.frame_id);
        this->get_parameter("model", config_.model);
        this->get_parameter("return_mode", config_.return_mode);
        this->get_parameter("rpm", config_.rpm);


        time_synchronization_ = false;


        this->declare_parameter("time_synchronization");


        this->get_parameter("time_synchronization", time_synchronization_);

        return true;

    }

    bool lslidarDriver::createRosIO() {
        std::string output_packets_topic = std::string("lslidar_packet");
        this->declare_parameter("output_packets_topic");
        this->get_parameter("output_packets_topic", output_packets_topic);
        packet_pub = this->create_publisher<lslidar_ch_msgs::msg::LslidarChPacket>(output_packets_topic, 10);

        // read data from live socket
        msop_input_.reset(new lslidar_ch_driver::InputSocket(this, msop_udp_port));
        difop_input_.reset(new lslidar_ch_driver::InputSocket(this, difop_udp_port));

        difop_thread_ = std::thread(std::bind(&lslidarDriver::difopPollThread, this));
        poll_thread_ = std::thread(std::bind(&lslidarDriver::pollThread, this));
        return true;

    }

    bool lslidarDriver::initialize() {
        this->initTimeStamp();
        if (!loadParameters()) {
            RCLCPP_INFO(this->get_logger(), "Cannot load all required ROS parameters...");
            return false;
        }
        if (!createRosIO()) {
            RCLCPP_INFO(this->get_logger(), "Cannot create all ROS IO ...");
            return false;
        }
        return true;
    }

    void lslidarDriver::initTimeStamp() {
        for (int i = 0; i < 10; i++) {
            this->packetTimeStamp[i] = 0;

        }
        this->pointcloudTimeStamp = 0;
        this->timeStamp = rclcpp::Time(0);
    }

/** poll the device
 *
 *  @returns true unless end of file reached
 */
    bool lslidarDriver::poll(void) {

        lslidar_ch_msgs::msg::LslidarChPacket::UniquePtr packet(
                new lslidar_ch_msgs::msg::LslidarChPacket());

        while (true) {
            int rc = msop_input_->getPacket(packet, "msop_pkg");
            if (rc == 0) break;
            if (rc < 0) return false;
        }

        if (time_synchronization_) {
           lslidar_ch_msgs::msg::LslidarChPacket pkt = *packet;

            this->packetTimeStamp[4] = pkt.data[1199];
            this->packetTimeStamp[5] = pkt.data[1198];
            this->packetTimeStamp[6] = pkt.data[1197];
            struct tm cur_time;
            memset(&cur_time, 0, sizeof(cur_time));
            cur_time.tm_sec = this->packetTimeStamp[4] + 1;
            cur_time.tm_min = this->packetTimeStamp[5];
            cur_time.tm_hour = this->packetTimeStamp[6];
            cur_time.tm_mday = this->packetTimeStamp[7];
            cur_time.tm_mon = this->packetTimeStamp[8] - 1;
            cur_time.tm_year = this->packetTimeStamp[9] + 2000 - 1900;
            this->pointcloudTimeStamp = mktime(&cur_time);

            uint64_t packet_timestamp;
            packet_timestamp = (pkt.data[1203] +
                                pkt.data[1202] * pow(2, 8) +
                                pkt.data[1201] * pow(2, 16) +
                                pkt.data[1200] * pow(2, 24)) * 1e3; //ns
            timeStamp = rclcpp::Time(this->pointcloudTimeStamp, packet_timestamp);// s,ns
            packet->header.stamp = timeStamp;
        }else{
            packet->header.stamp = get_clock()->now();
        }
        packet->header.frame_id = config_.frame_id;
        packet_pub->publish(std::move(packet));

        return true;
    }

    void lslidarDriver::difopPoll(void) {
        lslidar_ch_msgs::msg::LslidarChPacket::UniquePtr difop_packet_ptr(
                new lslidar_ch_msgs::msg::LslidarChPacket);

        // keep reading
        int rc = difop_input_->getPacket(difop_packet_ptr, "difop_pkg");
        if (rc == 0) {
            getFPGA_GPSTimeStamp(difop_packet_ptr);
        }else if (rc < 0)
            return;

    }
    void lslidarDriver::getFPGA_GPSTimeStamp(lslidar_ch_msgs::msg::LslidarChPacket::UniquePtr &packet) {
        unsigned char head2[]={packet->data[0],packet->data[1],packet->data[2],packet->data[3]};
        if(head2[0]==0xA5 && head2[1]==0xFF &&head2[3]==0x00 &&head2[0]==0x5A){
            this->packetTimeStamp[7] = packet->data[54];
            this->packetTimeStamp[8] = packet->data[53];
            this->packetTimeStamp[9] = packet->data[52];
        }

    }
}  // namespace lslidar_ch_driver

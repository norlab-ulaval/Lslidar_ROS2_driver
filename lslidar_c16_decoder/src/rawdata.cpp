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

#include "lslidar_c16_decoder/rawdata.hpp"
#include <angles/angles.h>



namespace lslidar_rawdata {

    RawData::RawData(rclcpp::Node *private_nh) : private_nh_(private_nh) {
        this->is_init_angle_ = false;
        this->is_init_curve_ = false;
        this->is_init_top_fw_ = false;
    }

    void RawData::loadConfigFile() {
        std::string model;
        std::string resolution_param;


        start_angle_ = 0.0f;
        end_angle_ = 360.0f;
        max_distance_ = 200.0f;
        min_distance_ = 0.2f;
        degree_mode_ = 0;
        distance_unit_ = 0.25f;
        config_vert_ = true;
        print_vert_ = true;
        model = std::string("LSC16");
        return_mode_ = 1;
        private_nh_->declare_parameter<float>("start_angle", 0.0f);
        private_nh_->declare_parameter<float>("end_angle", 360.0f);
        private_nh_->declare_parameter<float>("max_distance", 200.0f);
        private_nh_->declare_parameter<float>("min_distance", 0.2f);
        private_nh_->declare_parameter<int>("degree_mode", 0);
        private_nh_->declare_parameter<float>("distance_unit", 0.25f);
        private_nh_->declare_parameter<bool>("config_vert", true);
        private_nh_->declare_parameter<bool>("print_vert", true);
        private_nh_->declare_parameter<std::string>("model", "LSC16");
        private_nh_->declare_parameter<int>("return_mode", 1);

        private_nh_->get_parameter("start_angle", start_angle_);
        private_nh_->get_parameter("end_angle", end_angle_);
        private_nh_->get_parameter("max_distance", max_distance_);
        private_nh_->get_parameter("min_distance", min_distance_);
        private_nh_->get_parameter("degree_mode", degree_mode_);
        private_nh_->get_parameter("distance_unit", distance_unit_);
        private_nh_->get_parameter("config_vert", config_vert_);
        private_nh_->get_parameter("print_vert", print_vert_);
        private_nh_->get_parameter("model", model);
        private_nh_->get_parameter("return_mode", return_mode_);

        if (start_angle_ < 0 || start_angle_ > 360 || end_angle_ < 0 || end_angle_ > 360) {
            start_angle_ = 0;
            end_angle_ = 360;
            RCLCPP_INFO(private_nh_->get_logger(), "start angle and end angle select feature deactivated.");
        } else {
            RCLCPP_INFO(private_nh_->get_logger(), "start angle and end angle select feature activated.");
        }

        angle_flag_ = true;
        if (start_angle_ > end_angle_) {
            angle_flag_ = false;
            RCLCPP_INFO(private_nh_->get_logger(), "Start angle is smaller than end angle, not the normal state!");
        }

        start_angle_ = start_angle_ / 180 * M_PI;
        end_angle_ = end_angle_ / 180 * M_PI;

        RCLCPP_INFO(private_nh_->get_logger(), "config_vert: %d, print_vert: %d", config_vert_, print_vert_);
        RCLCPP_INFO(private_nh_->get_logger(), "distance threshlod, max: %f, min: %f", max_distance_, min_distance_);
        RCLCPP_INFO(private_nh_->get_logger(), "The distance unit is : %f", distance_unit_);

        intensity_mode_ = 1;
        info_print_flag_ = false;
        config_vert_angle = false;
        first_start = true;

        //numOfLasers = 32;
        R1_ = 0.04638;
        R2_ = 0.010875;
        lslidar_type = 2;
        rpm_ = 300;
        if (2 == degree_mode_) {
            //Vertical Angle Calibration for device package
            for (int i = 0; i < LSC16_SCANS_PER_FIRING; i++) {
                cos_scan_altitude_caliration[i] = std::cos(scan_altitude_original_2[i]);
                sin_scan_altitude_caliration[i] = std::sin(scan_altitude_original_2[i]);
                scan_altitude[i] = scan_altitude_original_2[i];
            }
        } else if (1 == degree_mode_) {
            for (int i = 0; i < LSC16_SCANS_PER_FIRING; i++) {
                cos_scan_altitude_caliration[i] = std::cos(scan_altitude_original_1[i]);
                sin_scan_altitude_caliration[i] = std::sin(scan_altitude_original_1[i]);
                scan_altitude[i] = scan_altitude_original_1[i];
            }

        }
        for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index) {
            float rotation = angles::from_degrees(ROTATION_RESOLUTION * rot_index);
            cos_azimuth_table[rot_index] = cosf(rotation);
            sin_azimuth_table[rot_index] = sinf(rotation);
        }



        // receive difop data
        // subscribe to difop lslidar packets, if not right correct data in difop, it will not revise the correct data in the
        // VERT_ANGLE, HORI_ANGLE etc.
        difop_sub_ = private_nh_->create_subscription<lslidar_c16_msgs::msg::LslidarC16Packet>("lslidar_packet_difop",
                                                                                               10, std::bind(
                        &RawData::processDifop, this, std::placeholders::_1));
    }

    void RawData::processDifop(const lslidar_c16_msgs::msg::LslidarC16Packet::SharedPtr difop_msg) {
        const uint8_t *data = &difop_msg->data[0];


        // check header
        if (data[0] != 0xa5 || data[1] != 0xff || data[2] != 0x00 || data[3] != 0x5a) {
            return;
        }


        int version_data = data[1202];
        if (config_vert_) {
            if (2 == version_data) {
                for (int i = 0; i < 16; i++) {
                    uint8_t data1 = data[234 + 2 * i];
                    uint8_t data2 = data[234 + 2 * i + 1];
                    int vert_angle = data1 * 256 + data2;
                    if (vert_angle > 32767) {
                        vert_angle = vert_angle - 65535;
                    }

                    //ROS_INFO_STREAM("data vert: " << vert_angle);
                    scan_altitude[i] = ((float) vert_angle / 100.f) * DEG_TO_RAD;
                    if (2 == degree_mode_) {
                        if (scan_altitude[i] != 0) {
                            if (fabs(scan_altitude_original_2[i] - scan_altitude[i]) * RAD_TO_DEG > 1.5) {
                                scan_altitude[i] = scan_altitude_original_2[i];
                            }
                        } else {
                            scan_altitude[i] = scan_altitude_original_2[i];
                        }
                    } else if (1 == degree_mode_) {
                        if (scan_altitude[i] != 0) {
                            if (fabs(scan_altitude_original_1[i] - scan_altitude[i]) * RAD_TO_DEG > 1.5) {
                                scan_altitude[i] = scan_altitude_original_1[i];
                            }
                        } else {
                            scan_altitude[i] = scan_altitude_original_1[i];
                        }

                    }
                    config_vert_angle = true;
                }
            } else {
                for (int i = 0; i < 16; i++) {
                    uint8_t data1 = data[245 + 2 * i];
                    uint8_t data2 = data[245 + 2 * i + 1];
                    int vert_angle = data1 * 256 + data2;
                    if (vert_angle > 32767) {
                        vert_angle = vert_angle - 65535;
                    }

                    //ROS_INFO_STREAM("data vert: " << vert_angle);
                    scan_altitude[i] = ((float) vert_angle / 100.f) * DEG_TO_RAD;

                    if (2 == degree_mode_) {
                        if (scan_altitude[i] != 0) {
                            if (fabs(scan_altitude_original_2[i] - scan_altitude[i]) * RAD_TO_DEG > 1.5) {
                                scan_altitude[i] = scan_altitude_original_2[i];
                            }
                        } else {
                            scan_altitude[i] = scan_altitude_original_2[i];
                        }
                    } else if (1 == degree_mode_) {
                        if (scan_altitude[i] != 0) {
                            if (fabs(scan_altitude_original_1[i] - scan_altitude[i]) * RAD_TO_DEG > 1.5) {
                                scan_altitude[i] = scan_altitude_original_1[i];
                            }
                        } else {
                            scan_altitude[i] = scan_altitude_original_1[i];
                        }

                    }
                    config_vert_angle = true;
                }
            }
        }

        // rpm
        if ((data[8] == 0x04) && (data[9] == 0xB0)) {
            rpm_ = 1200;
        } else if ((data[8] == 0x02) && (data[9] == 0x58)) {
            rpm_ = 600;
        } else if ((data[8] == 0x01) && (data[9] == 0x2C)) {
            rpm_ = 300;
        } else {
            rpm_ = (data[8] << 8) | data[9];
            if (rpm_ < 200) rpm_ = 600;
        }

        if (print_vert_) {
            RCLCPP_INFO(private_nh_->get_logger(), "rpm is %d", rpm_);
        }
    }


/** @brief convert raw packet to point cloud
 *  @param pkt raw packet to unpack
 *  @param pc shared pointer to point cloud (points are appended)
 */
    void
    RawData::unpack(const lslidar_c16_msgs::msg::LslidarC16Packet &pkt, pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud,
                    lslidar_c16_msgs::msg::LslidarC16Sweep::SharedPtr sweep_data, int Packet_num, std::vector<u_int32_t>& time_vect) {
        float azimuth;  // 0.01 dgree
        float intensity;
        float azimuth_diff;
        float azimuth_corrected_f;
        int azimuth_corrected;

        
        if (config_vert_angle) {
            for (int i = 0; i < LSC16_SCANS_PER_FIRING; i++) {
                cos_scan_altitude_caliration[i] = std::cos(scan_altitude[i]);
                sin_scan_altitude_caliration[i] = std::sin(scan_altitude[i]);

                if (print_vert_) {
                    RCLCPP_INFO(private_nh_->get_logger(), "Channel %d Data  %f", i, scan_altitude[i]);
                }
            }
            config_vert_angle = false;
        }

        const raw_packet_t *raw = (const raw_packet_t *) &pkt.data[0];

        for (int block = 0; block < BLOCKS_PER_PACKET; block++, this->block_num++)  // 1 packet:12 data blocks
        {
            if (UPPER_BANK != raw->blocks[block].header) {

                break;
            }
            azimuth = (float) (256 * raw->blocks[block].rotation_2 + raw->blocks[block].rotation_1);


            if (2 == return_mode_) {

                if (block < (BLOCKS_PER_PACKET - 2))  // 12
                {
                    int azi1, azi2;
                    azi1 = 256 * raw->blocks[block + 2].rotation_2 + raw->blocks[block + 2].rotation_1;
                    azi2 = 256 * raw->blocks[block].rotation_2 + raw->blocks[block].rotation_1;
                    azimuth_diff = (float) ((36000 + azi1 - azi2) % 36000);
                } else {
                    int azi1, azi2;
                    azi1 = 256 * raw->blocks[block].rotation_2 + raw->blocks[block].rotation_1;
                    azi2 = 256 * raw->blocks[block - 2].rotation_2 + raw->blocks[block - 2].rotation_1;
                    azimuth_diff = (float) ((36000 + azi1 - azi2) % 36000);
                }

            } else {

                if (block < (BLOCKS_PER_PACKET - 1))  // 12
                {
                    int azi1, azi2;
                    azi1 = 256 * raw->blocks[block + 1].rotation_2 + raw->blocks[block + 1].rotation_1;
                    azi2 = 256 * raw->blocks[block].rotation_2 + raw->blocks[block].rotation_1;
                    azimuth_diff = (float) ((36000 + azi1 - azi2) % 36000);
                } else {
                    //block 12
                    int azi1, azi2;
                    azi1 = 256 * raw->blocks[block].rotation_2 + raw->blocks[block].rotation_1;
                    azi2 = 256 * raw->blocks[block - 1].rotation_2 + raw->blocks[block - 1].rotation_1;
                    azimuth_diff = (float) ((36000 + azi1 - azi2) % 36000);
                }
            }

            float cos_azimuth;
            float sin_azimuth;
            packet_end = (POINTS_ONE_CHANNEL_PER_SECOND / BLOCKS_ONE_CHANNEL_PER_PKT) / 2 * 60 / rpm_ * return_mode_;

            for (int firing = 0, k = 0; firing < LSC16_FIRINGS_PER_BLOCK; firing++)  // 2
            {
                for (int dsr = 0; dsr < LSC16_SCANS_PER_FIRING; dsr++, k += RAW_SCAN_SIZE)  // 16   3
                {
                    azimuth_corrected_f = azimuth + azimuth_diff / (LSC16_SCANS_PER_FIRING * 2) *
                                                    (LSC16_SCANS_PER_FIRING * firing + dsr);

                    azimuth_corrected = ((int) round(azimuth_corrected_f)) % 36000;  // convert to integral value...

                    cos_azimuth = cos_azimuth_table[azimuth_corrected];
                    sin_azimuth = sin_azimuth_table[azimuth_corrected];

                    if (Packet_num < 10 && azimuth_corrected > 18000) continue;

                    if (Packet_num > int(packet_end * 0.9) && azimuth_corrected < 18000) continue;

                    //distance
                    union two_bytes tmp;
                    tmp.bytes[0] = raw->blocks[block].data[k];
                    tmp.bytes[1] = raw->blocks[block].data[k + 1];
                    int distance = tmp.uint;

                    // read intensity
                    intensity = raw->blocks[block].data[k + 2];
                    float distance2 = (distance * DISTANCE_RESOLUTION) * distance_unit_;

                    //The offset calibration
                    float arg_horiz = (float) azimuth_corrected_f * ROTATION_RESOLUTION;
                    arg_horiz = arg_horiz > 360 ? (arg_horiz - 360) : arg_horiz;
                    float arg_horiz_orginal = (14.67 - arg_horiz) * M_PI / 180;

                    pcl::PointXYZI point;
                    //VPoint point;

                    //if ((azimuth_corrected_f < scan_start_angle_) || (azimuth_corrected_f > scan_end_angle_)) continue;
                    if (distance2 > max_distance_ || distance2 < min_distance_) {
                        point.x = NAN;
                        point.y = NAN;
                        point.z = NAN;
                        point.intensity = 0;
                        //point.lines = dsr;
                        pointcloud->at(2 * this->block_num + firing, dsr) = point;
                        uint32_t timestamp = (LSC16_BLOCK_TDURATION*this->block_num + LSC16_FIRING_TOFFSET*firing + LSC16_DSR_TOFFSET*dsr)*1e3;
                        time_vect.push_back(timestamp);
                    } else {

                        point.x = distance2 * cos_scan_altitude_caliration[dsr] * cos_azimuth +
                                  R1_ * cos(arg_horiz_orginal);
                        point.y = -distance2 * cos_scan_altitude_caliration[dsr] * sin_azimuth +
                                  R1_ * sin(arg_horiz_orginal);
                        point.z = distance2 * sin_scan_altitude_caliration[dsr] + 0.426 / 100.f;

                        point.intensity = intensity;
                        //point.lines = dsr;

                        pointcloud->at(2 * this->block_num + firing, dsr) = point;

                        sweep_data->scans[dsr].points.push_back(lslidar_c16_msgs::msg::LslidarC16Point());
                        lslidar_c16_msgs::msg::LslidarC16Point &new_point = sweep_data->scans[dsr].points[
                                sweep_data->scans[dsr].points.size() - 1];

                        // Pack the data into point msg
                        new_point.x = point.x;
                        new_point.y = point.y;
                        new_point.z = point.z;
                        new_point.azimuth = angles::from_degrees(azimuth_corrected * ROTATION_RESOLUTION);
                        new_point.distance = sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2));
                        new_point.intensity = point.intensity;

                        uint32_t timestamp = (LSC16_BLOCK_TDURATION*this->block_num + LSC16_FIRING_TOFFSET*firing + LSC16_DSR_TOFFSET*dsr)*1e3;
                        time_vect.push_back(timestamp);
                        // RCLCPP_INFO(private_nh_->get_logger(), "block_number: %d", this->block_num);
                        // RCLCPP_INFO(private_nh_->get_logger(), "firing: %d", firing);
                        // RCLCPP_INFO(private_nh_->get_logger(), "dsr: %d", dsr);
                        // RCLCPP_INFO(private_nh_->get_logger(), "timestamp pointcloud nanosec: %u", timestamp);
                    }
                }
            }
        }
    }


}  // namespace lslidar_c16_decoder

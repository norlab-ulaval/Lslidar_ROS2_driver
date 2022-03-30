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


#include "lslidar_ch_decoder/lslidar_ch_decoder.h"


using namespace std;

namespace lslidar_ch_decoder {
    LslidarChDecoder::LslidarChDecoder() : LslidarChDecoder(rclcpp::NodeOptions()) {}

    LslidarChDecoder::LslidarChDecoder(const rclcpp::NodeOptions &options) :
            Node("cloud_node", options),
            sweep_data(new lslidar_ch_msgs::msg::LslidarChScan()) {
            this->initialize();
    }

    bool LslidarChDecoder::loadParameters() {
        this->declare_parameter<double>("min_range", 0.3);
        this->declare_parameter<double>("max_range", 200);
        this->declare_parameter<bool>("use_gps_ts", false);
        this->declare_parameter<string>("topic_name", "lslidar_point_cloud");
        this->declare_parameter<string>("frame_id", "lslidar");

        this->get_parameter<double>("min_range", min_range);
        this->get_parameter<double>("max_range", max_range);
        this->get_parameter<bool>("use_gps_ts", use_gps_ts);
        this->get_parameter<string>("topic_name", topic_name);
        this->get_parameter<string>("frame_id", frame_id);

        RCLCPP_INFO(this->get_logger(), "Using GPS timestamp or not %d", use_gps_ts);
        return true;
    }

    bool LslidarChDecoder::createRosIO() {

        packet_sub = this->create_subscription<lslidar_ch_msgs::msg::LslidarChPacket>("lslidar_packet", 10,
                                                                                              std::bind(
                                                                                                      &LslidarChDecoder::packetCallback,
                                                                                                      this,
                                                                                                      std::placeholders::_1));

     /*this->create_subscription<lslidar_ch_msgs::msg::LslidarChPacket>
                ("lslidar_packet", 10, bind(&LslidarChDecoder::packetCallback, this, placeholders::_1));
*/
        point_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name, 10);
        this->create_subscription<sensor_msgs::msg::PointCloud2>("lslidar_packet",10,
                std::bind(
                        &LslidarChDecoder::packetCallback,
                        this,
                        placeholders::_1
                        ));

        return true;

    }

    bool LslidarChDecoder::initialize() {
        if (!loadParameters()) {
            RCLCPP_INFO(this->get_logger(), "Cannot load all required parameters...");
            return false;
        }

        if (!createRosIO()) {
            RCLCPP_INFO(this->get_logger(), "Cannot create ROS I/O...");
            return false;
        }

        return true;
    }

    void LslidarChDecoder::publishPointCloud() {

        pcl::PointCloud<VPoint>::Ptr point_cloud(new pcl::PointCloud<VPoint>);
        point_cloud->header.frame_id = frame_id;
        point_cloud->height = 1;

        // The first and last point in each scan is ignored, which
        // seems to be corrupted based on the received data.
        // TODO: The two end points should be removed directly
        //    in the scans.
        uint64_t timestamp = get_clock()->now().nanoseconds();

        size_t j;
        VPoint point;
        if (sweep_data->points.size() > 0) {
            if (use_gps_ts) {
                point_cloud->header.stamp = pcl_conversions::toPCL(packet_timeStamp);
            } else {
                point_cloud->header.stamp = timestamp;
            }

            for (j = 1; j < sweep_data->points.size() - 1; ++j) {
                point.timestamp = sweep_data->points[j].time;
                point.x = sweep_data->points[j].x;
                point.y = sweep_data->points[j].y;
                point.z = sweep_data->points[j].z;
                point.intensity = sweep_data->points[j].intensity;
                point.ring = sweep_data->points[j].line;
                point_cloud->points.push_back(point);
                ++point_cloud->width;
            }

            sensor_msgs::msg::PointCloud2 pc_msg;
            pcl::toROSMsg(*point_cloud, pc_msg);
            point_cloud_pub->publish(pc_msg);
        }
        return;
    }


    void LslidarChDecoder::decodePacket(const RawPacket *packet) {

        // Compute the values for each firing
        for (size_t point_idx = 0; point_idx < POINTS_PER_PACKET; point_idx++) {
            //firings[point_idx].vertical_angle=verticalLineToAngle(packet->points[point_idx].vertical_line);
            firings[point_idx].vertical_line = packet->points[point_idx].vertical_line;
            TwoBytes point_amuzith;
            point_amuzith.bytes[0] = packet->points[point_idx].azimuth_2;
            point_amuzith.bytes[1] = packet->points[point_idx].azimuth_1;
            firings[point_idx].azimuth = static_cast<double>(point_amuzith.value) * 0.01 * M_PI / 180;
            ThreeBytes point_distance;
            point_distance.bytes[0] = packet->points[point_idx].distance_3;
            point_distance.bytes[1] = packet->points[point_idx].distance_2;
            point_distance.bytes[2] = packet->points[point_idx].distance_1;
            firings[point_idx].distance = static_cast<double>(point_distance.value) * DISTANCE_RESOLUTION;
            firings[point_idx].intensity = packet->points[point_idx].intensity;
        }
        return;
    }


    int LslidarChDecoder::checkPacketValidity(const RawPacket *packet) {

        //ROS_WARN("packet factory is %2x  %2x", packet->factory[0],packet->factory[1]);
        for (size_t blk_idx = 0; blk_idx < POINTS_PER_PACKET; blk_idx++) {
            if ((packet->points[blk_idx].vertical_line == 0xff) && (packet->points[blk_idx].azimuth_1 == 0xaa) &&
                (packet->points[blk_idx].azimuth_2 == 0xbb)) {
                return true;
            }
        }
        return false;

    }

    void LslidarChDecoder::packetCallback(
            const lslidar_ch_msgs::msg::LslidarChPacket::SharedPtr packet_msg) {

        // Convert the msg to the raw packet type.
        const RawPacket *raw_packet = (const RawPacket *) (&(packet_msg->data[0]));
        packet_timeStamp = packet_msg->header.stamp;

        packet_end_timeStamp = packet_timeStamp.nanoseconds();


        // Check if the packet is valid and find the header of frame
        bool packetType = checkPacketValidity(raw_packet);

        // Decode the packet
        decodePacket(raw_packet);
        double x = 0.0, y = 0.0, z = 0.0;
        double z_sin_altitude = 0.0;
        double z_cos_altitude = 0.0;
        //double cos_azimuth_half = 0.0;
        double sinTheta_1[128] = {0};
        double sinTheta_2[128] = {0};

        for (int i = 0; i < 128; i++) {
            sinTheta_1[i] = sin(big_angle[i / 4] * M_PI / 180);
            sinTheta_2[i] = sin((i % 4) * (-0.17) * M_PI / 180);
        }
        for (size_t point_idx = 0; point_idx < POINTS_PER_PACKET; point_idx++) {

            // Check if the point is valid.
            if (!isPointInRange(firings[point_idx].distance) || firings[point_idx].vertical_line > 128) continue;

            // Convert the point to xyz coordinate


            double azimuth_angle = firings[point_idx].azimuth;

            //double cos_azimuth_half=cos(firings[point_idx].azimuth*0.5);
            z_sin_altitude = sinTheta_1[firings[point_idx].vertical_line] +
                             2 * cos(azimuth_angle * 0.5 ) * sinTheta_2[firings[point_idx].vertical_line];

            z_cos_altitude = sqrt(1 - pow(z_sin_altitude, 2));
            x = firings[point_idx].distance * z_cos_altitude * cos(firings[point_idx].azimuth);
            y = firings[point_idx].distance * z_cos_altitude * sin(firings[point_idx].azimuth);
            z = firings[point_idx].distance * z_sin_altitude;
            double x_coord = x;
            double y_coord = y;
            double z_coord = z;

            // Compute the time of the point
            double point_time = static_cast<double>((packet_end_timeStamp - 868 * (POINTS_PER_PACKET - point_idx -1)) * 1e-9);


            sweep_data->points.push_back(lslidar_ch_msgs::msg::LslidarChPoint());
            lslidar_ch_msgs::msg::LslidarChPoint &new_point =        // new_point 为push_back最后一个的引用
                    sweep_data->points[sweep_data->points.size() - 1];
            // Pack the data into point msg
            new_point.time = point_time;
            new_point.x = x_coord;
            new_point.y = y_coord;
            new_point.z = z_coord;
            new_point.vertical_angle = verticalLineToAngle(firings[point_idx].vertical_line);
            new_point.azimuth = firings[point_idx].azimuth;
            new_point.distance = firings[point_idx].distance;
            new_point.intensity = firings[point_idx].intensity;
            new_point.line = firings[point_idx].vertical_line;
        }
        if (packetType) {
            publishPointCloud();
            sweep_data = lslidar_ch_msgs::msg::LslidarChScan::UniquePtr(
                    new lslidar_ch_msgs::msg::LslidarChScan());
        }

    }

} // end namespace lslidar_ch_decoder

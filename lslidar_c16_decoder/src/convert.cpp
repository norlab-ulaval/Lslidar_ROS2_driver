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

#include "lslidar_c16_decoder/convert.hpp"

namespace lslidar_c16_decoder {
    Convert::Convert() : Convert(rclcpp::NodeOptions()) {}

/** @brief Constructor. */
    Convert::Convert(const rclcpp::NodeOptions &options) : Node("cloud_node", options),
                                                           data_(new lslidar_rawdata::RawData(this)) {
        data_->loadConfigFile();  // load lidar parameters

        std::string output_points_topic = std::string("lslidar_point_cloud");
        std::string input_packets_topic = std::string("lslidar_packet");
        time_synchronization_ = false;
        scan_num = 1;
        publish_scan = false;
        scan_frame_id = std::string("lidar_link");
        this->declare_parameter<std::string>("output_points_topic", "lslidar_point_cloud");
        this->declare_parameter<std::string>("input_packets_topic", "lslidar_packet");
        this->declare_parameter<bool>("time_synchronization", false);
        this->declare_parameter<int>("scan_num", 1);
        this->declare_parameter<bool>("publish_scan", false);
        this->declare_parameter<std::string>("scan_frame_id", "lidar_link");

        this->get_parameter("output_points_topic", output_points_topic);
        this->get_parameter("input_packets_topic", input_packets_topic);
        this->get_parameter("time_synchronization", time_synchronization_);
        this->get_parameter("scan_num", scan_num);
        this->get_parameter("publish_scan", publish_scan);
        this->get_parameter("scan_frame_id", scan_frame_id);
        if (scan_num < 0) scan_num = 0;
        else if (scan_num > 15) scan_num = 15;
        output_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_points_topic, 10);
        scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 100);
        packet_sub_ = this->create_subscription<lslidar_c16_msgs::msg::LslidarC16ScanUnified>(input_packets_topic, 10,
                                                                                              std::bind(
                                                                                                      &Convert::processScan,
                                                                                                      this,
                                                                                                      std::placeholders::_1));

        if (time_synchronization_) {
            sync_sub_ = this->create_subscription<sensor_msgs::msg::TimeReference>("sync_header", 10,
                                                                                   std::bind(&Convert::timeSync, this,
                                                                                             std::placeholders::_1));
            //sync_sub_ = node.subscribe("sync_header", 10, &Convert::timeSync, (Convert*)this, ros::TransportHints().tcpNoDelay(true));
        }

        sweep_data = lslidar_c16_msgs::msg::LslidarC16Sweep::UniquePtr(
                new lslidar_c16_msgs::msg::LslidarC16Sweep());
    }

    void Convert::timeSync(const sensor_msgs::msg::TimeReference::SharedPtr time_msg) {
        global_time = time_msg->header.stamp;
    }

    void Convert::publishScan(lslidar_c16_msgs::msg::LslidarC16Sweep::SharedPtr sweep_data, int scan_num) {
        sensor_msgs::msg::LaserScan::UniquePtr scan(new sensor_msgs::msg::LaserScan);


        if (sweep_data->scans[scan_num].points.size() <= 1) return;

        uint16_t point_num = 2000;
        double angle_base = M_PI * 2 / point_num;

        scan->header.frame_id = scan_frame_id;
        scan->header.stamp = sweep_data->header.stamp;  // timestamp will obtained from sweep data stamp

        scan->scan_time = 1 / 10.0;  //s
        scan->time_increment = scan->scan_time / point_num;  //s

        scan->angle_min = 0;
        scan->angle_max = M_PI * 2;
        scan->angle_increment = (scan->angle_max - scan->angle_min) / point_num;

        scan->range_min = 0.15;
        scan->range_max = 150;
        scan->ranges.reserve(point_num);
        scan->ranges.assign(point_num, std::numeric_limits<float>::infinity());
        scan->intensities.reserve(point_num);
        scan->intensities.assign(point_num, std::numeric_limits<float>::infinity());

        for (uint16_t i = 0; i < sweep_data->scans[scan_num].points.size() - 1; i++) {
            int point_idx = sweep_data->scans[scan_num].points[i].azimuth / angle_base;

            if (point_idx >= point_num)
                point_idx = 0;
            if (point_idx < 0)
                point_idx = point_num - 1;

            scan->ranges[point_num - 1 - point_idx] = sweep_data->scans[scan_num].points[i].distance;
            scan->intensities[point_num - 1 - point_idx] = sweep_data->scans[scan_num].points[i].intensity;
        }
        scan_pub->publish(std::move(scan));
    }


/** @brief Callback for raw scan messages. */
    void Convert::processScan(const lslidar_c16_msgs::msg::LslidarC16ScanUnified::UniquePtr scanMsg) {
        // lslidar_rawdata::VPointCloud::Ptr outPoints(new lslidar_rawdata::VPointCloud);
        pcl::PointCloud<pcl::PointXYZI>::Ptr outPoints(new pcl::PointCloud<pcl::PointXYZI>);
        sweep_data = std::make_shared<lslidar_c16_msgs::msg::LslidarC16Sweep>();


        outPoints->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
        sweep_data->header.stamp = scanMsg->header.stamp;
        scan_timestamp = scanMsg->header.stamp;


        outPoints->header.frame_id = scanMsg->header.frame_id;
        outPoints->clear();
        outPoints->height = 16;
        outPoints->width = 24 * (int) scanMsg->packets.size();
        outPoints->is_dense = false;
        outPoints->resize(outPoints->height * outPoints->width);

        int blockNum, dsr;
        //lslidar_rawdata::VPoint point;
        pcl::PointXYZI point;
        point.x = NAN;
        point.y = NAN;
        point.z = NAN;
        point.intensity = 0;
        for (size_t j = 0; j < outPoints->height * outPoints->width; ++j) {
            blockNum = j / 16;
            dsr = j % 16;
            outPoints->at(blockNum, dsr) = point;
        }

        // process each packet provided by the driver
        data_->block_num = 0;
        std::vector<u_int32_t> time_vect;

        for (size_t i = 0; i < scanMsg->packets.size(); ++i) {
            data_->unpack(scanMsg->packets[i], outPoints, sweep_data, i, time_vect);
        }

        if (publish_scan)
            publishScan(sweep_data, scan_num);
        //pcl::removeNaNFromPointCloud(*outPoints, *outPoints, indices);
        std::vector<float> x_vect;
        std::vector<float> y_vect;
        std::vector<float> z_vect;
        std::vector<float> intensity_vect;
        int num_of_points = 0;

        for (size_t i = 0; i < outPoints->points.size(); ++i){
            x_vect.push_back(outPoints->points[i].x);
            y_vect.push_back(outPoints->points[i].y);
            z_vect.push_back(outPoints->points[i].z);
            intensity_vect.push_back(outPoints->points[i].intensity);
            num_of_points += 1;
        }


        sensor_msgs::msg::PointCloud2 outMsg;
        outMsg.header.stamp = scanMsg->header.stamp;
	    outMsg.header.frame_id = scan_frame_id;
        int height = 16;
        int width = int(time_vect.size()/height)+1; //TODO ceil? 
        bool is_dense = false;

        outMsg.height = height;
        outMsg.width = width;
        outMsg.is_dense = is_dense;

        sensor_msgs::PointCloud2Modifier pcd_modifier(outMsg);
        // this call also resizes the data structure according to the given width, height and fields
        pcd_modifier.setPointCloud2Fields(5, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                                            "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
                                            "t", 1, sensor_msgs::msg::PointField::UINT32);

        sensor_msgs::PointCloud2Iterator<float> iter_x(outMsg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(outMsg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(outMsg, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_i(outMsg, "intensity");
        sensor_msgs::PointCloud2Iterator<u_int32_t> iter_t(outMsg, "t");

        // RCLCPP_INFO(this->get_logger(), "x size : %lu", x_vect.size());
        // RCLCPP_INFO(this->get_logger(), "time size: %lu", time_vect.size());
        // RCLCPP_INFO(this->get_logger(), "pointcloud size : %d", outMsg.width*outMsg.height);


        int index_in_vectors = 0;
        for (; iter_x!=iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_i, ++iter_t, ++index_in_vectors)
        {
            if (index_in_vectors>=time_vect.size())
            {
                *iter_x = NAN;
                *iter_y = NAN;
                *iter_z = NAN;
                *iter_i = NAN;
                *iter_t = NAN;
            }
            // copy the data
            else{
                *iter_x = x_vect[index_in_vectors];
                *iter_y = y_vect[index_in_vectors];
                *iter_z = z_vect[index_in_vectors];
                *iter_i = intensity_vect[index_in_vectors];
                *iter_t = time_vect[index_in_vectors];
            }
        }
         
        output_->publish(outMsg);
    }
}  // namespace lslidar_c16_decoder

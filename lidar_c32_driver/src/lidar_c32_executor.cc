/*
 * This file is part of lidar_c32 driver.
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

#include "rclcpp/rclcpp.hpp"
#include "lidar_c32_driver/lidar_c32_driver.h"
#include "lidar_c32_decoder/convert.hpp"

int main(int argc, char* argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exec;
  
  rclcpp::NodeOptions options(rclcpp::NodeOptions().use_intra_process_comms(true));

  auto driver = std::make_shared<lidar_c32_driver::lidarDriver>(options);
  exec.add_node(driver);
																					  
  //auto converter = std::make_shared<lidar_c32_decoder::Convert>(options);
  //exec.add_node(converter);				

  exec.spin();

  rclcpp::shutdown();

  return 0;
}
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


#include "lidar_c32_driver/input.h"

namespace lidar_c32_driver
{
static const size_t packet_size = sizeof(lidar_c32_msgs::msg::LidarC32Packet().data);
////////////////////////////////////////////////////////////////////////
// Input base class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
 *
 *  @param private_nh ROS private handle for calling node.
 *  @param port UDP port number.
 */
int Input_first_time = 0;
static void my_handler(int sig)
{
	//flag = 0;
	abort();
}

Input::Input(rclcpp::Node* private_nh, uint16_t port) : private_nh_(private_nh), port_(port)
{
  npkt_update_flag_ = false;
  cur_rpm_ = 0;
  return_mode_ = 1;
  flag = 1;
  signal(SIGINT, my_handler);
  
  if(Input_first_time == 0)
  {
	  private_nh->declare_parameter("device_ip");
	  private_nh->declare_parameter("add_multicast");
	  private_nh->declare_parameter("group_ip");
	  Input_first_time++;
  }
  private_nh->get_parameter("device_ip", devip_str_);
  private_nh->get_parameter("add_multicast", add_multicast);
  private_nh->get_parameter("group_ip", group_ip);

  if (!devip_str_.empty())
	  RCLCPP_INFO(private_nh->get_logger(), "[driver][input] accepting packets from IP address: %s", devip_str_.c_str());
}

int Input::getRpm(void)
{
  return cur_rpm_;
}

int Input::getReturnMode(void)
{
  return return_mode_;
}

bool Input::getUpdateFlag(void)
{
  return npkt_update_flag_;
}

void Input::clearUpdateFlag(void)
{
  npkt_update_flag_ = false;
}
////////////////////////////////////////////////////////////////////////
// InputSocket class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
   *
   *  @param private_nh ROS private handle for calling node.
   *  @param port UDP port number
*/
InputSocket::InputSocket(rclcpp::Node* private_nh, uint16_t port) : Input(private_nh, port)
{
  msop_timeout = false;
  difop_timeout = false;																				 
  sockfd_ = -1;

  if (!devip_str_.empty())
  {
    inet_aton(devip_str_.c_str(), &devip_);
  }

  RCLCPP_INFO(private_nh_->get_logger(), "[driver][socket] Opening UDP socket: port %d", port);
  sockfd_ = socket(PF_INET, SOCK_DGRAM, 0);
  if (sockfd_ == -1)
  {
    RCLCPP_ERROR(private_nh_->get_logger(), "[driver][socket] create socket fail");
    return;
  }

  int opt = 1;
  if (setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, (const void*)&opt, sizeof(opt)))
  {
    RCLCPP_ERROR(private_nh_->get_logger(), "[driver][socket] setsockopt fail");
    return;
  }

  sockaddr_in my_addr;                   // my address information
  memset(&my_addr, 0, sizeof(my_addr));  // initialize to zeros
  my_addr.sin_family = AF_INET;          // host byte order
  my_addr.sin_port = htons(port);        // port in network byte order
  my_addr.sin_addr.s_addr = INADDR_ANY;  // automatically fill in my IP

  if (bind(sockfd_, (sockaddr*)&my_addr, sizeof(sockaddr)) == -1)
  {
    RCLCPP_ERROR(private_nh_->get_logger(), "[driver][socket] socket bind fail");
    return;
  }
	
	if (add_multicast) {
            struct ip_mreq group;
            group.imr_multiaddr.s_addr = inet_addr(group_ip.c_str());
			group.imr_interface.s_addr =  htonl(INADDR_ANY);
			//group.imr_interface.s_addr = inet_addr("192.168.1.102");

            if (setsockopt(sockfd_, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *) &group, sizeof(group)) < 0) {
                RCLCPP_ERROR(private_nh_->get_logger(), "Adding multicast group error");
                close(sockfd_);
                exit(1);
            } else
				RCLCPP_INFO(private_nh_->get_logger(), "Adding multicast group...OK.\n");
        }
		
  if (fcntl(sockfd_, F_SETFL, O_NONBLOCK | FASYNC) < 0)
  {
    RCLCPP_ERROR(private_nh_->get_logger(), "[driver][socket] fcntl fail");
    return;
  }
}

/** @brief destructor */
InputSocket::~InputSocket(void)
{
  (void)close(sockfd_);
}

/** @brief Get one lidar packet. */
int InputSocket::getPacket(lidar_c32_msgs::msg::LidarC32Packet& pkt, const double time_offset, std::string pkg_name)
{
  double time1 = private_nh_->get_clock()->now().seconds();
  struct pollfd fds[1];
  fds[0].fd = sockfd_;
  fds[0].events = POLLIN;
  static const int POLL_TIMEOUT = 3000;  // one second (in msec)

  sockaddr_in sender_address;
  socklen_t sender_address_len = sizeof(sender_address);
 
  while (flag == 1)
  {
    // Receive packets that should now be available from the
    // socket using a blocking read.
    // poll() until input available
    do
    {
	  int retval;
	  retval = poll(fds, 1, POLL_TIMEOUT);

      if (retval < 0)  // poll() error?
      {
        if (errno != EINTR)
			RCLCPP_ERROR(private_nh_->get_logger(), "[driver][socket] poll() error: %s", strerror(errno));
        return 1;
      }
      if (retval == 0)  // poll() timeout?
      {
		time_t curTime = time(NULL);
		struct tm *curTm = localtime(&curTime);
		char bufTime[30] = {0};
		sprintf(bufTime, "%d-%d-%d %d:%d:%d", curTm->tm_year + 1900, curTm->tm_mon + 1,
				curTm->tm_mday, curTm->tm_hour, curTm->tm_min, curTm->tm_sec);
		
		if(pkg_name == "msop_pkg"){
			RCLCPP_WARN(private_nh_->get_logger(), "lidar msop poll() timeout: %s", devip_str_.c_str());
		}
		if(pkg_name == "difop_pkg"){
			RCLCPP_WARN(private_nh_->get_logger(), "lidar difop poll() timeout:  %s", devip_str_.c_str());
		}  
        return 1;
      }
      if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) || (fds[0].revents & POLLNVAL))  // device error?
      {
        RCLCPP_ERROR(private_nh_->get_logger(), "[driver][socket] poll() reports Rslidar error");
        return 1;
      }
    } while ((fds[0].revents & POLLIN) == 0);
    ssize_t nbytes = recvfrom(sockfd_, &pkt.data[0], packet_size, 0, (sockaddr*)&sender_address, &sender_address_len);

    if (nbytes < 0)
    {
      if (errno != EWOULDBLOCK)
      {
        RCLCPP_ERROR(private_nh_->get_logger(), "[driver][socket] recvfail");
        return 1;
      }
    }
    else if ((size_t)nbytes == packet_size)
    {
      if (devip_str_ != "" && sender_address.sin_addr.s_addr != devip_.s_addr)
        continue;
      else
        break;  // done
    }
	RCLCPP_WARN(private_nh_->get_logger(), "[driver][socket] incomplete rslidar packet read: %d bytes", nbytes);
  }
  if (flag == 0)
  {
    abort();
  }

  if (pkt.data[0] == 0xA5 && pkt.data[1] == 0xFF && pkt.data[2] == 0x00 && pkt.data[3] == 0x5A)
  {
	 //difop
    int rpm = (pkt.data[8]<<8)|pkt.data[9];
	 // ROS_INFO("rpm=%d",rpm);
    int mode = 1;
    if (cur_rpm_ != rpm || return_mode_ != mode)
    {
      cur_rpm_ = rpm;
      return_mode_ = mode;
      npkt_update_flag_ = true;
    }
  }
  // Average the times at which we begin and end reading.  Use that to
  // estimate when the scan occurred. Add the time offset.
  double time2 = private_nh_->get_clock()->now().seconds();
  pkt.stamp = rclcpp::Time(((time2 + time1) / 2.0 + time_offset)*1e9);
  return 0;
}

////////////////////////////////////////////////////////////////////////
// InputPCAP class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
   *  @param private_nh ROS private handle for calling node.
   *  @param port UDP port number
   *  @param packet_rate expected device packet frequency (Hz)
   *  @param filename PCAP dump file name
   */
/*InputPCAP::InputPCAP(rclcpp::Node* private_nh, uint16_t port, double packet_rate, std::string filename,
                     bool read_once, bool read_fast, double repeat_delay)
  : Input(private_nh, port), packet_rate_(packet_rate), filename_(filename)
{
  pcap_ = NULL;
  empty_ = true;

  // get parameters using private node handle
  private_nh->get_parameter_or("read_once", read_once_, false);
  private_nh->get_parameter_or("read_fast", read_fast_, false);
  private_nh->get_parameter_or("repeat_delay", repeat_delay_, 0.0);

  if (read_once_)                     
  if (read_fast_)
  {
    RCLCPP_INFO(private_nh_->get_logger(), "[driver][pcap] Read input file as quickly as possible.");
  }
  if (repeat_delay_ > 0.0)
  {
    RCLCPP_INFO(private_nh_->get_logger(), "[driver][pcap] Delay %.3f seconds before repeating input file.", repeat_delay_);
  }

  // Open the PCAP dump file
  RCLCPP_INFO(private_nh_->get_logger(), "[driver][pcap] Opening PCAP file \"%s\"", filename_.c_str());
  if ((pcap_ = pcap_open_offline(filename_.c_str(), errbuf_)) == NULL)
  {
    RCLCPP_FATAL(private_nh_->get_logger(), "[driver][pcap] Error opening rslidar socket dump file.");
    return;
  }

  std::stringstream filter;
  if (devip_str_ != "")  // using specific IP?
  {
    filter << "src host " << devip_str_ << " && ";
  }
  filter << "udp dst port " << port;
  int ret = pcap_compile(pcap_, &pcap_packet_filter_, filter.str().c_str(), 1, PCAP_NETMASK_UNKNOWN);
  if (ret < 0)
  {
    auto x = filter.str();
    RCLCPP_FATAL(private_nh_->get_logger(), "[driver][pcap] pcap compile filter fail. filter: %s", x.c_str());
    return;
  }
}*/

/** destructor */
/*InputPCAP::~InputPCAP(void)
{
  pcap_close(pcap_);
}*/

/** @brief Get one lidar packet. */
/*int InputPCAP::getPacket(lidar_c32_msgs::msg::LidarC32Packet& pkt, const double time_offset, std::string pkg_name)
{
	struct pcap_pkthdr* header;
	const u_char* pkt_data;

  while (rclcpp::ok())
  {
    int res;
    if ((res = pcap_next_ex(pcap_, &header, &pkt_data)) >= 0)
    {
      // Skip packets not for the correct port and from the
      // selected IP address.
      if (!devip_str_.empty() && (0 == pcap_offline_filter(&pcap_packet_filter_, header, pkt_data)))
        continue;

      // Keep the reader from blowing through the file.
      if (read_fast_ == false)
        packet_rate_.sleep();

      memcpy(&pkt.data[0], pkt_data + 42, packet_size);

      if (pkt.data[0] == 0xA5 && pkt.data[1] == 0xFF && pkt.data[2] == 0x00 && pkt.data[3] == 0x5A)
      {
        // difop
        int rpm = (pkt.data[8]<<8)|pkt.data[9];
        int mode = 1;

        if ((pkt.data[45] == 0x08 && pkt.data[46] == 0x02 && pkt.data[47] >= 0x09) || (pkt.data[45] > 0x08)
            || (pkt.data[45] == 0x08 && pkt.data[46] > 0x02))
        {
          if (pkt.data[300] != 0x01 && pkt.data[300] != 0x02)
          {
            mode = 0;
          }
        }

        if (cur_rpm_ != rpm || return_mode_ != mode)
        {
          cur_rpm_ = rpm;
          return_mode_ = mode;
          npkt_update_flag_ = true;
        }
      }

      pkt.stamp = private_nh_->get_clock()->now();  // time_offset not considered here, as no
                                      // synchronization required
      empty_ = false;
      return 0;  // success
    }

    if (empty_)  // no data in file?
    {
      RCLCPP_ERROR(private_nh_->get_logger(), "[driver][pcap] Error %d reading rslidar packet: %s", res, pcap_geterr(pcap_));
      return -1;
    }

    if (read_once_)
    {
      RCLCPP_INFO(private_nh_->get_logger(), "[driver][pcap] end of file reached -- done reading.");
      return -1;
    }

    if (repeat_delay_ > 0.0)
    {
      RCLCPP_INFO(private_nh_->get_logger(), "[driver][pcap] end of file reached -- delaying %.3f seconds.", repeat_delay_);
      usleep(rint(repeat_delay_ * 1000000.0));
    }

    RCLCPP_INFO(private_nh_->get_logger(), "[driver][pcap] replaying rslidar dump file");

    // I can't figure out how to rewind the file, because it
    // starts with some kind of header.  So, close the file
    // and reopen it with pcap.
    pcap_close(pcap_);
    pcap_ = pcap_open_offline(filename_.c_str(), errbuf_);
    empty_ = true;  // maybe the file disappeared?
  }                 // loop back and try again
  if (!rclcpp::ok())
  {
    return -1;
  }
  return 0;
}*/
}

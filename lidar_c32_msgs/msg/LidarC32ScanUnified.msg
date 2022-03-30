#lidar C32 scan packets

std_msgs/Header  header         #standard ros message header
uint64           basetime       #time to the top of hour
LidarC32Packet[] packets        #vector of raw packets

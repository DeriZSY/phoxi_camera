#ifndef POHXI_CAMERA_CAL_DEV
#define POHXI_CAMERA_CAL_DEV
# include <string>
# include <iostream>
# include "ros/ros.h"
# include "phoxi_camera/ConnectCamera.h"
# include "phoxi_camera/SaveFrame.h"
# include "phoxi_camera/GetFrame.h"
# include "phoxi_camera/GetDeviceList.h"
# include "sensor_msgs/Image.h"
# include "sensor_msgs/PointCloud2.h"

// Macro Definition for coloring 
#define RD(str) "\033[31;5m"#str"\033[0m"
#define GR(str) "\033[32;5m"#str"\033[0m"

using namespace std;
using namespace ros;

void depth_im_call_back(const sensor_msgs::Image::ConstPtr& depth_im);
void pcl_call_back(const sensor_msgs::PointCloud2::ConstPtr& pcl_map);
bool phoxi_connect(const string& cam_id);
bool get_frame(const int id);

#endif
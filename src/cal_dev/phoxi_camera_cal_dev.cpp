# include "phoxi_camera_cal_dev.h"
using namespace std;
using namespace ros;

void depth_im_call_back(const sensor_msgs::Image::ConstPtr& depth_im)
{
    ROS_INFO("Depth map received in callback function!!!");
}

void pcl_call_back(const sensor_msgs::PointCloud2::ConstPtr& pcl_map)
{
    ROS_INFO("Point cloud map received in callback function!!!");
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "phoxi_cal_dev");
    ros::NodeHandle n;
    // Connect to the camera on PhoxiControler
    ros::ServiceClient ConnectCamera_client = n.serviceClient<phoxi_camera::ConnectCamera>("phoxi_camera/connect_camera");
    phoxi_camera::ConnectCamera ConnectCamera_srv;
    ConnectCamera_srv.request.name = "1711004";
    ConnectCamera_client.call(ConnectCamera_srv);
    if (ConnectCamera_srv.response.success)
        ROS_INFO("Camera Conncted");
    else 
    {
        cout << ConnectCamera_srv.response.message << endl;
        ROS_ERROR("Cammera Not Connecting");
    }
    // Obtain a frame from the camera
    ros::ServiceClient GetFrame_client = n.serviceClient<phoxi_camera::GetFrame>("phoxi_camera/get_frame");
    phoxi_camera::GetFrame GetFrame_srv;
    GetFrame_srv.request.in = -1;
    if (GetFrame_client.call(GetFrame_srv))
    {
        ROS_INFO("Get Image");
        cout << GetFrame_srv.response.message << endl;
        ros::Subscriber sub_depthMap = n.subscribe("/phoxi_camera/depth_map",1000,depth_im_call_back);
        ros::Subscriber sub_pointCloud = n.subscribe("/phoxi_camera/pointcloud", 1000, pcl_call_back);
        ros::spin();
    }
    else
    {
        ROS_INFO("Failed");
        return -1;
    }
    

    return 0;
} 
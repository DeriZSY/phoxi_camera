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
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(*msg, pcl_pc);
    pcl::fromPCLPointCloud2(pcl_pc, cloud);
}

bool phoxi_connect(const string& cam_id)
{
    ros::NodeHandle n;
    ros::ServiceClient ConnectCamera_client = n.serviceClient<phoxi_camera::ConnectCamera>("phoxi_camera/connect_camera");
    phoxi_camera::ConnectCamera ConnectCamera_srv;
    ConnectCamera_srv.request.name = cam_id;
    if (ConnectCamera_client.call(ConnectCamera_srv)) { ROS_INFO("Camera Conncted"); }
    else 
    {
        ROS_ERROR("Cammera Not Connected");
        cout << RD("Filed with Message:") << ConnectCamera_srv.response.message << endl;
    }
    return ConnectCamera_srv.response.success;
}

bool get_frame(const int id)
{
    ros::NodeHandle n;
    ros::ServiceClient GetFrame_client = n.serviceClient<phoxi_camera::GetFrame>("phoxi_camera/get_frame");
    phoxi_camera::GetFrame GetFrame_srv;
    GetFrame_srv.request.in = id;
    if (GetFrame_client.call(GetFrame_srv)) { ROS_INFO("Get Frame Success");}
    else                       
    { 
        ROS_ERROR("Get Frame Failed"); 
        cout << RD(Failed With Message:) << GetFrame_srv.response.message << endl;
    }
    return GetFrame_srv.response.success;
}

int main (int argc, char** argv)
{
    double start, end, duration;
    // start = clock();
    ros::init(argc, argv, "phoxi_cal_dev");
    ros::NodeHandle n;
    // Connect to the camera on PhoxiControler
    bool connectCamera_success = phoxi_connect("1711004");
    bool getFrame_success = get_frame(-1);
    if (connectCamera_success && getFrame_success)
    {
        ros::Subscriber sub_depthMap = n.subscribe("/phoxi_camera/depth_map",1000,depth_im_call_back);
        ros::Subscriber sub_pointCloud = n.subscribe("/phoxi_camera/pointcloud", 1000, pcl_call_back);
        // end = clock();
        // cout <<GR(Duration:) << ((end - start)/1000)<< endl;
        ros::spin();
    }
    return 0;
} 
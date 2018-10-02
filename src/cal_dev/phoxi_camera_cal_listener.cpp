# include "phoxi_camera_cal_lib.h"
using namespace std;
using namespace ros;

void depth_im_call_back(const sensor_msgs::Image::ConstPtr& depth_im)
{
    ROS_INFO("Depth map received in callback function!!!");
    return;
}

void pcl_call_back(const sensor_msgs::PointCloud2::ConstPtr& pcl_map)
{
    static int count = 0;
    count++;
    ROS_INFO("Point cloud map received in callback function!!!");
    string im_path;
    ros::param::get("~image_path", im_path);
    im_path = im_path + to_string(count) + ".PCD";
    cout<< GR([#INFO] Image Path: ) << im_path << endl;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(*pcl_map, pcl_pc);
    pcl::fromPCLPointCloud2(pcl_pc, cloud);
    pcl::io::savePCDFile(im_path,cloud);
    return;
}

int main(int argc,char** argv)
{
    ros::init(argc, argv, "phoxi_cal_listener");
    ros::NodeHandle n;
    ros::Subscriber sub_depthMap = n.subscribe("/phoxi_camera/depth_map",1000,depth_im_call_back);
    ros::Subscriber sub_pointCloud = n.subscribe("/phoxi_camera/pointcloud", 1000, pcl_call_back);
    ros::spin();
}
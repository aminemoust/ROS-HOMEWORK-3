/**
 * A Pcl viewer of the pcd saved with ORB_SLAM2
 * @author Amine Moustaghfir
 * @version 1.0
 **/
#include  <memory> // We need to include this for shared_ptr

#include<ros/ros.h>

//pcl library
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>    

void pcl_view(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg){
    ROS_INFO("Cloud: width = %d, height = %d\n", msg->width, msg->height);

    //visualize the Cloud with pcl
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
    viewer->setBackgroundColor(0,0,0);
    viewer->addPointCloud<pcl::PointXYZ>( msg, "sample cloud" );
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PCL_VIEWER");

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZ>>("cloud_pcd", 1, pcl_view);

    ros::spin();

    return 0;
}
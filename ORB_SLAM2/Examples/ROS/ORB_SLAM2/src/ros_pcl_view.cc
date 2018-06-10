/**
 * A Pcl viewer of the pcd saved with ORB_SLAM2
 * @author Amine Moustaghfir
 * @version 1.0
 **/
#include  <memory> // We need to include this for shared_ptr
#include <stdlib.h> 

#include<ros/ros.h>

//pcl library
//#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>   
//#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/kdtree/kdtree.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

float tolerance;

void create_cluster(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_in, std::vector<pcl::PointIndices>& cluster_indices,
                        std::shared_ptr<pcl::visualization::PCLVisualizer>& viewer){
    pcl::PCDWriter writer;

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {   
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            cloud_cluster->points.push_back (cloud_in->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
        std::stringstream ss;
        ss << "point/cloud_cluster_" << j << ".pcd";
        //writer.write<pcl::PointXYZ> (ss.str(), *cloud_cluster, false); //*
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> 
            cluster_color(cloud_cluster, rand()%256, rand()%256, rand()%256);
        viewer->addPointCloud<pcl::PointXYZ>(cloud_cluster, ss.str());
        j++;
    }
}

void pcl_view(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg){
    ROS_INFO("Cloud: width = %d, height = %d\n", msg->width, msg->height);

    //visualize the Cloud with pcl
    
    
    //clusterization 
    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud(msg);
    vg.setLeafSize(0.01, 0.01, 0.01);
    vg.filter(*cloud_filtered);
    std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << std::endl;

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setMinClusterSize (50);
    //ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    //ec.setClusterTolerance (0.15); // distance threshold
    ec.setClusterTolerance (tolerance);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

    ROS_INFO("Extracting from the cloud with : %i points. \nNum of clusters: %i", cloud_filtered->points.size(), cluster_indices.size());

    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
    viewer->setBackgroundColor(0,0,0);
    viewer->addCoordinateSystem(1.0);
    //Normal cloud viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered, "sample cloud" );
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    //viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered, "sample cloud" );
    viewer->initCameraParameters();

    //create_cluster(cloud_filtered, cluster_indices, viewer);
    std::cout << "Visualizing the clustering..." << std::endl; 

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {   
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
        std::stringstream ss;
        ss << "point/cloud_cluster_" << j << ".pcd";
        //writer.write<pcl::PointXYZ> (ss.str(), *cloud_cluster, false); //*
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> 
            cluster_color(cloud_cluster, rand()%256, rand()%256, rand()%256);
        viewer->addPointCloud<pcl::PointXYZ>(cloud_cluster, cluster_color, ss.str());
        j++;
    }

    
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(1);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PCL_VIEWER");

    if(argc != 2){
        std::cout << "Usage: rosrun ORB_SLAM2 PCL_VIEWER tolerance_value" << endl;
        return 1;
    }

    tolerance = atof(argv[1]);  // convert from string to float

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZ>>("cloud_pcd", 1, pcl_view);

    ros::spin();

    return 0;
}
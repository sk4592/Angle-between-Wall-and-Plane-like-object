#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <math.h>
#include <std_msgs/Float64.h>
#include <pcl/filters/passthrough.h>


using namespace std;

double a, b, c, d, a1, b1, c1, d1;
std_msgs::Float64 val;

std::vector<pcl::PointIndices> cluster_indices;  
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB> ());  
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZRGB> ());  
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB> ()), cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr final(new pcl::PointCloud<pcl::PointXYZRGB> ()) ;
sensor_msgs::PointCloud2::Ptr data(new sensor_msgs::PointCloud2());
sensor_msgs::PointCloud2::Ptr data2(new sensor_msgs::PointCloud2());
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
pcl::PointCloud<pcl::PointXYZRGB>::Ptr wall (new pcl::PointCloud<pcl::PointXYZRGB> ());
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);


void callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    auto now = ros::Time::now();
    // implement ransac plane segmentation
    // obtain points for duster using cluster
    // fit a plane to the duster and calculate angles between planes  
    // cout << msg->header.frame_id << "\n";
    cloud->clear();
    cloud_filtered->clear();
    final->clear();
    cloud_plane->clear();
    pcl::fromROSMsg(*msg, *cloud);
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*cloud_filtered);
    
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold (0.01); 
    seg.setInputCloud(cloud_filtered);
    seg.segment (*inliers, *coefficients);
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative(false);
    extract.filter(*wall); // wall
    extract.setNegative(true);
    extract.filter(*cloud_filtered2); // object


    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud_filtered2);
    pass.setFilterFieldName ("z");
    // cout << "coeffs\n";
    // cout << coefficients->values[3] << "\n";
    pass.setFilterLimits (0.0, abs(coefficients->values[3]));
    //pass.setFilterLimitsNegative (true);
    pass.filter (*final);


    /*
    /*
    int i=0, nr_points = (int) cloud_filtered->size ();

    while (cloud_filtered->size () > 0.5 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // Get the points associated with the planar surface
        extract.filter (*cloud_plane);
        std::cout << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;
        
        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_f);
        *cloud_filtered = *cloud_f; //object
        *cloud_filtered2 = *cloud_plane; // wall
    }
    */

    a = coefficients->values[0];
    b = coefficients->values[1];
    c = coefficients->values[2];
    d = coefficients->values[3];
    
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (final);

    
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (0.03); // 2cm
    ec.setMinClusterSize (800);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (final);
    ec.extract (cluster_indices);

    int j = 0;
    // cout << "hello\n";
    
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        // cout << "hi\n";
        cloud_cluster->clear();
        cout << cluster_indices.size() << "\n";
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->push_back ((*final)[*pit]); //*
        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
        
    }
    cout << "no of clusters\n";
    cout << cluster_indices.size() << "\n";
    
    cloud_plane->clear();
    
    
    seg.setInputCloud (cloud_cluster);
    seg.segment (*inliers, *coefficients);
    extract.setInputCloud (cloud_cluster);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    

    /*

    while (cloud_cluster->size () > 0.5 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_cluster);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        break;
        }
        

        // Extract the planar inliers from helloer);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // Get the points associated with the planar surface
        extract.filter (*cloud_plane);

        if (tmp == cloud_plane->size()) {
            break;
        }
        std::cout << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;
        tmp = cloud_plane->size();

        // Remove the planar inliers, extract the rest
        //extract.setNegative (true);
        //extract.filter (*cloud_f);
        //*cloud_filtered = *cloud_f;
        *cloud_cluster = *cloud_plane;
    }    
    
    */
    a1 = coefficients->values[0];
    b1 = coefficients->values[1];
    c1 = coefficients->values[2];
    d1 = coefficients->values[3];

    /*
    cout << a1 << " " << b1 << " " << c1 << " " << d1 << "\n";
    cout << cloud_cluster->points[0].x << " " << cloud_cluster->points[0].y << " " << cloud_cluster->points[0].z << "\n";
    cout << cloud_cluster->points[1].x << " " << cloud_cluster->points[1].y << " " << cloud_cluster->points[1].z << "\n";
    cout << cloud_cluster->points[2].x << " " << cloud_cluster->points[2].y << " " << cloud_cluster->points[2].z << "\n";
    */

    auto cross = (b*c1 - b1*c) - (a*c1 - a1*c) + (a*b1 - a1*b);

    auto dot = (a * a1) + (b * b1) + (c * c1);
    auto den = (sqrt(pow(a, 2) + pow(b, 2) + pow(c, 2))) * (sqrt(pow(a1, 2) + pow(b1, 2) + pow(c1, 2)));
    
    cout << "ang" << "\n";
    auto tmp = acos(abs(dot / den)) * 180 / M_PI;
    // auto cross = sqrt(1 - pow((dot/den), 2));
    //auto res = (cross / dot);
    auto ang = atan2(cross, dot);
    auto theta = (ang > M_PI_2) || (ang < 0) ? M_PI - acos(abs(dot / den)) : acos(abs(dot / den));
    cout << "ang " << ang * 180 / M_PI << "\n";
    cout << "cos" << tmp << "\n";
    // cout << "theta: " << theta * 180 / M_PI << "\n";
    val.data = theta * 180 / M_PI;
    
    
    pcl::toROSMsg(*cloud_plane, *data);
    pcl::toROSMsg(*wall, *data2);
    data->header.frame_id = "camera_depth_optical_frame";
    data2->header.frame_id = "camera_depth_optical_frame";
    cout << "time\n";
    cout << (ros::Time::now() - now).toSec() << "\n";
    //cout << data->header.frame_id << "\n";

    /*
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.015f, 0.015f, 0.015f);
    sor.filter(*cloud_filtered);
    
    */

}



int main(int argc, char **argv) {
    ros::init(argc, argv, "angle");
    ros::NodeHandle n;
    ros::Rate rate(10);
    ros::Publisher pub2 = n.advertise<sensor_msgs::PointCloud2>("wall", 1000);
    ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("object", 1000);
    ros::Publisher pub_ang = n.advertise<std_msgs::Float64>("angle", 10);
    ros::Subscriber sub = n.subscribe("/camera/depth/color/points", 10, callback);
    while (ros::ok()) {
        ros::spinOnce();
        if (cluster_indices.size() > 0) {
            pub.publish(data);
            rate.sleep();
            pub2.publish(data2);
            rate.sleep();
            pub_ang.publish(val);
            rate.sleep();
        }
    }
    return 0;
}

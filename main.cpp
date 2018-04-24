#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <Eigen/Geometry>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#define foreach BOOST_FOREACH

int main ()
{

rosbag::Bag bag;
bag.open("./sync_filtered_test.bag", rosbag::bagmode::Read);

std::string imu_topic = "/imu0/data"; //imu topic name
std::string scan_topic = "/scan_corrected"; // scan topic name

std::vector<std::string> topics;
topics.push_back(imu_topic);
topics.push_back(scan_topic);

rosbag::View view(bag, rosbag::TopicQuery(topics));


pcl::PointCloud<pcl::PointXYZ>::Ptr Pcl_ptr (new pcl::PointCloud<pcl::PointXYZ>);
Pcl_ptr->height = 1;
Pcl_ptr->width = 1521;

int inc;
//pcl::visualization::PCLVisualizer* viewer;
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
viewer->setBackgroundColor (0, 0, 0);
viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
viewer->addCoordinateSystem (1.0);
viewer->initCameraParameters ();

viewer->setCameraPosition(7.3, 13.6, 14.4,
                          0.21, -.8, 0.57,
                          0, 0, 1);
viewer->setCameraClipDistances(0.10, 360);

bool not_initialized(true);
Eigen::Quaternionf curr_ori = Eigen::Quaternionf::Identity();

float f = 20; // laserscan update frequency [Hz]

foreach(rosbag::MessageInstance const m, view)
{
    sensor_msgs::Imu::ConstPtr imu = m.instantiate<sensor_msgs::Imu>();
    if (imu!=NULL)
        //std::cout << imu->orientation <<std::endl;
        curr_ori = Eigen::Quaternionf(float (imu->orientation.w),float (imu->orientation.x),float (imu->orientation.y), float (imu->orientation.z));

    sensor_msgs::LaserScan::ConstPtr laser = m.instantiate<sensor_msgs::LaserScan>();
    if (laser!=NULL){
        inc = 0;
        Pcl_ptr->points.clear(); // clear the previous points
        foreach(float range, laser->ranges)
        {
        Pcl_ptr->points.push_back(pcl::PointXYZ(range * cos(laser->angle_min + inc*laser->angle_increment),
                                    range * sin(laser->angle_min + inc*laser->angle_increment),
                                    0));
        inc++;
        }
        //Pcl_ptr->sensor_orientation_ = Eigen::Quaternionf::Identity ();
        Pcl_ptr->sensor_orientation_ = curr_ori;
        if (not_initialized){
            viewer->addPointCloud<pcl::PointXYZ> (Pcl_ptr, "sample cloud");
            not_initialized = false;
        } else {
            viewer->updatePointCloud(Pcl_ptr, "sample cloud");
        }
        viewer->spinOnce(int (round(1/f*1000))); // in ms
        }
}
return(0);
}

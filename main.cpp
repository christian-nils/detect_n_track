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
    // Initialize a bag variable
    rosbag::Bag bag;
    // Open the bag
    bag.open("/home/christian-nils/CN/Program/python/test3/2018-03-21-12-48-27_0.bag", rosbag::bagmode::Read);

    // Declare the topics to read from
    std::string imu_topic = "/imu/data"; //imu topic name
    std::string scan_topic = "/scan"; // scan topic name

    // Initialize the topics time to resynchronize the data
    double imu_time;
    double scan_time;

    // Create one rosbag view for each topic, this will be helpful to resynchronize the data
    rosbag::View imu_view(bag, rosbag::TopicQuery(imu_topic));
    rosbag::View scan_view(bag, rosbag::TopicQuery(scan_topic));

    // Declare the iterator of the imu_view
    rosbag::View::iterator imu_iterator = imu_view.begin();

    // Declare the different pointcloud variables (1 pcl for each laserscan data, 1 pcl representing the corrected pcl, 1 stacked pcl)
    pcl::PointCloud<pcl::PointXYZ>::Ptr Pcl_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    Pcl_ptr->height = 1;
    Pcl_ptr->width = 1521;
    pcl::PointCloud<pcl::PointXYZ>::Ptr Pcl_rotated_ptr (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr Pcl_stacked_ptr (new pcl::PointCloud<pcl::PointXYZ> ());

    // Declare inlier variable use to remove the points outside the road
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    // Declare the inlier variable that is used to remove the ground lines
    pcl::PointIndices::Ptr ground_inliers (new pcl::PointIndices);

    // Declare the extraction method
    pcl::ExtractIndices<pcl::PointXYZ> extract;


    // Declare the model coefficients to read the line parameters
    pcl::ModelCoefficients::Ptr line_coefficients (new pcl::ModelCoefficients);

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_LINE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setOptimizeCoefficients(true);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (1);

    // declaration of couple of other variables
    int inc;
    int curr_ind;
    float ground_line_angle;
    bool not_initialized(true);
    float f = 20; // laserscan update frequency [Hz]

    // Declare the PCL viewer to have a visualization
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    viewer->setCameraPosition(7.3, 13.6, 14.4, 0.21, -.8, 0.57, 0, 0, 1);
    viewer->setCameraClipDistances(0.10, 360);

    // Declare quaternions used to correct the rotations
    Eigen::Quaternionf curr_ori = Eigen::Quaternionf::Identity();
    Eigen::Quaternionf imu2laser = Eigen::Quaternionf(0.0474822869477, 0.033214825533, 0.998319382996, -0.000785700601824); // quaternion from imu 2 laserscan frame (given by Alex/Gabriele)
    Eigen::Quaternionf rectifyYaw = Eigen::Quaternionf(Eigen::AngleAxisf(-49./180 * M_PI, Eigen::Vector3f::UnitZ())); // Quaternion to rectify the global orientation of the road

    /* this main foreach loop is checking the closest IMU message for each given laser scan message
    It will look for the message that is within 0.005s away from the Scan message*/
    foreach(rosbag::MessageInstance const m, scan_view)
    {
        if (m.getTime()>scan_view.getBeginTime()+ros::Duration(400, 0)) // to delay the start Duration(s, ns)
        {
            // instantiate the laser scan message
            sensor_msgs::LaserScan::ConstPtr laser = m.instantiate<sensor_msgs::LaserScan>();
            // assign the scan_time
            scan_time = laser->header.stamp.toSec();
            // look for the closest orientation from the IMU message
            for (; imu_iterator != imu_view.end(); imu_iterator++)
            {
                sensor_msgs::Imu::ConstPtr imu = imu_iterator->instantiate<sensor_msgs::Imu>();

                imu_time = imu->header.stamp.toSec();
                if (fabs(imu_time-scan_time)<=0.005)
                {
                    // store the orientation from this IMU message, this will used to rectify the point cloud (below)
                    curr_ori = Eigen::Quaternionf(float (imu->orientation.w),float (imu->orientation.x),float (imu->orientation.y), float (imu->orientation.z));
                    break;
                }

            }


            // reset the increment variable used for populating the PCL variable
            inc = 0;
             // clear the previous points
            Pcl_ptr->points.clear();
            foreach(float range, laser->ranges)
            {
                if (range != 120)
                {
                    Pcl_ptr->points.push_back(pcl::PointXYZ(range * cos(laser->angle_min + inc*laser->angle_increment),
                                                            range * sin(laser->angle_min + inc*laser->angle_increment),
                                                            0));
                }
                inc++;
            }
            Pcl_ptr->width = Pcl_ptr->points.size();
            // Rectify the orientation of the pointcloud to have the longitudinal orientation along x, and the lateral orientation along y
            pcl::transformPointCloud(*Pcl_ptr, *Pcl_rotated_ptr, Eigen::Vector3f(0,0,.1004), rectifyYaw * curr_ori * imu2laser);
            /* Removed the that are outside the road (points with lateral distance <0 and >8) and points that are below a -0.4 z
            that correspond to point belonging to the road */
            // Reset the current point indice to 0
            curr_ind = 0;
            // Clear the inliers variable
            inliers->indices.clear();
            // Check each point from the PCL to see if it is inside the boundaries or not
            foreach(pcl::PointXYZ point, Pcl_rotated_ptr->points)
            {
                if (point.y < -8 || point.y > 0 || point.z < -.4)
                {
                    inliers->indices.push_back(curr_ind);
                }

                curr_ind++;
            }
            // Remove the points from the pcl
            extract.setIndices(inliers);
            extract.setNegative(true);
            extract.filterDirectly(Pcl_rotated_ptr);

            /* FIXME: the line detection algorithm, below, has to be fixed. It may be that there is more than 1 line present
            in the current pcl and therefore a while loop would be necessary here */

            /*   Remove the ground line, all detected line with an angle between 15 and 75 degrees are removed from the pcl */
            seg.setInputCloud(Pcl_rotated_ptr);
            seg.segment(*ground_inliers, *line_coefficients);
            // calculate the angle from the y axis, the line with an angle comprises between 15 and 75 is removed from the pcl
            ground_line_angle = fabs(atan(line_coefficients->values[4]/line_coefficients->values[3])*180/M_PI);
            if ( 15 < ground_line_angle && ground_line_angle < 75 )
            {
                extract.setIndices(ground_inliers);
                extract.setNegative(true);
                extract.filterDirectly(Pcl_rotated_ptr);

            }

            /*  appending the previous transformed pcl only if the number of points did not reach 10000 */
            if (Pcl_stacked_ptr->points.size()>100000)
            {
                Pcl_stacked_ptr->points.clear();
            }
            foreach (pcl::PointXYZ pts, Pcl_rotated_ptr->points)
            {
                Pcl_stacked_ptr->points.push_back(pts);
            }


            if (not_initialized)
            {
                viewer->addPointCloud<pcl::PointXYZ> (Pcl_stacked_ptr, "sample cloud");
                not_initialized = false;
            }
            else
            {
                viewer->updatePointCloud(Pcl_stacked_ptr, "sample cloud");
            }

            viewer->spinOnce(int (round(1/f*1000))); // in ms

        }
    }
    return(0);
}

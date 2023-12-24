#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/imgcodecs/legacy/constants_c.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/foreach.hpp>

std::string src_bag = "/home/ubuntu/project/small.bag";
std::string new_bag = "/home/ubuntu/project/imu_pcd_image.bag";
std::string imu_topic = "/imu/data_raw";
std::string pcd2_topic = "/rslidar_points";
std::string odom_topic = "/odom";
std::string image_topic = "/camera/color/image_raw";
std::string image_dp_topic = "/camera/depth/image_rect_raw";

int main(int argc, char **argv)
{
    rosbag::Bag i_bag, o_bag;
    i_bag.open(src_bag, rosbag::bagmode::Read);
    o_bag.open(new_bag, rosbag::bagmode::Write);
    std::vector<std::string> topics;
    topics.push_back(std::string(imu_topic));
    topics.push_back(std::string(pcd2_topic));
    topics.push_back(std::string(odom_topic));
    topics.push_back(std::string(image_topic));
    topics.push_back(std::string(image_dp_topic));
    rosbag::View view(i_bag, rosbag::TopicQuery(topics));

    //  pcl::visualization::CloudViewer viewer("Point Cloud Viewer");

    for (const rosbag::MessageInstance &m : view)
    {
        if (m.getTopic() == image_topic)
        {

            sensor_msgs::Image::ConstPtr image = m.instantiate<sensor_msgs::Image>();
            if (image != nullptr)
            {
                /* std::cout << "image stamp:" << image->header.stamp << std::endl;

                // Convert ROS Image message to OpenCV Mat
                cv::Mat img;
                try
                {
                    cv_bridge::CvImageConstPtr cv_ptr;
                    cv_ptr = cv_bridge::toCvShare(image, "bgr8");
                    img = cv_ptr->image;
                }
                catch (cv_bridge::Exception &e)
                {
                    ROS_ERROR("cv_bridge exception: %s", e.what());
                    return 1;
                }

                // Display the image using OpenCV
                cv::imshow("Camera Image", img);
                cv::waitKey(1); */

                // sensor_msgs::Image::ConstPtr image = m.instantiate<sensor_msgs::Image>();
                if (image != nullptr)
                {
                    // Convert ROS Image message to OpenCV Mat
                    cv::Mat img;
                    try
                    {
                        cv_bridge::CvImageConstPtr cv_ptr;
                        cv_ptr = cv_bridge::toCvShare(image, "bgr8");
                        img = cv_ptr->image;
                    }
                    catch (cv_bridge::Exception &e)
                    {
                        ROS_ERROR("cv_bridge exception: %s", e.what());
                        return 1;
                    }

                    // 使用k均值聚类对彩色图像进行分割
                    cv::Mat reshaped_image = img.reshape(1, img.rows * img.cols);
                    cv::Mat reshaped_image32f;
                    reshaped_image.convertTo(reshaped_image32f, CV_32F);
                    int K = 5; // 设置聚类数
                    cv::Mat best_labels, centers;
                    cv::kmeans(reshaped_image32f, K, best_labels, cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 10, 1.0), 3, cv::KMEANS_PP_CENTERS, centers);

                    // 将聚类结果转换为彩色图像
                    cv::Mat segmented_image(img.size(), img.type());
                    for (int i = 0; i < img.rows; ++i)
                    {
                        for (int j = 0; j < img.cols; ++j)
                        {
                            int cluster_idx = best_labels.at<int>(i + j * img.rows, 0);
                            segmented_image.at<cv::Vec3b>(i, j) = centers.at<cv::Vec3f>(cluster_idx, 0);
                        }
                    }

                    // 显示分割结果
                    cv::imshow("Segmented Image", segmented_image);
                    cv::waitKey(1);

                    // 将分割结果写入到新的bag文件
                    // sensor_msgs::ImagePtr segmented_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", segmented_image).toImageMsg();
                    // o_bag.write("/segmented_image", image->header.stamp, segmented_msg);
                }

                // Write the image message to the new bag file
                // o_bag.write(image_topic, image->header.stamp, image);
            }
        }
        /*    if (m.getTopic() == image_dp_topic)
           {
               // Process Depth Image messages
               sensor_msgs::Image::ConstPtr depth_image = m.instantiate<sensor_msgs::Image>();
               if (depth_image != nullptr)
               {
                   std::cout << "Depth image stamp:" << depth_image->header.stamp << std::endl;

                   // Convert ROS Image message to OpenCV Mat
                   cv::Mat depth_img;
                   try
                   {
                       cv_bridge::CvImageConstPtr cv_ptr;
                       cv_ptr = cv_bridge::toCvShare(depth_image, "32FC1");
                       depth_img = cv_ptr->image;
                   }
                   catch (cv_bridge::Exception &e)
                   {
                       ROS_ERROR("cv_bridge exception: %s", e.what());
                       return 1;
                   }

                   // Display the depth image using OpenCV
                   cv::imshow("Depth Image", depth_img);
                   cv::waitKey(1);

                   // Write the depth image message to the new bag file
                   //  o_bag.write(depth_image_topic, depth_image->header.stamp, depth_image);
               }
           } */
        /*         if (m.getTopic() == pcd2_topic)
                {
                    // Process PointCloud2 messages
                    sensor_msgs::PointCloud2::ConstPtr cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
                    if (cloud_msg != nullptr)
                    {
                        std::cout << "Point Cloud stamp:" << cloud_msg->header.stamp << std::endl;

                        // Convert ROS PointCloud2 message to PCL PointCloud
                        pcl::PCLPointCloud2 pcl_pc2;
                        pcl_conversions::toPCL(*cloud_msg, pcl_pc2);
                        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
                        pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

                        // Display the point cloud using PCL viewer
                        viewer.showCloud(cloud);
                    }
                } */
    }
    i_bag.close();
    o_bag.close();
    return 0;
}

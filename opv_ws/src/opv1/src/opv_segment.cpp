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

std::string src_bag = "/home/ubuntu/project/small_a.bag";
std::string new_bag = "/home/ubuntu/project/imu_pcd_image.bag";
std::string imu_topic = "/imu/data_raw";
std::string pcd2_topic = "/rslidar_points";
std::string odom_topic = "/odom";
std::string image_topic = "/camera/color/image_raw";
std::string image_dp_topic = "/camera/depth/image_rect_raw";

// 函数：使用 GrabCut 算法进行图像分割
cv::Mat performGrabCut(const cv::Mat &inputImage)
{
    cv::Mat result;
    cv::Mat mask(inputImage.size(), CV_8UC1, cv::Scalar(cv::GC_PR_BGD)); // 创建掩码

    // 定义一个矩形区域，该区域包含前景目标
    cv::Rect rectangle(50, 50, 200, 300);

    // 使用 GrabCut 算法进行图像分割
    cv::Mat bgModel, fgModel; // 创建背景模型和前景模型
    cv::grabCut(inputImage, mask, rectangle, bgModel, fgModel, 5, cv::GC_INIT_WITH_RECT);

    // 将掩码中的前景和可能的前景区域设置为前景，其他区域设置为背景
    mask = (mask == cv::GC_PR_FGD) | (mask == cv::GC_FGD);

    // 创建一个输出图像，将前景提取出来
    inputImage.copyTo(result, mask);

    return result;
}

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
    setlocale(LC_ALL, ""); // 设置为系统默认的本地化环境
                           //  pcl::visualization::CloudViewer viewer("Point Cloud Viewer");

    for (const rosbag::MessageInstance &m : view)
    {
        if (m.getTopic() == image_topic)
        {

            sensor_msgs::Image::ConstPtr image = m.instantiate<sensor_msgs::Image>();
        /*     if (image != nullptr)
            { */
                 std::cout << "image stamp:" << image->header.stamp << std::endl;

                // sensor_msgs::Image::ConstPtr image = m.instantiate<sensor_msgs::Image>();
                 if (image != nullptr)
                 {
                     // 将ROS Image消息转换为OpenCV Mat
                     cv::Mat img;
                     try
                     {
                         cv_bridge::CvImageConstPtr cv_ptr;
                         cv_ptr = cv_bridge::toCvShare(image, "bgr8");
                         img = cv_ptr->image;
                     }
                     catch (cv_bridge::Exception &e)
                     {
                         ROS_ERROR("cv_bridge异常: %s", e.what());
                         return 1;
                     }

                     // 使用OpenCV显示原始图像
                     cv::imshow("org image", img);
                     cv::waitKey(1);

                     // 使用 GrabCut 算法进行图像分割
                     cv::Mat segmented_image = performGrabCut(img);

                     // 显示分割结果
                     cv::imshow("Segmented Image", segmented_image);
                     cv::waitKey(1);

                     // 将分割结果写入到新的bag文件
                     // sensor_msgs::ImagePtr segmented_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", segmented_image).toImageMsg();
                     // o_bag.write("/segmented_image", image->header.stamp, segmented_msg);
                     /* } */

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

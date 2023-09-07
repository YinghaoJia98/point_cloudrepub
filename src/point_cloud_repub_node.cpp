#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>

ros::Publisher PointCloudPub_;
std::vector<sensor_msgs::PointCloudConstPtr> VectorOfInputPointCloud_;
std::string InputTopic_, OutputTopic_, OutFrame_;

void PointCloudCallback(const sensor_msgs::PointCloudConstPtr &InputPointCloud)
{
    sensor_msgs::PointCloud2 PointCloudToOutputTem_;
    sensor_msgs::convertPointCloudToPointCloud2(*InputPointCloud, PointCloudToOutputTem_);
    PointCloudToOutputTem_.header.stamp = InputPointCloud->header.stamp;
    PointCloudToOutputTem_.header.frame_id = OutFrame_.c_str();
    PointCloudPub_.publish(PointCloudToOutputTem_);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_cloud_repub_node");
    ros::NodeHandle nh_;

    ROS_INFO("start point_cloud_repub");
    nh_.param<std::string>("/point_cloud_repub/point_cloud_repub_settings/InputTopic",
                           InputTopic_, "/scan");
    nh_.param<std::string>("/point_cloud_repub/point_cloud_repub_settings/OutputTopic",
                           OutputTopic_, "/livox_mid360_points");
    nh_.param<std::string>("/point_cloud_repub/point_cloud_repub_settings/OutFrame",
                           OutFrame_, "livox_mid360");

    ros::Subscriber PointCloudSub_ = nh_.subscribe<sensor_msgs::PointCloud>(
        InputTopic_.c_str(), 100, PointCloudCallback);
    PointCloudPub_ = nh_.advertise<sensor_msgs::PointCloud2>(OutputTopic_.c_str(), 100);

    ros::spin();
}

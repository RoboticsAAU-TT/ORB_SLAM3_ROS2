#ifndef __MONOCULAR_SLAM_NODE_HPP__
#define __MONOCULAR_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"

using ImuMsg = sensor_msgs::msg::Imu;
using ImageMsg = sensor_msgs::msg::Image;

class MonocularInertialSlamNode : public rclcpp::Node
{
public:
    MonocularInertialSlamNode(ORB_SLAM3::System* pSLAM);

    ~MonocularInertialSlamNode();

private:

    void GrabImu(const ImuMsg::SharedPtr msg);
    void GrabImage(const ImageMsg::SharedPtr msg);
    cv::Mat GetImage(const ImageMsg::SharedPtr msg);
    void SyncWithImu();

    ORB_SLAM3::System* m_SLAM;
    std::thread *syncThread_;

    // IMU
    rclcpp::Subscription<ImuMsg>::SharedPtr m_imu_subscriber;
    queue<ImuMsg::SharedPtr> imuBuf_;
    std::mutex imuBufMutex_;

    // Image
    rclcpp::Subscription<ImageMsg>::SharedPtr m_image_subscriber;
    queue<ImageMsg::SharedPtr> imgBuf_;
    std::mutex imgBufMutex_;

};

#endif

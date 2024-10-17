#include "monocular-inertial-slam-node.hpp"

#include<opencv2/core/core.hpp>

using std::placeholders::_1;

MonocularInertialSlamNode::MonocularInertialSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2")
{
    m_SLAM = pSLAM;
    // std::cout << "slam changed" << std::endl;
    m_imu_subscriber = this->create_subscription<ImuMsg>("imu", 1000, std::bind(&MonocularInertialSlamNode::GrabImu, this, _1));
    m_image_subscriber = this->create_subscription<ImageMsg>(
        "camera",
        100,
        std::bind(&MonocularInertialSlamNode::GrabImage, this, std::placeholders::_1));
    std::cout << "slam changed" << std::endl;

    syncThread_ = new std::thread(&MonocularInertialSlamNode::SyncWithImu, this);
}

MonocularInertialSlamNode::~MonocularInertialSlamNode()
{

    // Delete sync thread
    syncThread_->join();
    delete syncThread_;

    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    // m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

cv::Mat MonocularInertialSlamNode::GetImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }

    if (cv_ptr->image.type() == 0)
    {
        return cv_ptr->image.clone();
    }
    else
    {
        std::cerr << "Error image type" << std::endl;
        return cv_ptr->image.clone();
    }
}

void MonocularInertialSlamNode::GrabImu(const ImuMsg::SharedPtr msg)
{
    imuBufMutex_.lock();
    imuBuf_.push(msg);
    imuBufMutex_.unlock();
}

void MonocularInertialSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    imgBufMutex_.lock();

    if (!empty(imgBuf_))
        imgBuf_.pop();
    imgBuf_.push(msg);

    imgBufMutex_.unlock();
}

void MonocularInertialSlamNode::SyncWithImu()
{
    while (1)
    {
        cv::Mat img;
        double tImg = 0;
        if (!empty(imgBuf_) && !empty(imuBuf_))
        {

            imgBufMutex_.lock();
            tImg = Utility::StampToSec(imgBuf_.front()->header.stamp);
            imgBufMutex_.unlock();

            if (tImg > Utility::StampToSec(imuBuf_.back()->header.stamp)){
                continue;
            }

            imgBufMutex_.lock();
            img = GetImage(imgBuf_.front());
            imgBuf_.pop();
            imgBufMutex_.unlock();

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            imuBufMutex_.lock();
            if (!empty(imuBuf_))
            {
                // Load imu measurements from buffer
                vImuMeas.clear();
                while (!empty(imuBuf_) && Utility::StampToSec(imuBuf_.front()->header.stamp) <= tImg)
                {
                    double t = Utility::StampToSec(imuBuf_.front()->header.stamp);
                    cv::Point3f acc(imuBuf_.front()->linear_acceleration.x, imuBuf_.front()->linear_acceleration.y, imuBuf_.front()->linear_acceleration.z);
                    cv::Point3f gyr(imuBuf_.front()->angular_velocity.x, imuBuf_.front()->angular_velocity.y, imuBuf_.front()->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                    imuBuf_.pop();
                }
            }
            imuBufMutex_.unlock();

            m_SLAM->TrackMonocular(img, tImg, vImuMeas);

            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
        }
    }
}

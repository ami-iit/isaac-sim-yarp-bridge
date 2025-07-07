// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-2-Clause
#include "IsaacSimRGBDSensorNWCROS2.h"
#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <sensor_msgs/image_encodings.hpp>

YARP_DECLARE_LOG_COMPONENT(RGBD)
YARP_LOG_COMPONENT(RGBD, "yarp.device.IsaacSimRGBDSensorNWCROS2")


bool yarp::dev::IsaacSimRGBDSensorNWCROS2::open(yarp::os::Searchable& config)
{
    m_errorHandler.setPrefix("[open] ");
    if (!m_paramsParser.parseParams(config))
    {
        m_errorHandler << "Failed to parse parameters for IsaacSimRGBDSensorNWCROS2";
        return false;
    }

    rclcpp::init(0, nullptr);
    m_subscriber = std::make_shared<RGBDSubscriber>(m_paramsParser.m_node_name, m_paramsParser.m_rgb_topic_name, m_paramsParser.m_depth_topic_name, this);
    rclcpp::spin(m_subscriber);

    return true;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::close()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_receivedOnce = false;
    if (m_subscriber)
    {
        m_subscriber.reset();
    }
    rclcpp::shutdown();
    return true;
}

int yarp::dev::IsaacSimRGBDSensorNWCROS2::getRgbHeight()
{
    return 0;
}

int yarp::dev::IsaacSimRGBDSensorNWCROS2::getRgbWidth()
{
    return 0;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::getRgbSupportedConfigurations(yarp::sig::VectorOf<yarp::dev::CameraConfig>& configurations)
{
    return false;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::getRgbResolution(int& width, int& height)
{
    return false;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::setRgbResolution(int width, int height)
{
    return false;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::getRgbFOV(double& horizontalFov, double& verticalFov)
{
    return false;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::setRgbFOV(double horizontalFov, double verticalFov)
{
    return false;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::getRgbMirroring(bool& mirror)
{
    return false;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::setRgbMirroring(bool mirror)
{
    return false;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::getRgbIntrinsicParam(yarp::os::Property& intrinsic)
{
    return false;
}

int yarp::dev::IsaacSimRGBDSensorNWCROS2::getDepthHeight()
{
    return 0;
}

int yarp::dev::IsaacSimRGBDSensorNWCROS2::getDepthWidth()
{
    return 0;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::setDepthResolution(int width, int height)
{
    return false;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::getDepthFOV(double& horizontalFov, double& verticalFov)
{
    return false;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::setDepthFOV(double horizontalFov, double verticalFov)
{
    return false;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::getDepthIntrinsicParam(yarp::os::Property& intrinsic)
{
    return false;
}

double yarp::dev::IsaacSimRGBDSensorNWCROS2::getDepthAccuracy()
{
    return 0.0;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::setDepthAccuracy(double accuracy)
{
    return false;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::getDepthClipPlanes(double& nearPlane, double& farPlane)
{
    return false;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::setDepthClipPlanes(double nearPlane, double farPlane)
{
    return false;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::getDepthMirroring(bool& mirror)
{
    return false;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::setDepthMirroring(bool mirror)
{
    return false;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::getExtrinsicParam(yarp::sig::Matrix& extrinsic)
{
    return false;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::getRgbImage(yarp::sig::FlexImage& rgbImage, yarp::os::Stamp* timeStamp)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_errorHandler.setPrefix("[getRgbImage] ");
    if (!m_receivedOnce)
    {
        m_errorHandler << "No images received yet. Please ensure the ROS2 topics are publishing data.";
        return false;
    }

    if (timeStamp != nullptr)
    {
        *timeStamp = m_rgbTimestamp;
    }

    rgbImage = m_rgbImage;

    return true;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::getDepthImage(yarp::sig::ImageOf<yarp::sig::PixelFloat>& depthImage, yarp::os::Stamp* timeStamp)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_errorHandler.setPrefix("[getDepthImage] ");
    if (!m_receivedOnce)
    {
        m_errorHandler << "No images received yet. Please ensure the ROS2 topics are publishing data.";
        return false;
    }

    if (timeStamp != nullptr)
    {
        *timeStamp = m_depthTimestamp;
    }

    depthImage = m_depthImage;

    return true;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::getImages(yarp::sig::FlexImage& colorFrame, yarp::sig::ImageOf<yarp::sig::PixelFloat>& depthFrame, yarp::os::Stamp* colorStamp, yarp::os::Stamp* depthStamp)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_errorHandler.setPrefix("[getImages] ");
    if (!m_receivedOnce)
    {
        m_errorHandler << "No images received yet. Please ensure the ROS2 topics are publishing data.";
        return false;
    }

    if (colorStamp != nullptr)
    {
        *colorStamp = m_rgbTimestamp;
    }
    if (depthStamp != nullptr)
    {
        *depthStamp = m_depthTimestamp;
    }

    colorFrame = m_rgbImage;
    depthFrame = m_depthImage;

    return true;
}

yarp::dev::IRGBDSensor::RGBDSensor_status yarp::dev::IsaacSimRGBDSensorNWCROS2::getSensorStatus()
{
    if (!m_receivedOnce)
    {
        return RGBDSensor_status::RGBD_SENSOR_NOT_READY;
    }
    return RGBDSensor_status::RGBD_SENSOR_OK_IN_USE;
}

std::string yarp::dev::IsaacSimRGBDSensorNWCROS2::getLastErrorMsg(yarp::os::Stamp* timeStamp)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_errorHandler.getLastErrorMsg();
}

void yarp::dev::IsaacSimRGBDSensorNWCROS2::updateImages(const sensor_msgs::msg::Image::ConstSharedPtr& rgb, const sensor_msgs::msg::Image::ConstSharedPtr& depth)
{
    // The code of these conversions have been ispired from
    // https://github.com/robotology/yarp-devices-ros2/blob/e1b9c86aa91c0fb3a14c6d2415e75c3868e222dc/src/devices/ros2RGBDConversionUtils/Ros2RGBDConversionUtils.cpp

    std::lock_guard<std::mutex> lock(m_mutex);

    const auto& rosPixelType = rgb->encoding;
    int yarpPixelType = VOCAB_PIXEL_INVALID;

    if (rosPixelType == sensor_msgs::image_encodings::RGB8)
    {
        yarpPixelType = VOCAB_PIXEL_RGB;
    }
    else if (rosPixelType == sensor_msgs::image_encodings::BGR8)
    {
        yarpPixelType = VOCAB_PIXEL_BGR;
    }
    else
    {
        yCError(RGBD) << "Unsupported RGB pixel type:" << rosPixelType;
        return;
    }
    m_rgbImage.setQuantum(0);
    m_rgbImage.setPixelCode(yarpPixelType);
    m_rgbImage.setPixelSize(rgb->step / rgb->width); // The step is a full row length in bytes
    m_rgbImage.resize(rgb->width, rgb->height);
    size_t c = 0;
    unsigned char* rgbData = m_rgbImage.getRawImage();
    for (auto it = rgb->data.begin(); it != rgb->data.end(); it++)
    {
        rgbData[c++] = *it;
    }
    m_rgbTimestamp.update(rgb->header.stamp.sec + (rgb->header.stamp.nanosec / 1e9));

    if (depth->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
    {
        m_depthImage.resize(depth->width, depth->height);
        size_t c = 0;
        uint16_t* p = (uint16_t*)(depth->data.data());
        uint16_t* siz = (uint16_t*)(depth->data.data()) + (depth->data.size() / sizeof(uint16_t));
        unsigned char* depthData = m_depthImage.getRawImage();
        int count = 0;
        for (; p < siz; p++)
        {
            float value = static_cast<float>(*p) / 1000.0; // Convert from millimeters to meters, since the input is a 16-bit unsigned integer
            ((float*)(depthData))[c++] = value;
            count++;
        }
    }
    else if (depth->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    {
        m_depthImage.resize(depth->width, depth->height);
        unsigned char* depthData = m_depthImage.getRawImage();
        size_t c = 0;
        for (auto it = depth->data.begin(); it != depth->data.end(); it++)
        {
            depthData[c++] = *it;
        }
    }
    else
    {
        yCError(RGBD) << "Unsupported depth format:" << rosPixelType;
        return;
    }
    m_depthTimestamp.update(depth->header.stamp.sec + (depth->header.stamp.nanosec / 1e9));

    m_receivedOnce = true;
}

yarp::dev::IsaacSimRGBDSensorNWCROS2::RGBDSubscriber::RGBDSubscriber(const std::string& name, const std::string& rgbTopic, const std::string& depthTopic, IsaacSimRGBDSensorNWCROS2* parent)
    : Node(name)
{
    m_parent = parent; // Store the parent device pointer

    // Create message_filters subscribers
    m_rgb_sub.subscribe(this, rgbTopic);
    m_depth_sub.subscribe(this, depthTopic);

    const uint32_t queue_size = 10; // Default queue size for synchronization

    // ApproximateTimeSynchronizer setup
    m_sync = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(queue_size), m_rgb_sub, m_depth_sub);
    m_sync->registerCallback(std::bind(&RGBDSubscriber::callback, this, std::placeholders::_1, std::placeholders::_2));
}

void yarp::dev::IsaacSimRGBDSensorNWCROS2::RGBDSubscriber::callback(const sensor_msgs::msg::Image::ConstSharedPtr& rgb, const sensor_msgs::msg::Image::ConstSharedPtr& depth)
{
    m_parent->updateImages(rgb, depth);
}

void yarp::dev::IsaacSimRGBDSensorNWCROS2::ErrorHandler::setPrefix(const std::string& prefix)
{
    m_prefix = prefix;
    m_lastErrorMsg.clear();
}

void yarp::dev::IsaacSimRGBDSensorNWCROS2::ErrorHandler::operator<<(const std::string& errorMsg)
{
    m_lastErrorMsg = m_prefix + errorMsg;
    m_errorTimestamp.update();
    yCError(RGBD, "%s", m_lastErrorMsg.c_str());
}

void yarp::dev::IsaacSimRGBDSensorNWCROS2::ErrorHandler::operator<<(const std::stringstream& errorMsg)
{
    this->operator<<(errorMsg.str());
}

const std::string& yarp::dev::IsaacSimRGBDSensorNWCROS2::ErrorHandler::getLastErrorMsg() const
{
     return m_lastErrorMsg;
}

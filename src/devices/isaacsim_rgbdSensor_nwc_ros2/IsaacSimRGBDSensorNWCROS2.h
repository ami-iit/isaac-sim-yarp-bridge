// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-2-Clause

#ifndef ISAACSIM_RGBD_SENSOR_NWC_ROS2_H
#define ISAACSIM_RGBD_SENSOR_NWC_ROS2_H

#include <yarp/dev/DeviceDriver.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/IRGBDSensor.h>

#include <rclcpp/node.hpp>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/msg/image.hpp>

#include <memory>
#include <mutex>
#include <atomic>
#include <sstream>

#include "IsaacSimRGBDSensorNWCROS2_ParamsParser.h"

namespace yarp::dev
{
class IsaacSimRGBDSensorNWCROS2;
}


class yarp::dev::IsaacSimRGBDSensorNWCROS2 :
    public yarp::dev::DeviceDriver,
    public yarp::dev::IRGBDSensor
{

public:
    IsaacSimRGBDSensorNWCROS2() = default;
    ~IsaacSimRGBDSensorNWCROS2() override = default;

    // DeviceDriver
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // IRGBDSensor
    int    getRgbHeight() override;
    int    getRgbWidth() override;
    bool   getRgbSupportedConfigurations(yarp::sig::VectorOf<yarp::dev::CameraConfig>& configurations) override;
    bool   getRgbResolution(int& width, int& height) override;
    bool   setRgbResolution(int width, int height) override;
    bool   getRgbFOV(double& horizontalFov, double& verticalFov) override;
    bool   setRgbFOV(double horizontalFov, double verticalFov) override;
    bool   getRgbMirroring(bool& mirror) override;
    bool   setRgbMirroring(bool mirror) override;

    bool   getRgbIntrinsicParam(yarp::os::Property& intrinsic) override;
    int    getDepthHeight() override;
    int    getDepthWidth() override;
    bool   setDepthResolution(int width, int height) override;
    bool   getDepthFOV(double& horizontalFov, double& verticalFov) override;
    bool   setDepthFOV(double horizontalFov, double verticalFov) override;
    bool   getDepthIntrinsicParam(yarp::os::Property& intrinsic) override;
    double getDepthAccuracy() override;
    bool   setDepthAccuracy(double accuracy) override;
    bool   getDepthClipPlanes(double& nearPlane, double& farPlane) override;
    bool   setDepthClipPlanes(double nearPlane, double farPlane) override;
    bool   getDepthMirroring(bool& mirror) override;
    bool   setDepthMirroring(bool mirror) override;


    bool   getExtrinsicParam(yarp::sig::Matrix& extrinsic) override;
    bool   getRgbImage(yarp::sig::FlexImage& rgbImage, yarp::os::Stamp* timeStamp = nullptr) override;
    bool   getDepthImage(yarp::sig::ImageOf<yarp::sig::PixelFloat>& depthImage, yarp::os::Stamp* timeStamp = nullptr) override;
    bool   getImages(yarp::sig::FlexImage& colorFrame, yarp::sig::ImageOf<yarp::sig::PixelFloat>& depthFrame, yarp::os::Stamp* colorStamp = NULL, yarp::os::Stamp* depthStamp = NULL) override;

    RGBDSensor_status  getSensorStatus() override;
    std::string getLastErrorMsg(yarp::os::Stamp* timeStamp = NULL) override;

private:
    class RGBDSubscriber : public rclcpp::Node
    {
    public:
        RGBDSubscriber(const std::string& name, const std::string& rgbTopic, const std::string& depthTopic, IsaacSimRGBDSensorNWCROS2* parent);
    private:
        typedef message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::Image,
            sensor_msgs::msg::Image
        > SyncPolicy;

        void callback(const sensor_msgs::msg::Image::ConstSharedPtr& rgb,
            const sensor_msgs::msg::Image::ConstSharedPtr& depth);

        message_filters::Subscriber<sensor_msgs::msg::Image> m_rgb_sub;
        message_filters::Subscriber<sensor_msgs::msg::Image> m_depth_sub;
        std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> m_sync;
        IsaacSimRGBDSensorNWCROS2* m_parent; // Pointer to the parent device to call updateImages
    };

    class ErrorHandler
    {
        std::string m_lastErrorMsg;
        std::string m_prefix;
        yarp::os::Stamp m_errorTimestamp;
    public:
        void setPrefix(const std::string& prefix);
        void operator << (const std::string& errorMsg);
        void operator << (const std::stringstream& errorMsg);
        const std::string& getLastErrorMsg() const;
    };

    void updateImages(const sensor_msgs::msg::Image::ConstSharedPtr& rgb,
        const sensor_msgs::msg::Image::ConstSharedPtr& depth);

    IsaacSimRGBDSensorNWCROS2_ParamsParser m_paramsParser;
    std::shared_ptr<RGBDSubscriber> m_subscriber;
    std::atomic<bool> m_receivedOnce{ false };
    yarp::os::Stamp m_rgbTimestamp;
    yarp::os::Stamp m_depthTimestamp;
    yarp::sig::FlexImage m_rgbImage;
    yarp::sig::ImageOf<yarp::sig::PixelFloat> m_depthImage;
    ErrorHandler m_errorHandler;
    std::mutex m_mutex;
};
#endif

// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-2-Clause

#ifndef ISAACSIM_RGBD_SENSOR_NWC_ROS2_H
#define ISAACSIM_RGBD_SENSOR_NWC_ROS2_H

#include <yarp/dev/DeviceDriver.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/IRGBDSensor.h>

namespace yarp::dev
{
class IsaacSimRGBDSensorNWCROS2;
}


class yarp::dev::IsaacSimRGBDSensorNWCROS2 :
    public yarp::dev::DeviceDriver,
    public yarp::dev::IRGBDSensor
{

public:
    IsaacSimRGBDSensorNWCROS2();
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
};
#endif

// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-2-Clause
#include "IsaacSimControlBoardNWCROS2.h"
#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>

YARP_DECLARE_LOG_COMPONENT(CB)
YARP_LOG_COMPONENT(CB, "yarp.device.IsaacSimControlBoardNWCROS2")

yarp::dev::IsaacSimControlBoardNWCROS2::~IsaacSimControlBoardNWCROS2()
{
    close();
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::open(yarp::os::Searchable& config)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    std::string errorPrefix = "[open] ";
    if (!m_paramsParser.parseParams(config))
    {
        yCError(CB) << errorPrefix << "Error while parsing configuration parameters.";
        return false;
    }

    if (!rclcpp::ok())
    {
        rclcpp::init(0, nullptr);
    }

    m_node = std::make_shared<CBNode>(m_paramsParser.m_node_name, m_paramsParser.m_joint_state_input_topic_name, this);
    m_executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
    m_executor->add_node(m_node);
    m_executorThread = std::thread([this]() { m_executor->spin(); });

    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::close()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_node)
    {
        m_executor->cancel();
        m_executorThread.join();
        m_node.reset();
    }
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setPid(const yarp::dev::PidControlTypeEnum& pidtype, int j, const yarp::dev::Pid& p)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setPids(const yarp::dev::PidControlTypeEnum& pidtype, const yarp::dev::Pid* ps)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setPidReference(const yarp::dev::PidControlTypeEnum& pidtype, int j, double ref)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setPidReferences(const yarp::dev::PidControlTypeEnum& pidtype, const double* refs)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setPidErrorLimit(const yarp::dev::PidControlTypeEnum& pidtype, int j, double limit)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setPidErrorLimits(const yarp::dev::PidControlTypeEnum& pidtype, const double* limits)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getPidError(const yarp::dev::PidControlTypeEnum& pidtype, int j, double* err)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getPidErrors(const yarp::dev::PidControlTypeEnum& pidtype, double* errs)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getPidOutput(const yarp::dev::PidControlTypeEnum& pidtype, int j, double* out)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getPidOutputs(const yarp::dev::PidControlTypeEnum& pidtype, double* outs)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setPidOffset(const yarp::dev::PidControlTypeEnum& pidtype, int j, double v)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getPid(const yarp::dev::PidControlTypeEnum& pidtype, int j, yarp::dev::Pid* p)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getPids(const yarp::dev::PidControlTypeEnum& pidtype, yarp::dev::Pid* pids)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getPidReference(const yarp::dev::PidControlTypeEnum& pidtype, int j, double* ref)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getPidReferences(const yarp::dev::PidControlTypeEnum& pidtype, double* refs)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getPidErrorLimit(const yarp::dev::PidControlTypeEnum& pidtype, int j, double* limit)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getPidErrorLimits(const yarp::dev::PidControlTypeEnum& pidtype, double* limits)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::resetPid(const yarp::dev::PidControlTypeEnum& pidtype, int j)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::disablePid(const yarp::dev::PidControlTypeEnum& pidtype, int j)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::enablePid(const yarp::dev::PidControlTypeEnum& pidtype, int j)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::isPidEnabled(const yarp::dev::PidControlTypeEnum& pidtype, int j, bool* enabled)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getAxes(int* ax)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getAxes] ";
    if (!m_measurements.valid.load())
    {
        yCError(CB) << errorPrefix << "No valid data received yet.";
        return false;
    }

    std::lock_guard<std::mutex> lock_measurements(m_measurements.mutex);

    *ax = static_cast<int>(m_measurements.name.size());
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::positionMove(int j, double ref)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::positionMove(const double* refs)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::positionMove(const int n_joints, const int* joints, const double* refs)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getTargetPosition(const int joint, double* ref)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getTargetPositions(double* refs)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getTargetPositions(const int n_joint, const int* joints, double* refs)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::relativeMove(int j, double delta)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::relativeMove(const double* deltas)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::relativeMove(const int n_joints, const int* joints, const double* deltas)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::checkMotionDone(int j, bool* flag)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::checkMotionDone(bool* flag)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::checkMotionDone(const int n_joints, const int* joints, bool* flags)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setRefSpeed(int j, double sp)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setRefSpeeds(const double* spds)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setRefSpeeds(const int n_joints, const int* joints, const double* spds)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setRefAcceleration(int j, double acc)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setRefAccelerations(const double* accs)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setRefAccelerations(const int n_joints, const int* joints, const double* accs)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRefSpeed(int j, double* ref)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRefSpeeds(double* spds)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRefSpeeds(const int n_joints, const int* joints, double* spds)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRefAcceleration(int j, double* acc)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRefAccelerations(double* accs)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRefAccelerations(const int n_joints, const int* joints, double* accs)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::stop(int j)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::stop()
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::stop(const int n_joints, const int* joints)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getLastJointFault(int j, int& fault, std::string& message)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::velocityMove(int j, double v)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::velocityMove(const double* v)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::resetEncoder(int j)
{
    yCError(CB) << "[resetEncoder] It is not possible to reset an encoder in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::resetEncoders()
{
    yCError(CB) << "[resetEncoders] It is not possible to reset encoders in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setEncoder(int j, double val)
{
    yCError(CB) << "[setEncoder] It is not possible to set an encoder in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setEncoders(const double* vals)
{
    yCError(CB) << "[setEncoders] It is not possible to set encoders in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getEncoder(int j, double* v)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getEncoder] ";
    if (!m_measurements.valid.load())
    {
        yCError(CB) << errorPrefix << "No valid data received yet.";
        return false;
    }

    std::lock_guard<std::mutex> lock_measurements(m_measurements.mutex);
    if (j < 0 || j >= static_cast<int>(m_measurements.position.size()))
    {
        yCError(CB) << errorPrefix << "Index" << j << "out of range. Valid range is [0," << m_measurements.position.size() - 1 << "]";
        return false;
    }
    *v = m_measurements.position[j];
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getEncoders(double* encs)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getEncoders] ";
    if (!m_measurements.valid.load())
    {
        yCError(CB) << errorPrefix << "No valid data received yet.";
        return false;
    }

    std::lock_guard<std::mutex> lock_measurements(m_measurements.mutex);
    std::copy(m_measurements.position.begin(), m_measurements.position.end(), encs);
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getEncodersTimed(double* encs, double* t)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getEncodersTimed] ";
    if (!m_measurements.valid.load())
    {
        yCError(CB) << errorPrefix << "No valid data received yet.";
        return false;
    }

    std::lock_guard<std::mutex> lock_measurements(m_measurements.mutex);

    std::copy(m_measurements.position.begin(), m_measurements.position.end(), encs);

    // Copy in timestamp a vector of size equal to the number of joints and equal to the timestamp of the measurement
    std::fill(t, t + m_measurements.position.size(), m_measurements.timestamp);

    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getEncoderTimed(int j, double* v, double* t)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getEncoderTimed] ";
    if (!m_measurements.valid.load())
    {
        yCError(CB) << errorPrefix << "No valid data received yet.";
        return false;
    }

    std::lock_guard<std::mutex> lock_measurements(m_measurements.mutex);

    if (j < 0 || j >= static_cast<int>(m_measurements.position.size()))
    {
        yCError(CB) << errorPrefix << "Index" << j << "out of range. Valid range is [0," << m_measurements.position.size() - 1 << "]";
        return false;
    }
    *v = m_measurements.position[j];
    *t = m_measurements.timestamp;
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getEncoderSpeed(int j, double* sp)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getEncoderSpeed] ";
    if (!m_measurements.valid.load())
    {
        yCError(CB) << errorPrefix << "No valid data received yet.";
        return false;
    }

    std::lock_guard<std::mutex> lock_measurements(m_measurements.mutex);

    if (j < 0 || j >= static_cast<int>(m_measurements.velocity.size()))
    {
        yCError(CB) << errorPrefix << "Index" << j << "out of range. Valid range is [0," << m_measurements.velocity.size() - 1 << "]";
        return false;
    }
    *sp = m_measurements.velocity[j];
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getEncoderSpeeds(double* spds)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getEncoderSpeeds] ";
    if (!m_measurements.valid.load())
    {
        yCError(CB) << errorPrefix << "No valid data received yet.";
        return false;
    }

    std::lock_guard<std::mutex> lock_measurements(m_measurements.mutex);

    std::copy(m_measurements.velocity.begin(), m_measurements.velocity.end(), spds);
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getEncoderAcceleration(int j, double* acc)
{
    yCError(CB) << "[getEncoderAcceleration] It is not possible to get encoder acceleration in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getEncoderAccelerations(double* accs)
{
    yCError(CB) << "[getEncoderAccelerations] It is not possible to get encoder accelerations in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getNumberOfMotorEncoders(int* num)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::resetMotorEncoder(int m)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::resetMotorEncoders()
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setMotorEncoderCountsPerRevolution(int m, const double cpr)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getMotorEncoderCountsPerRevolution(int m, double* cpr)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setMotorEncoder(int m, const double val)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setMotorEncoders(const double* vals)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getMotorEncoder(int m, double* v)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getMotorEncoders(double* encs)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getMotorEncodersTimed(double* encs, double* t)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getMotorEncoderTimed(int m, double* v, double* t)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getMotorEncoderSpeed(int m, double* sp)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getMotorEncoderSpeeds(double* spds)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getMotorEncoderAcceleration(int m, double* acc)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getMotorEncoderAccelerations(double* accs)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::enableAmp(int j)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::disableAmp(int j)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getAmpStatus(int* st)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getAmpStatus(int j, int* v)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setMaxCurrent(int j, double v)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getMaxCurrent(int j, double* v)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getNominalCurrent(int m, double* val)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setNominalCurrent(int m, const double val)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getPeakCurrent(int m, double* val)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setPeakCurrent(int m, const double val)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getPWM(int m, double* val)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getPWMLimit(int m, double* val)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setPWMLimit(int m, const double val)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getPowerSupplyVoltage(int m, double* val)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setLimits(int j, double min, double max)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getLimits(int j, double* min, double* max)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setVelLimits(int j, double min, double max)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getVelLimits(int j, double* min, double* max)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRemoteVariable(std::string key, yarp::os::Bottle& val)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setRemoteVariable(std::string key, const yarp::os::Bottle& val)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRemoteVariablesList(yarp::os::Bottle* listOfKeys)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::isCalibratorDevicePresent(bool* isCalib)
{
    return false;
}

yarp::dev::IRemoteCalibrator* yarp::dev::IsaacSimControlBoardNWCROS2::getCalibratorDevice()
{
    return nullptr;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::calibrateSingleJoint(int j)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::calibrateWholePart()
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::homingSingleJoint(int j)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::homingWholePart()
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::parkSingleJoint(int j, bool _wait)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::parkWholePart()
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::quitCalibrate()
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::quitPark()
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::calibrateAxisWithParams(int j, unsigned int ui, double v1, double v2, double v3)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setCalibrationParameters(int j, const yarp::dev::CalibrationParameters& params)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::calibrationDone(int j)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::abortPark()
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::abortCalibration()
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getNumberOfMotors(int* num)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getTemperature(int m, double* val)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getTemperatures(double* vals)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getTemperatureLimit(int m, double* val)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setTemperatureLimit(int m, const double val)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getGearboxRatio(int m, double* val)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setGearboxRatio(int m, const double val)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getAxisName(int j, std::string& name)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getAxisName] ";
    if (!m_measurements.valid.load())
    {
        yCError(CB) << errorPrefix << "No valid data received yet.";
        return false;
    }

    std::lock_guard<std::mutex> lock_measurements(m_measurements.mutex);
    if (j < 0 || j >= static_cast<int>(m_measurements.name.size()))
    {
        yCError(CB) << errorPrefix << "Index" << j << "out of range. Valid range is [0," << m_measurements.name.size() - 1 << "]";
        return false;
    }
    name = m_measurements.name[j];
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getJointType(int j, yarp::dev::JointTypeEnum& type)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRefTorques(double* refs)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRefTorque(int j, double* t)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setRefTorques(const double* t)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setRefTorque(int j, double t)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setRefTorques(const int n_joint, const int* joints, const double* t)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getMotorTorqueParams(int j, yarp::dev::MotorTorqueParameters* params)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setMotorTorqueParams(int j, const yarp::dev::MotorTorqueParameters params)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setImpedance(int j, double stiff, double damp)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setImpedanceOffset(int j, double offset)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getTorque(int j, double* t)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getTorque] ";
    if (!m_measurements.valid.load())
    {
        yCError(CB) << errorPrefix << "No valid data received yet.";
        return false;
    }
    std::lock_guard<std::mutex> lock_measurements(m_measurements.mutex);
    if (j < 0 || j >= static_cast<int>(m_measurements.effort.size()))
    {
        yCError(CB) << errorPrefix << "Index" << j << "out of range. Valid range is [0," << m_measurements.effort.size() - 1 << "]";
        return false;
    }
    *t = m_measurements.effort[j];
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getTorques(double* t)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getTorques] ";
    if (!m_measurements.valid.load())
    {
        yCError(CB) << errorPrefix << "No valid data received yet.";
        return false;
    }
    std::lock_guard<std::mutex> lock_measurements(m_measurements.mutex);
    std::copy(m_measurements.effort.begin(), m_measurements.effort.end(), t);
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getTorqueRange(int j, double* min, double* max)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getTorqueRanges(double* min, double* max)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getImpedance(int j, double* stiff, double* damp)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getImpedanceOffset(int j, double* offset)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getCurrentImpedanceLimit(int j, double* min_stiff, double* max_stiff, double* min_damp, double* max_damp)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getControlMode(int j, int* mode)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getControlModes(int* modes)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getControlModes(const int n_joint, const int* joints, int* modes)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setControlMode(const int j, const int mode)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setControlModes(const int n_joints, const int* joints, int* modes)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setControlModes(int* modes)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setPosition(int j, double ref)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setPositions(const int n_joints, const int* joints, const double* dpos)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setPositions(const double* refs)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRefPosition(const int joint, double* ref)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRefPositions(double* refs)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRefPositions(const int n_joint, const int* joints, double* refs)
{
    return false;
}

yarp::os::Stamp yarp::dev::IsaacSimControlBoardNWCROS2::getLastInputStamp()
{
    return yarp::os::Stamp();
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::velocityMove(const int n_joints, const int* joints, const double* spds)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRefVelocity(const int joint, double* vel)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRefVelocities(double* vels)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRefVelocities(const int n_joint, const int* joints, double* vels)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getInteractionMode(int j, yarp::dev::InteractionModeEnum* mode)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getInteractionModes(int n_joints, int* joints, yarp::dev::InteractionModeEnum* modes)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getInteractionModes(yarp::dev::InteractionModeEnum* modes)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setInteractionMode(int j, yarp::dev::InteractionModeEnum mode)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setInteractionModes(int n_joints, int* joints, yarp::dev::InteractionModeEnum* modes)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setInteractionModes(yarp::dev::InteractionModeEnum* modes)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setRefDutyCycle(int m, double ref)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setRefDutyCycles(const double* refs)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRefDutyCycle(int m, double* ref)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRefDutyCycles(double* refs)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getDutyCycle(int m, double* val)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getDutyCycles(double* vals)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getCurrent(int m, double* curr)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getCurrents(double* currs)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getCurrentRange(int m, double* min, double* max)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getCurrentRanges(double* min, double* max)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setRefCurrents(const double* currs)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setRefCurrent(int m, double curr)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setRefCurrents(const int n_motor, const int* motors, const double* currs)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRefCurrents(double* currs)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRefCurrent(int m, double* curr)
{
    return false;
}

void yarp::dev::IsaacSimControlBoardNWCROS2::updateMeasurements(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
{
    std::lock_guard<std::mutex> lock(m_measurements.mutex);
    constexpr double rad2deg = 180.0 / M_PI;

    m_measurements.name = msg->name;

    m_measurements.position = msg->position;
    for (auto& pos : m_measurements.position) {
        pos *= rad2deg;
    }

    m_measurements.velocity = msg->velocity;
    for (auto& vel : m_measurements.velocity) {
        vel *= rad2deg;
    }

    m_measurements.effort = msg->effort;
    m_measurements.timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    m_measurements.valid = true;
}

yarp::dev::IsaacSimControlBoardNWCROS2::CBNode::CBNode(const std::string& node_name, const std::string& topic_name, IsaacSimControlBoardNWCROS2* parent)
: rclcpp::Node(node_name)
{
    m_subscription = this->create_subscription<sensor_msgs::msg::JointState>(
        topic_name, 10,
        [parent](const sensor_msgs::msg::JointState::ConstSharedPtr msg) {
        parent->updateMeasurements(msg);
    });
}


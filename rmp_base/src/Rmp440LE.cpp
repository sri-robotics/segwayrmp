/*
  COPYRIGHT (c) 2014 SEGWAY Inc.

  Software License Agreement:

  The software supplied herewith by Segway Inc. (the "Company") for its 
  RMP Robotic Platforms is intended and supplied to you, the Company's 
  customer, for use solely and exclusively with Segway products. The 
  software is owned by the Company and/or its supplier, and is protected 
  under applicable copyright laws.  All rights are reserved. Any use in 
  violation of the foregoing restrictions may subject the user to criminal
  sanctions under applicable laws, as well as to civil liability for the 
  breach of the terms and conditions of this license. The Company may 
  immediately terminate this Agreement upon your use of the software with 
  any products that are not Segway products.

  You shall indemnify, defend and hold the Company harmless from any claims, 
  demands, liabilities or expenses, including reasonable attorneys fees, incurred 
  by the Company as a result of any claim or proceeding against the Company 
  arising out of or based upon: 

  (i) The combination, operation or use of the software by you with any hardware, 
      products, programs or data not supplied or approved in writing by the Company, 
      if such claim or proceeding would have been avoided but for such combination, 
      operation or use.

  (ii) The modification of the software by or on behalf of you.

  (iii) Your use of the software.

  THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,
  WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING, BUT NOT LIMITED
  TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
  PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
  IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
  CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
*/

#include "Rmp440LE.h"

#include <sstream>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <rmp_msgs/MotorStatus.h>
#include <rmp_msgs/Battery.h>

#include <RmpConfigurationCommand.h>

// Constant definitions
static const int PORT_NUMBER_MAX = 65536; 
static const double UPDATE_FREQUENCY_MIN = 0.5; // [Hz]
static const double UPDATE_FREQUENCY_MAX = 100.0; // [Hz]
static const double G_TO_M_S2 = 9.80665;
static const float DEG_TO_RAD = 0.0174532925;
static const double DEADMAN_VALIDITY_PERIOD = 0.5; // [s]
static const double MAX_UPDATE_PERIOD = 0.05; // [s]
static const double UNKNOWN_COV = 99999.0;
static const size_t WHEEL_COUNT = 4;
static const std::string LEFT_FRONT_WHEEL_JOINT_NAME("left_front_wheel");
static const std::string RIGHT_FRONT_WHEEL_JOINT_NAME("right_front_wheel");
static const std::string LEFT_REAR_WHEEL_JOINT_NAME("left_rear_wheel");
static const std::string RIGHT_REAR_WHEEL_JOINT_NAME("right_rear_wheel");
static const size_t LEFT_FRONT_WHEEL_IDX = 0;
static const size_t RIGHT_FRONT_WHEEL_IDX = 1; 
static const size_t LEFT_REAR_WHEEL_IDX = 2;
static const size_t RIGHT_REAR_WHEEL_IDX = 3; 

Rmp440LE::Rmp440LE()
  : m_NodeHandle("~")
  , m_OdometryInitialized(false)
{
}

Rmp440LE::~Rmp440LE()
{
  m_Rmp440LEInterface.ShutDown();
}

void Rmp440LE::Initialize()
{
  // Parameters
  std::string transportType, ipAddress, devicePort;
  int portNumber;
  std::string odometryTopic, jointStatesTopic, inertialTopic, pseTopic, motorStatusTopic, batteryTopic, velocityCommandTopic, deadmanTopic, audioCommandTopic, faultStatusTopic;
  double updateFrequency, maxTranslationalVelocity, maxTurnRate;
  
  m_NodeHandle.param("transport_type", transportType, std::string("udp"));
  m_NodeHandle.param("ip_address", ipAddress, std::string("192.168.0.40"));
  m_NodeHandle.param("port_number", portNumber, 8080);
  m_NodeHandle.param("device_port", devicePort, std::string("/dev/ttyACM0"));
  m_NodeHandle.param("update_frequency", updateFrequency, 50.0);
  m_NodeHandle.param("odometry_topic", odometryTopic, std::string("/rmp440le/odom"));
  m_NodeHandle.param("joint_states_topic", jointStatesTopic, std::string("/rmp440le/joint_states"));
  m_NodeHandle.param("inertial_topic", inertialTopic, std::string("/rmp440le/inertial"));
  m_NodeHandle.param("pse_topic", pseTopic, std::string("/rmp440le/pse"));
  m_NodeHandle.param("motor_status_topic", motorStatusTopic, std::string("/rmp440le/motor_status"));
  m_NodeHandle.param("battery_topic", batteryTopic, std::string("/rmp440le/battery"));
  m_NodeHandle.param("velocity_command_topic", velocityCommandTopic, std::string("/rmp440le/base/vel_cmd"));
  m_NodeHandle.param("deadman_topic", deadmanTopic, std::string("/rmp440le/deadman"));
  m_NodeHandle.param("audio_command_topic", audioCommandTopic, std::string("/rmp440le/audio_cmd"));
  m_NodeHandle.param("fault_status_topic", faultStatusTopic, std::string("/rmp440le/fault_status"));
  m_NodeHandle.param("max_translational_velocity", maxTranslationalVelocity, 8.0);
  m_NodeHandle.param("max_turn_rate", maxTurnRate, 4.4);

  // Start communication
  try 
  {
    if (transportType == std::string("udp"))
    {
      if ((portNumber < 0) || (portNumber > PORT_NUMBER_MAX))
      {
        ROS_ERROR_STREAM("Invalid port number: " << portNumber << " should be between 0 and " << PORT_NUMBER_MAX);
        return;
      }
      
      m_Rmp440LEInterface.Initialize(ipAddress, static_cast<uint16_t>(portNumber));
    }
    else if (transportType == std::string("usb"))
    {
      m_Rmp440LEInterface.Initialize(devicePort);
    }
    else
    {
      ROS_ERROR_STREAM("Unknown/unsupported transport: " << transportType);
      return;
    }
    
    m_Rmp440LEInterface.ResetParamsToDefault();

    m_Rmp440LEInterface.SetConfiguration(segway::RmpConfigurationCommandSet::SET_AUDIO_COMMAND, static_cast<uint32_t>(segway::MOTOR_AUDIO_TEST_SWEEP));

    ros::Duration(2.0).sleep();

    // Reset wheel encoders
    bool integratorsReset = m_Rmp440LEInterface.ResetIntegrators(segway::RESET_ALL_POSITION_DATA);

    if (!integratorsReset)
    {
      ROS_WARN_STREAM("Unable to reset position integrators.");
    }
    else
    {
      m_Rmp440LEInterface.SetConfiguration(segway::RmpConfigurationCommandSet::SET_AUDIO_COMMAND, static_cast<uint32_t>(segway::MOTOR_AUDIO_TEST_SWEEP));
    }

    ros::Duration(2.0).sleep();

    // Set the max speeds
    bool maxSpeedSet = m_Rmp440LEInterface.SetConfiguration(segway::RmpConfigurationCommandSet::SET_MAXIMUM_VELOCITY, static_cast<float>(maxTranslationalVelocity));
    maxSpeedSet = maxSpeedSet && m_Rmp440LEInterface.SetConfiguration(segway::RmpConfigurationCommandSet::SET_MAXIMUM_TURN_RATE, static_cast<float>(maxTurnRate));

    if (!maxSpeedSet)
    {
      ROS_WARN_STREAM("Unable to set the maximum speed.");
    }
    else
    {
      m_Rmp440LEInterface.SetConfiguration(segway::RmpConfigurationCommandSet::SET_AUDIO_COMMAND, static_cast<uint32_t>(segway::MOTOR_AUDIO_TEST_SWEEP));
    }

    // Set up ROS communication
    m_OdometryPublisher = m_NodeHandle.advertise<nav_msgs::Odometry>(odometryTopic, 1);
    m_JointStatesPublisher = m_NodeHandle.advertise<sensor_msgs::JointState>(jointStatesTopic, 1);
    m_InertialPublisher = m_NodeHandle.advertise<sensor_msgs::Imu>(inertialTopic, 1);
    m_PsePublisher = m_NodeHandle.advertise<sensor_msgs::Imu>(pseTopic, 1);
    m_MotorStatusPublisher = m_NodeHandle.advertise<rmp_msgs::MotorStatus>(motorStatusTopic, 1);
    m_BatteryPublisher = m_NodeHandle.advertise<rmp_msgs::Battery>(batteryTopic, 1);
    m_FaultStatusPublisher = m_NodeHandle.advertise<rmp_msgs::FaultStatus>(faultStatusTopic, 1);
    m_VelocityCommandSubscriber = m_NodeHandle.subscribe<geometry_msgs::TwistStamped>(velocityCommandTopic, 1, &Rmp440LE::ProcessVelocityCommand, this);
    m_DeadmanSubscriber = m_NodeHandle.subscribe<rmp_msgs::BoolStamped>(deadmanTopic, 1, &Rmp440LE::ProcessDeadman, this);
    m_AudioCommandSubscriber = m_NodeHandle.subscribe<rmp_msgs::AudioCommand>(audioCommandTopic, 1, &Rmp440LE::ProcessAudioCommand, this);
    
    if (updateFrequency < UPDATE_FREQUENCY_MIN)
    {
      updateFrequency = UPDATE_FREQUENCY_MIN;
    }
    else if (updateFrequency > UPDATE_FREQUENCY_MAX)
    {
      updateFrequency = UPDATE_FREQUENCY_MAX;
    }
    
    ros::Rate rate(updateFrequency);

    InitializeMessages();

    ros::Time lastUpdate = ros::Time::now();
    ros::Duration maxUpdatePeriod(MAX_UPDATE_PERIOD);
    bool forceUpdate = false;
    
    // Processing loop
    while (ros::ok())
    {
      ros::spinOnce();

      if ((ros::Time::now() - lastUpdate) > maxUpdatePeriod)
      {
        forceUpdate = true;
      }

      if (m_Rmp440LEInterface.Update(forceUpdate))
      {
        UpdateStatus();
        lastUpdate = ros::Time::now();
        forceUpdate = false;
      }

      rate.sleep();
    }
  } 
  catch (std::exception& rException) 
  {
    ROS_ERROR_STREAM("Exception caught: " << rException.what() << ". Will return.");
    return;
  }
}

void Rmp440LE::InitializeMessages()
{
  std::string baseFrame, odometryFrame, inertialFrame, pseFrame;

  m_NodeHandle.param("base_frame", baseFrame, std::string("/rmp440le/base_footprint"));
  m_NodeHandle.param("odometry_frame", odometryFrame, std::string("/rmp440le/odom"));
  m_NodeHandle.param("inertial_frame", inertialFrame, std::string("/rmp440le/inertial"));
  m_NodeHandle.param("pse_frame", pseFrame, std::string("/rmp440le/pse"));
  
  m_OdometryMsg.header.frame_id = odometryFrame;
  m_OdometryMsg.child_frame_id = baseFrame;
  m_OdometryMsg.pose.pose = geometry_msgs::Pose();
  m_OdometryMsg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  m_OdometryMsg.pose.covariance = {{ UNKNOWN_COV, 0, 0, 0, 0, 0,
                                     0, UNKNOWN_COV, 0, 0, 0, 0,
                                     0, 0, UNKNOWN_COV, 0, 0, 0,
                                     0, 0, 0, UNKNOWN_COV, 0, 0,
                                     0, 0, 0, 0, UNKNOWN_COV, 0,
                                     0, 0, 0, 0, 0, UNKNOWN_COV }};

  m_JointStateMsg.name.resize(WHEEL_COUNT);
  m_JointStateMsg.position.resize(WHEEL_COUNT, 0.0);
  m_JointStateMsg.velocity.resize(WHEEL_COUNT, 0.0);
  m_JointStateMsg.effort.resize(0);
  m_JointStateMsg.name[LEFT_FRONT_WHEEL_IDX] = LEFT_FRONT_WHEEL_JOINT_NAME;
  m_JointStateMsg.name[RIGHT_FRONT_WHEEL_IDX] = RIGHT_FRONT_WHEEL_JOINT_NAME;
  m_JointStateMsg.name[LEFT_REAR_WHEEL_IDX] = LEFT_REAR_WHEEL_JOINT_NAME;
  m_JointStateMsg.name[RIGHT_REAR_WHEEL_IDX] = RIGHT_REAR_WHEEL_JOINT_NAME;


  m_InertialMsg.header.frame_id = inertialFrame;

  boost::array<double, 9> covariance = {{ UNKNOWN_COV, 0, 0,
                                          0, UNKNOWN_COV, 0,
                                          0, 0, UNKNOWN_COV }};

  m_InertialMsg.orientation_covariance = covariance;
  m_InertialMsg.angular_velocity_covariance = covariance;
  m_InertialMsg.linear_acceleration_covariance = covariance;

  m_PseMsg = m_InertialMsg;
  m_PseMsg.header.frame_id = pseFrame;
}

void Rmp440LE::ProcessVelocityCommand(const geometry_msgs::TwistStamped::ConstPtr& rpVelocityCommand)
{
  if (!IsDeadmanValid())
  {
    return;
  }
  
  if (m_Rmp440LEInterface.GetUserDefinedFeedback<uint32_t>(segway::OPERATIONAL_STATE) != segway::TRACTOR_MODE)
  {
    ROS_WARN_STREAM("Velocity command won't be processed. The rmp is in " << m_Rmp440LEInterface.GetUserDefinedFeedback<uint32_t>(segway::OPERATIONAL_STATE) << " mode and should be in " << segway::TRACTOR_MODE << " (TRACTOR)");

    return;
  }
  
  float maximumVelocity = m_Rmp440LEInterface.GetMaximumVelocity();
  float maximumTurnRate = m_Rmp440LEInterface.GetMaximumTurnRate();

  if ((maximumVelocity <= 0.0) || (maximumTurnRate <= 0))
  {
    std::stringstream stringStream;
    stringStream << "Invalid max velocity/turn rate: " << maximumVelocity << "/" << maximumTurnRate;
    
    throw std::logic_error(stringStream.str());
  }
  
  float normalizedVelocity = static_cast<float>(rpVelocityCommand->twist.linear.x) / maximumVelocity;
  // Segway defines yaw as negative z
  float normalizedYawRate = -1.0 * static_cast<float>(rpVelocityCommand->twist.angular.z) / maximumTurnRate;
  
  if ((normalizedVelocity > fabs(1.0)) || (normalizedYawRate > fabs(1.0)))
  {
    ROS_WARN_STREAM("Velocity command out of range: " << rpVelocityCommand->twist.linear.x << ", " << rpVelocityCommand->twist.angular.z << " should be within: " << maximumVelocity << " [m/s], " << maximumTurnRate << " [rad/s]");
    
    return;
  }
  
  m_Rmp440LEInterface.SetVelocity(normalizedVelocity, normalizedYawRate);
}

void Rmp440LE::ProcessDeadman(const rmp_msgs::BoolStamped::ConstPtr& rpDeadmanMsg)
{
  m_DeadmanMsg = *rpDeadmanMsg;
}

void Rmp440LE::ProcessAudioCommand(const rmp_msgs::AudioCommand::ConstPtr& rpAudioCommand)
{
  uint32_t command = rpAudioCommand->command;
      
  if ((command < segway::MOTOR_AUDIO_PLAY_NO_SONG) || (command > segway::MOTOR_AUDIO_SIMULATE_MOTOR_NOISE))
  {
    ROS_WARN_STREAM("Invalid audio command received: " << command << " should be between " << segway::MOTOR_AUDIO_PLAY_NO_SONG << " and " << segway::MOTOR_AUDIO_SIMULATE_MOTOR_NOISE << ". Won't be processed.");

    return;
  }
  
  m_Rmp440LEInterface.SetConfiguration(segway::RmpConfigurationCommandSet::SET_AUDIO_COMMAND, command);
}

void Rmp440LE::UpdateStatus()
{
  UpdateImu();
  UpdateOdometry();
  UpdateBattery();
  UpdateMotorStatus();
  UpdateFaultStatus();
}

void Rmp440LE::UpdateImu()
{
  // Pse Data
  uint32_t pseDataIsValid = m_Rmp440LEInterface.GetUserDefinedFeedback<uint32_t>(segway::PSE_DATA_IS_VALID);
  if (pseDataIsValid >= segway::PSE_VALID)
  {
    float pitch = DEG_TO_RAD * m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::PSE_PITCH_DEG);
    float roll = DEG_TO_RAD * m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::PSE_ROLL_DEG);

    m_PseMsg.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, 0.0);
    m_PseMsg.angular_velocity.x = DEG_TO_RAD * m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::PSE_PITCH_RATE_DPS);
    m_PseMsg.angular_velocity.y = DEG_TO_RAD * m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::PSE_ROLL_RATE_DPS);
    m_PseMsg.angular_velocity.z  = DEG_TO_RAD * m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::PSE_YAW_RATE_DPS);
    m_PseMsg.header.stamp = ros::Time::now();

    m_PsePublisher.publish(m_PseMsg);
  }

  // Inertial data
  m_InertialMsg.linear_acceleration.x = G_TO_M_S2 * m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::INERTIAL_X_ACC_G);
  m_InertialMsg.linear_acceleration.y = G_TO_M_S2 * m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::INERTIAL_Y_ACC_G);
  m_InertialMsg.linear_acceleration.z = 0.0;
  m_InertialMsg.angular_velocity.x = m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::INERTIAL_X_RATE_RPS);
  m_InertialMsg.angular_velocity.y = m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::INERTIAL_Y_RATE_RPS);
  m_InertialMsg.angular_velocity.z = m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::INERTIAL_Z_RATE_RPS);
  m_InertialMsg.header.stamp = ros::Time::now();

  m_InertialPublisher.publish(m_InertialMsg);
}

void Rmp440LE::UpdateOdometry()  
{
  double linearDisplacement = static_cast<double>(m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::LINEAR_POS_M));
  double leftFrontDisplacement = static_cast<double>(m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::LEFT_FRONT_POS_M));
  double leftRearDisplacement = static_cast<double>(m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::LEFT_REAR_POS_M));
  double rightFrontDisplacement = static_cast<double>(m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::RIGHT_FRONT_POS_M));
  double rightRearDisplacement = static_cast<double>(m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::RIGHT_REAR_POS_M));

  if (m_OdometryInitialized)
  {
    double deltaLinearDisplacement = linearDisplacement - m_WheelsDisplacement.m_Linear;
    double deltaLeftSideDisplacement = ((leftFrontDisplacement - m_WheelsDisplacement.m_LeftFront) + (leftRearDisplacement - m_WheelsDisplacement.m_LeftRear)) / 2.0;
    double deltaRightSideDisplacement = ((rightFrontDisplacement - m_WheelsDisplacement.m_RightFront) + (rightRearDisplacement - m_WheelsDisplacement.m_RightRear)) / 2.0;

    double wheelTrackWidth = static_cast<double>(m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::FRAM_WHEEL_TRACK_WIDTH_M));
    double deltaHeading = (deltaRightSideDisplacement - deltaLeftSideDisplacement) / wheelTrackWidth;

    double previousHeading = tf::getYaw(m_OdometryMsg.pose.pose.orientation);
    double averageHeading = previousHeading + deltaHeading / 2.0;

    m_OdometryMsg.pose.pose.position.x += deltaLinearDisplacement * cos(averageHeading);
    m_OdometryMsg.pose.pose.position.y += deltaLinearDisplacement * sin(averageHeading);
    m_OdometryMsg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(previousHeading + deltaHeading);
    m_OdometryMsg.twist.twist.linear.x = static_cast<double>(m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::LINEAR_VEL_MPS));
    m_OdometryMsg.twist.twist.angular.z = -1.0 * static_cast<double>(m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::DIFFERENTIAL_WHEEL_VEL_RPS));
    m_OdometryMsg.header.stamp = ros::Time::now();

    m_OdometryPublisher.publish(m_OdometryMsg);
  }
  else
  {
    m_OdometryInitialized = true;
  }

  // Update
  m_WheelsDisplacement.m_Linear = linearDisplacement;
  m_WheelsDisplacement.m_LeftFront = leftFrontDisplacement;
  m_WheelsDisplacement.m_LeftRear = leftRearDisplacement;
  m_WheelsDisplacement.m_RightFront = rightFrontDisplacement;
  m_WheelsDisplacement.m_RightRear = rightRearDisplacement;

  double inverseRadius = 2.0 / static_cast<double>(m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::FRAM_TIRE_DIAMETER_M));
  
  m_JointStateMsg.position[LEFT_FRONT_WHEEL_IDX] = inverseRadius * leftFrontDisplacement;
  m_JointStateMsg.position[RIGHT_FRONT_WHEEL_IDX] = inverseRadius * rightFrontDisplacement;
  m_JointStateMsg.position[LEFT_REAR_WHEEL_IDX] = inverseRadius * leftRearDisplacement;
  m_JointStateMsg.position[RIGHT_REAR_WHEEL_IDX] = inverseRadius * rightRearDisplacement;
  m_JointStateMsg.velocity[LEFT_FRONT_WHEEL_IDX] = inverseRadius * static_cast<double>(m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::LEFT_FRONT_VEL_MPS));
  m_JointStateMsg.velocity[RIGHT_FRONT_WHEEL_IDX] = inverseRadius * static_cast<double>(m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::RIGHT_FRONT_VEL_MPS));
  m_JointStateMsg.velocity[LEFT_REAR_WHEEL_IDX] = inverseRadius * static_cast<double>(m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::LEFT_REAR_VEL_MPS));
  m_JointStateMsg.velocity[RIGHT_REAR_WHEEL_IDX] = inverseRadius * static_cast<double>(m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::RIGHT_REAR_VEL_MPS));
  m_JointStateMsg.header.stamp = ros::Time::now();

  m_JointStatesPublisher.publish(m_JointStateMsg);
}

void Rmp440LE::UpdateBattery()
{
  rmp_msgs::Battery batteryMsg;
  // Soc
  batteryMsg.charge_state[rmp_msgs::Battery::BATT_FRONT_1_IDX] = m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::FRONT_BASE_BATT_1_SOC);
  batteryMsg.charge_state[rmp_msgs::Battery::BATT_FRONT_2_IDX] = m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::FRONT_BASE_BATT_2_SOC);
  batteryMsg.charge_state[rmp_msgs::Battery::BATT_REAR_1_IDX] = m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::REAR_BASE_BATT_1_SOC);
  batteryMsg.charge_state[rmp_msgs::Battery::BATT_REAR_2_IDX] = m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::REAR_BASE_BATT_2_SOC);
  batteryMsg.charge_state[rmp_msgs::Battery::BATT_AUX_IDX] = m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::AUX_BATT_SOC);
  // Temperature
  batteryMsg.temperature[rmp_msgs::Battery::BATT_FRONT_1_IDX] =  m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::FRONT_BASE_BATT_1_TEMP_DEGC);
  batteryMsg.temperature[rmp_msgs::Battery::BATT_FRONT_2_IDX] =  m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::FRONT_BASE_BATT_2_TEMP_DEGC);
  batteryMsg.temperature[rmp_msgs::Battery::BATT_REAR_1_IDX] =  m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::REAR_BASE_BATT_1_TEMP_DEGC);
  batteryMsg.temperature[rmp_msgs::Battery::BATT_REAR_2_IDX] =  m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::REAR_BASE_BATT_2_TEMP_DEGC);
  batteryMsg.temperature[rmp_msgs::Battery::BATT_AUX_IDX] =  m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::AUX_BATT_TEMP_DEGC);
  // Other fields...
  batteryMsg.min_propulsion_charge_state = m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::MIN_PROPULSION_BATT_SOC);
  batteryMsg.aux_battery_voltage = m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::AUX_BATT_VOLTAGE_V);
  batteryMsg.aux_battery_current = m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::AUX_BATT_CURRENT_A);
  batteryMsg.abb_system_status = m_Rmp440LEInterface.GetUserDefinedFeedback<uint32_t>(segway::ABB_SYSTEM_STATUS);
  batteryMsg.aux_battery_status = m_Rmp440LEInterface.GetUserDefinedFeedback<uint32_t>(segway::AUX_BATT_STATUS);
  batteryMsg.header.stamp = ros::Time::now();
  
  m_BatteryPublisher.publish(batteryMsg);
}

void Rmp440LE::UpdateMotorStatus()
{
  rmp_msgs::MotorStatus motorStatus;
  // Current 
  motorStatus.current[rmp_msgs::MotorStatus::RIGHT_FRONT_IDX] = m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::RIGHT_FRONT_CURRENT_A0PK);
  motorStatus.current[rmp_msgs::MotorStatus::LEFT_FRONT_IDX] = m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::LEFT_FRONT_CURRENT_A0PK);
  motorStatus.current[rmp_msgs::MotorStatus::RIGHT_REAR_IDX] = m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::RIGHT_REAR_CURRENT_A0PK);
  motorStatus.current[rmp_msgs::MotorStatus::LEFT_REAR_IDX] = m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::LEFT_REAR_CURRENT_A0PK);
  // Current limit
  motorStatus.current_limit[rmp_msgs::MotorStatus::RIGHT_FRONT_IDX] = m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::RIGHT_FRONT_CURRENT_LIMIT_A0PK);
  motorStatus.current_limit[rmp_msgs::MotorStatus::LEFT_FRONT_IDX] = m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::LEFT_FRONT_CURRENT_LIMIT_A0PK);
  motorStatus.current_limit[rmp_msgs::MotorStatus::RIGHT_REAR_IDX] = m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::RIGHT_REAR_CURRENT_LIMIT_A0PK);
  motorStatus.current_limit[rmp_msgs::MotorStatus::LEFT_REAR_IDX] = m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::LEFT_REAR_CURRENT_LIMIT_A0PK);
  // Mcu inst power
  motorStatus.mcu_inst_power[rmp_msgs::MotorStatus::RIGHT_FRONT_IDX] = m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::MCU_0_INST_POWER_W);
  motorStatus.mcu_inst_power[rmp_msgs::MotorStatus::LEFT_FRONT_IDX] = m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::MCU_1_INST_POWER_W);
  motorStatus.mcu_inst_power[rmp_msgs::MotorStatus::RIGHT_REAR_IDX] = m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::MCU_2_INST_POWER_W);
  motorStatus.mcu_inst_power[rmp_msgs::MotorStatus::LEFT_REAR_IDX] = m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::MCU_3_INST_POWER_W);
  // Mcu total energy
  motorStatus.mcu_total_energy[rmp_msgs::MotorStatus::RIGHT_FRONT_IDX] = m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::MCU_0_TOTAL_ENERGY_WH);
  motorStatus.mcu_total_energy[rmp_msgs::MotorStatus::LEFT_FRONT_IDX] = m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::MCU_1_TOTAL_ENERGY_WH);
  motorStatus.mcu_total_energy[rmp_msgs::MotorStatus::RIGHT_REAR_IDX] = m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::MCU_2_TOTAL_ENERGY_WH);
  motorStatus.mcu_total_energy[rmp_msgs::MotorStatus::LEFT_REAR_IDX] = m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::MCU_3_TOTAL_ENERGY_WH);
  // Other fields...
  motorStatus.max_current = m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::MAX_MOTOR_CURRENT_A0PK);
  motorStatus.min_current_limit = m_Rmp440LEInterface.GetUserDefinedFeedback<float>(segway::MIN_MOTOR_CURRENT_LIMIT_A0PK);
  motorStatus.header.stamp = ros::Time::now();

  m_MotorStatusPublisher.publish(motorStatus);
}

void Rmp440LE::UpdateFaultStatus()
{
  rmp_msgs::FaultStatus faultStatus;
  m_Rmp440LEInterface.GetFaultStatusDescription(faultStatus.fault_list);
  faultStatus.header.stamp = ros::Time::now();

  m_FaultStatusPublisher.publish(faultStatus);
}

bool Rmp440LE::IsDeadmanValid()
{
  ros::Duration duration = ros::Time::now() - m_DeadmanMsg.header.stamp;
  
  if ( m_DeadmanMsg.data &&
       (duration < ros::Duration(DEADMAN_VALIDITY_PERIOD)) )
  {
    return true;
  }

  return false;
}

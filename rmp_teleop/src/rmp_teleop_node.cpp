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

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/TwistStamped.h>
#include <rmp_msgs/BoolStamped.h>
#include <rmp_msgs/AudioCommand.h>

#include <RmpJoystickConverter.h>

/**
 * This class is used to convert joystick input to rmp control commands.
 */
class RmpTeleop
{
public:
  /**
   * Constructor
   */
  RmpTeleop();

  /**
   * Destructor
   */
  ~RmpTeleop();

  /**
   * Initialize the communication
   */
  void Initialize();

private:
  /**
   * Process joystick messages
   */
  void ProcessJoystickMessage(const sensor_msgs::Joy::ConstPtr& rpJoyMessage);

  /**
   * Ros interface
   */
  ros::NodeHandle m_NodeHandle;

  /**
   * Velocity command publisher
   */
  ros::Publisher m_VelocityCommandPublisher;

  /**
   * Deadman message publisher
   */
  ros::Publisher m_DeadmanPublisher;

  /**
   * Audio command publisher
   */
  ros::Publisher m_AudioCommandPublisher;

  /**
   * Joystick message subscriber
   */
  ros::Subscriber m_JoystickMessageSubscriber;

  /**
   * Joystick converter
   * Convert joystick messages to rmp commands.
   */
  JoystickConverter::Ptr m_pJoystickConverter;

  /**
   * Translational velocity scale
   * Scale joystick input to translational velocity command
   */
  double m_TranslationalVelocityScale;

  /**
   * Rotational velocity scale
   * Scale joystick input to rotational velocity command
   */
  double m_RotationalVelocityScale;

  /**
   * Translational velocity scale
   * Scale joystick input to translational velocity command
   * when the boost is active
   */
  double m_TranslationalVelocityBoostScale;

  /**
   * Rotational velocity scale
   * Scale joystick input to rotational velocity command
   * when the boost is active
   */
  double m_RotationalVelocityBoostScale;
};

RmpTeleop::RmpTeleop()
  : m_NodeHandle("~")
  , m_TranslationalVelocityScale(0.0)
  , m_RotationalVelocityScale(0.0)
  , m_TranslationalVelocityBoostScale(0.0)
  , m_RotationalVelocityBoostScale(0.0)
{}

RmpTeleop::~RmpTeleop()
{}

void RmpTeleop::Initialize()
{
  // Parameters
  std::string joyMessageTopic, velocityCommandTopic, deadmanTopic, audioCommandTopic;
  double updateFrequency;

  m_NodeHandle.param("joy_topic", joyMessageTopic, std::string("/rmp440le/joy"));
  m_NodeHandle.param("velocity_command_topic", velocityCommandTopic, std::string("/rmp440le/base/vel_cmd"));
  m_NodeHandle.param("deadman_topic", deadmanTopic, std::string("/rmp440le/deadman"));
  m_NodeHandle.param("audio_command_topic", audioCommandTopic, std::string("/rmp440le/audio_cmd"));
  m_NodeHandle.param("update_frequency", updateFrequency, 50.0);
  m_NodeHandle.param("translational_velocity_scale", m_TranslationalVelocityScale, 2.2352);
  m_NodeHandle.param("rotational_velocity_scale", m_RotationalVelocityScale, 3.0);
  m_NodeHandle.param("translational_velocity_boost_scale", m_TranslationalVelocityBoostScale, 8.0);
  m_NodeHandle.param("rotational_velocity_boost_scale", m_RotationalVelocityBoostScale, 4.4);

  m_pJoystickConverter = JoystickConverter::Create(JoystickConverter::XBOX_WIRELLESS);

  // Set up ROS communication
  m_VelocityCommandPublisher = m_NodeHandle.advertise<geometry_msgs::TwistStamped>(velocityCommandTopic, 1);
  m_DeadmanPublisher = m_NodeHandle.advertise<rmp_msgs::BoolStamped>(deadmanTopic, 1);
  m_AudioCommandPublisher = m_NodeHandle.advertise<rmp_msgs::AudioCommand>(audioCommandTopic, 1);
  m_JoystickMessageSubscriber = m_NodeHandle.subscribe<sensor_msgs::Joy>(joyMessageTopic, 1, &RmpTeleop::ProcessJoystickMessage, this);

  ros::Rate rate(updateFrequency);

  while (ros::ok())
  {
    ros::spinOnce();

    rate.sleep();
  }
}

void RmpTeleop::ProcessJoystickMessage(const sensor_msgs::Joy::ConstPtr& rpJoyMessage)
{
  geometry_msgs::TwistStamped velocityCommand;

  if (m_pJoystickConverter->GetBoost(*rpJoyMessage))
  {
    velocityCommand.twist.linear.x = m_TranslationalVelocityBoostScale * m_pJoystickConverter->GetTranslationalVelocity(*rpJoyMessage);
    velocityCommand.twist.angular.z = m_RotationalVelocityBoostScale * m_pJoystickConverter->GetRotationalVelocity(*rpJoyMessage);
  }
  else
  {
    velocityCommand.twist.linear.x = m_TranslationalVelocityScale * m_pJoystickConverter->GetTranslationalVelocity(*rpJoyMessage);
    velocityCommand.twist.angular.z = m_RotationalVelocityScale * m_pJoystickConverter->GetRotationalVelocity(*rpJoyMessage);
  }
  

  velocityCommand.header.stamp = ros::Time::now();

  m_VelocityCommandPublisher.publish(velocityCommand);

  rmp_msgs::BoolStamped deadman;
  deadman.header.stamp = ros::Time::now();
    
  if (m_pJoystickConverter->GetDeadman(*rpJoyMessage))
  {
    deadman.data = true;
  }
  else
  {
    deadman.data = false;
  }

  m_DeadmanPublisher.publish(deadman);

  if (m_pJoystickConverter->GetAudioCommand(*rpJoyMessage) > 0)
  {
    rmp_msgs::AudioCommand audioCommand;
    audioCommand.header.stamp = ros::Time::now();
    audioCommand.command = static_cast<uint32_t>(m_pJoystickConverter->GetAudioCommand(*rpJoyMessage));

    m_AudioCommandPublisher.publish(audioCommand);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rmp_teleop_node");
  
  RmpTeleop teleop;
  teleop.Initialize();
  
  return 0;
}

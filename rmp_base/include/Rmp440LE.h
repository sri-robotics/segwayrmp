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

#ifndef RMP_440LE_H
#define RMP_440LE_H 

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <rmp_msgs/BoolStamped.h>
#include <rmp_msgs/AudioCommand.h>
#include <rmp_msgs/FaultStatus.h>

#include <Rmp440LEInterface.h>

/**
 * This structure represents the wheel displacements of a Segway RMP.
 */
struct WheelsDisplacement
{
  /**
   * Constructor
   */
  WheelsDisplacement()
    : m_Linear(0.0)
    , m_LeftFront(0.0)
    , m_LeftRear(0.0)
    , m_RightFront(0.0)
    , m_RightRear(0.0)
  {}

  /**
   * RMP linear displacement [m]
   */
  double m_Linear;
  
  /**
   * Left front wheel linear displacement [m]
   */
  double m_LeftFront;
  
  /**
   * Left rear wheel linear displacement [m]
   */
  double m_LeftRear;
  
  /**
   * Right front wheel linear displacement [m]
   */
  double m_RightFront;
  
  /**
   * Right rear wheel linear displacement [m]
   */
  double m_RightRear;
};

/**
 * This class setup a ros interface to the Segway RMP 440LE.
 */
class Rmp440LE
{
public:
  /**
   * Constructor
   */
  Rmp440LE();
  
  /**
   * Destructor
   */
  ~Rmp440LE();
  
  /**
   * Initialize the communication.
   */
  void Initialize();
  
private:
  /**
   * Initialize ros messages
   */
  void InitializeMessages();

  /**
   * Process velocity commands. Velocity commands are defined by a linear and angular vector.
   * Translational velocity is defined in the x field of the linear vector in [m/s] with the positive axis pointing forward.
   * Turn rate is defined in the z field of the angular vector in [rad/s] with the positive axis pointing upward.
   * The rmp has to be in tractor mode for the command to be effective.
   * The command should not be outside the predefined range. Please refer to the rmp user manual for more information.
   * @param rpVelocityCommand velocity command
   * @throw std::logic_error if the velocity command is out of range
   */
  void ProcessVelocityCommand(const geometry_msgs::TwistStamped::ConstPtr& rpVelocityCommand);

  /**
   * Process deadman messages. The deadman needs to be pressed (true) for the velocity commands to be processed.
   * @param rpDeadmanMsg deadman message
   */
  void ProcessDeadman(const rmp_msgs::BoolStamped::ConstPtr& rpDeadmanMsg);

  /**
   * Process audio commands
   * The different audio commands are defined in the rmp user manual (p.51) and in the rmp_msg package.
   * @param rpAudioCommand audio command
   */
  void ProcessAudioCommand(const rmp_msgs::AudioCommand::ConstPtr& rpAudioCommand);

  /**
   * Publish rmp status related information such as imu, odometry, battery, motor and fault(s).
   */
  void UpdateStatus();

  /**
   * Publish Pse (Pitch State Estimate) and inertial data
   * The coordinate systems are defined in the rmp user manual (p.12).
   */
  void UpdateImu();

  /**
   * Publish odometry
   */
  void UpdateOdometry();

  /**
   * Publish battery data
   */
  void UpdateBattery();

  /**
   * Publish motor status
   */
  void UpdateMotorStatus();

  /**
   * Publish fault status
   * An empty messages means no fault
   */
  void UpdateFaultStatus();

  /**
   * Check wether the deadman command is valid
   * It has to be true and has to be "recent".
   * @result wether the deadman command is valid
   */
  bool IsDeadmanValid();

  /**
   * Rmp 440LE interface
   */
  segway::Rmp440LEInterface m_Rmp440LEInterface;

  /**
   * Ros interface
   */
  ros::NodeHandle m_NodeHandle;

  /**
   * Odometry publisher
   */
  ros::Publisher m_OdometryPublisher;

  /**
   * Joint states publisher
   */ 
  ros::Publisher m_JointStatesPublisher;

  /**
   * Inertial data publisher
   */
  ros::Publisher m_InertialPublisher;

  /**
   * Pse (Pitch State Estimate) publisher
   */
  ros::Publisher m_PsePublisher;

  /**
   * Motor status publisher
   */
  ros::Publisher m_MotorStatusPublisher;

  /**
   * Battery data publisher
   */
  ros::Publisher m_BatteryPublisher;

  /**
   * Fault status publisher
   */
  ros::Publisher m_FaultStatusPublisher;

  /**
   * Velocity command subscriber
   */
  ros::Subscriber m_VelocityCommandSubscriber;

  /**
   * Deadman command subscriber
   */
  ros::Subscriber m_DeadmanSubscriber;

  /**
   * Audio command subscriber
   */
  ros::Subscriber m_AudioCommandSubscriber;

  /**
   * Odometry message 
   */
  nav_msgs::Odometry m_OdometryMsg;

  /**
   * Joint state message
   */
  sensor_msgs::JointState m_JointStateMsg;

  /**
   * Inertial data message
   */
  sensor_msgs::Imu m_InertialMsg;

  /**
   * Pse (Pitch State Estimate) message
   */
  sensor_msgs::Imu m_PseMsg;

  /**
   * Most recent deadman message
   */
  rmp_msgs::BoolStamped m_DeadmanMsg;

  /**
   * Last updated wheel displacement
   */
  WheelsDisplacement m_WheelsDisplacement;

  /**
   * Wether the odometry has been initialized
   */
  bool m_OdometryInitialized;
};

#endif // RMP_440LE_H

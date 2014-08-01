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

#ifndef RMP_CONFIGURATION_COMMAND_H
#define RMP_CONFIGURATION_COMMAND_H

#include <RmpUserDefinedFeedbackType.h>

namespace segway
{
  /**
   * Define Rmp Configuration Command Ids
   * Please read the rmp user manual (for instance, page 53 of the RMP 440LE manual)
   */
  enum RmpConfigurationCommandId
  {
    CONFIG_CMD_ID_NONE = 0,
    CONFIG_CMD_ID_SET_MAXIMUM_VELOCITY,
    CONFIG_CMD_ID_SET_MAXIMUM_ACCELERATION,
    CONFIG_CMD_ID_SET_MAXIMUM_DECELERATION,
    CONFIG_CMD_ID_SET_DTZ_DECEL_RATE,
    CONFIG_CMD_ID_SET_COASTDOAN_ACCEL,
    CONFIG_CMD_ID_SET_MAXIMUM_TURN_RATE,
    CONFIG_CMD_ID_SET_MAXIMUM_TURN_ACCEL,
    CONFIG_CMD_ID_SET_TIRE_DIAMETER,
    CONFIG_CMD_ID_SET_WHEEL_BASE_LENGTH,
    CONFIG_CMD_ID_SET_WHEEL_TRACK_WIDTH,
    CONFIG_CMD_ID_SET_TRANSMISSION_RATIO,
    CONFIG_CMD_ID_SET_INPUT_CONFIG_BITMAP,
    CONFIG_CMD_ID_SET_ETH_IP_ADDRESS,
    CONFIG_CMD_ID_SET_ETH_PORT_NUMBER,
    CONFIG_CMD_ID_SET_ETH_SUBNET_MASK,
    CONFIG_CMD_ID_SET_GATEWAY,
    CONFIG_CMD_ID_SET_USER_FB_1_BITMAP,
    CONFIG_CMD_ID_SET_USER_FB_2_BITMAP,
    CONFIG_CMD_ID_SET_USER_FB_3_BITMAP,
    CONFIG_CMD_ID_SET_USER_FB_4_BITMAP,
    CONFIG_CMD_ID_FORCE_CONFIG_FEEDBACK_BITMAPS = 30,
    CONFIG_CMD_ID_SET_AUDIO_COMMAND,
    CONFIG_CMD_ID_SET_OPERATIONAL_MODE,
    CONFIG_CMD_ID_SEND_SP_FAULT_LOG,
    CONFIG_CMD_ID_RESET_INTEGRATORS,
    CONFIG_CMD_ID_RESET_PARAMS_TO_DEFAULT
  };

  /**
   * Define Rmp Operational Mode Requests
   * Please read the rmp user manual (for instance, page 51 of the RMP 440LE manual)
   */
  enum RmpOperationalModeRequest
  {
    DISABLE_REQUEST = 1,
    POWERDOWN_REQUEST,
    DTZ_REQUEST,
    STANDBY_REQUEST,
    TRACTOR_REQUEST
  };

  /**
   * Define Audio Songs
   * Please read the rmp user manual (for instance, page 51 of the RMP 440LE manual)
   */
  enum RmpAudioSong
  {
    MOTOR_AUDIO_PLAY_NO_SONG = 0,
    MOTOR_AUDIO_PLAY_POWER_ON_SONG,
    MOTOR_AUDIO_PLAY_POWER_OFF_SONG,
    MOTOR_AUDIO_PLAY_ALARM_SONG,
    MOTOR_AUDIO_PLAY_MODE_UP_SONG,
    MOTOR_AUDIO_PLAY_MODE_DOWN_SONG,
    MOTOR_AUDIO_PLAY_ENTER_ALARM_SONG,
    MOTOR_AUDIO_PLAY_EXIT_ALARM_SONG,
    MOTOR_AUDIO_PLAY_FINAL_SHUTDOWN_SONG,
    MOTOR_AUDIO_PLAY_CORRECT_ISSUE,
    MOTOR_AUDIO_PLAY_ISSUE_CORRECTED,
    MOTOR_AUDIO_PLAY_CORRECT_ISSUE_REPEATING,
    MOTOR_AUDIO_PLAY_BEGINNER_ACK,
    MOTOR_AUDIO_PLAY_EXPERT_ACK,
    MOTOR_AUDIO_ENTER_FOLLOW,
    MOTOR_AUDIO_TEST_SWEEP,
    MOTOR_AUDIO_SIMULATE_MOTOR_NOISE
  };

  /**
   * Define Reset Integrator Bitmaps
   * Please read the rmp user manual (for instance, page 52 of the RMP 440LE manual)
   */
  enum RmpResetIntegrator
  {
    RESET_LINEAR_POSITION = 0x00000001,
    RESET_RIGHT_FRONT_POSITION = 0x00000002,
    RESET_LEFT_FRONT_POSITION = 0x00000004,
    RESET_RIGHT_REAR_POSITION = 0x00000008,
    RESET_LEFT_REAR_POSITION = 0x00000010,
    RESET_ALL_POSITION_DATA = 0x0000001F
  };

  class RmpConfigurationCommandSet;

  /**
   * This struct describes a configuration command
   * Please read the rmp user manual (for instance, page 53 of the RMP 440LE manual)
   */
  template<typename T>
  struct RmpConfigurationCommand
  {
    friend class RmpConfigurationCommandSet;
    
  public:
    /**
     * Configuration command id
     */
    const RmpConfigurationCommandId m_CommandId;

    /**
     * Minimum value of the command
     */
    const T m_Min;

    /**
     * Maximum value of the command
     */
    const T m_Max;

    /**
     * User defined feedback affected by the command
     */
    const UserDefinedFeedbackType m_FeedbackType;

  private:
    /**
     * All constructors should be private.
     * The user should not be able to create/modify any objects of this class.
     */
    
    /**
     * Private constructor
     * @param commandId command id
     * @param min minimum value of the command
     * @param max maximum value of the command
     * @param feedbackType user defined feedback type
     */
    RmpConfigurationCommand(RmpConfigurationCommandId commandId, T min, T max, UserDefinedFeedbackType feedbackType)
      : m_CommandId(commandId)
      , m_Min(min)
      , m_Max(max)
      , m_FeedbackType(feedbackType)
    {}

    /**
    * Default constructor
    */
    RmpConfigurationCommand();

    /**
     * Copy constructor
     */
    RmpConfigurationCommand(const RmpConfigurationCommand&);

    /**
     * Assignment operator
     */
    RmpConfigurationCommand& operator=(const RmpConfigurationCommand&);

    /**
     * Destructor
     */
    ~RmpConfigurationCommand()
    {}
  };

  /**
   * This class describes the set of commands that can be sent to a rmp
   * Please read the rmp user manual (for instance, page 53 of the RMP 440LE manual)
   */
  class RmpConfigurationCommandSet
  {
  public:
    const static RmpConfigurationCommand<uint32_t> NONE;
    const static RmpConfigurationCommand<float> SET_MAXIMUM_VELOCITY;
    const static RmpConfigurationCommand<float> SET_MAXIMUM_ACCELERATION;
    const static RmpConfigurationCommand<float> SET_MAXIMUM_DECELERATION;
    const static RmpConfigurationCommand<float> SET_MAXIMUM_DTZ_DECEL_RATE;
    const static RmpConfigurationCommand<float> SET_COASTDOWN_ACCEL;
    const static RmpConfigurationCommand<float> SET_MAXIMUM_TURN_RATE;
    const static RmpConfigurationCommand<float> SET_MAXIMUM_TURN_ACCEL;
    const static RmpConfigurationCommand<float> SET_TIRE_DIAMETER;
    const static RmpConfigurationCommand<float> SET_WHEEL_BASE_LENGTH;
    const static RmpConfigurationCommand<float> SET_WHEEL_TRACK_WIDTH;
    const static RmpConfigurationCommand<float> SET_TRANSMISSION_RATIO;
    const static RmpConfigurationCommand<uint32_t> SET_INPUT_CONFIG_BITMAP;
    const static RmpConfigurationCommand<uint32_t> SET_ETH_IP_ADDRESS;
    const static RmpConfigurationCommand<uint32_t> SET_ETH_PORT_ADDRESS;
    const static RmpConfigurationCommand<uint32_t> SET_ETH_SUBNET_MASK;
    const static RmpConfigurationCommand<uint32_t> SET_ETH_GATEWAY;
    const static RmpConfigurationCommand<uint32_t> SET_USER_FB_1_BITMAP;
    const static RmpConfigurationCommand<uint32_t> SET_USER_FB_2_BITMAP;
    const static RmpConfigurationCommand<uint32_t> SET_USER_FB_3_BITMAP;
    const static RmpConfigurationCommand<uint32_t> SET_USER_FB_4_BITMAP;
    const static RmpConfigurationCommand<uint32_t> FORCE_CONFIG_FEEDBACK_BITMAPS;
    const static RmpConfigurationCommand<uint32_t> SET_AUDIO_COMMAND;
    const static RmpConfigurationCommand<uint32_t> SET_OPERATIONAL_MODE;
    const static RmpConfigurationCommand<uint32_t> SEND_SP_FAULT_LOG;
    const static RmpConfigurationCommand<uint32_t> RESET_INTEGRATORS;
    const static RmpConfigurationCommand<uint32_t> RESET_PARAMS_TO_DEFAULT;
  };
  
} // namespace segway

#endif // RMP_CONFIGURATION_COMMAND_H

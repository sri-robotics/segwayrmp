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

#include <RmpConfigurationCommand.h>

/**
 * Please read the rmp user manual (for instance, page 53 of the RMP 440LE manual)
 */
namespace segway
{
  const RmpConfigurationCommand<uint32_t> RmpConfigurationCommandSet::NONE(CONFIG_CMD_ID_NONE, 0, 0, NO_FEEDBACK);
  const RmpConfigurationCommand<float> RmpConfigurationCommandSet::SET_MAXIMUM_VELOCITY(CONFIG_CMD_ID_SET_MAXIMUM_VELOCITY, 0.0, 8.047, FRAM_VEL_LIMIT_MPS);
  const RmpConfigurationCommand<float> RmpConfigurationCommandSet::SET_MAXIMUM_ACCELERATION(CONFIG_CMD_ID_SET_MAXIMUM_ACCELERATION, 0.0, 7.848, FRAM_ACCEL_LIMIT_MPS2);
  const RmpConfigurationCommand<float> RmpConfigurationCommandSet::SET_MAXIMUM_DECELERATION(CONFIG_CMD_ID_SET_MAXIMUM_DECELERATION, 0.0, 7.848, FRAM_DECEL_LIMIT_MPS2);
  const RmpConfigurationCommand<float> RmpConfigurationCommandSet::SET_MAXIMUM_DTZ_DECEL_RATE(CONFIG_CMD_ID_SET_DTZ_DECEL_RATE, 0.0, 7.848, FRAM_DTZ_DECEL_LIMIT_MPS2);
  const RmpConfigurationCommand<float> RmpConfigurationCommandSet::SET_COASTDOWN_ACCEL(CONFIG_CMD_ID_SET_COASTDOAN_ACCEL, 0.0, 1.961, FRAM_COASTDOWN_DECEL_MPS2);
  const RmpConfigurationCommand<float> RmpConfigurationCommandSet::SET_MAXIMUM_TURN_RATE(CONFIG_CMD_ID_SET_MAXIMUM_TURN_RATE, 0.0, 4.5, FRAM_YAW_RATE_LIMIT_RPS);
  const RmpConfigurationCommand<float> RmpConfigurationCommandSet::SET_MAXIMUM_TURN_ACCEL(CONFIG_CMD_ID_SET_MAXIMUM_TURN_ACCEL, 0.0, 28.274, FRAM_YAW_ACCEL_LIMIT_RPS2);
  const RmpConfigurationCommand<float> RmpConfigurationCommandSet::SET_TIRE_DIAMETER(CONFIG_CMD_ID_SET_TIRE_DIAMETER, 0.3556, 1.0, FRAM_TIRE_DIAMETER_M);
  const RmpConfigurationCommand<float> RmpConfigurationCommandSet::SET_WHEEL_BASE_LENGTH(CONFIG_CMD_ID_SET_WHEEL_BASE_LENGTH, 0.4142, 1.0, FRAM_WHEEL_BASE_LENGTH_M);
  const RmpConfigurationCommand<float> RmpConfigurationCommandSet::SET_WHEEL_TRACK_WIDTH(CONFIG_CMD_ID_SET_WHEEL_TRACK_WIDTH, 0.506476, 1.0, FRAM_WHEEL_TRACK_WIDTH_M);
  const RmpConfigurationCommand<float> RmpConfigurationCommandSet::SET_TRANSMISSION_RATIO(CONFIG_CMD_ID_SET_TRANSMISSION_RATIO, 1.0, 200.0, FRAM_TRANSMISSION_RATIO);
  const RmpConfigurationCommand<uint32_t> RmpConfigurationCommandSet::SET_INPUT_CONFIG_BITMAP(CONFIG_CMD_ID_SET_INPUT_CONFIG_BITMAP, 0x00000000, 0x0000000F, FRAM_CONFIG_BITMAP);
  const RmpConfigurationCommand<uint32_t> RmpConfigurationCommandSet::SET_ETH_IP_ADDRESS(CONFIG_CMD_ID_SET_ETH_IP_ADDRESS, 0x00000000, 0xFFFFFFFF, FRAM_ETH_IP_ADDRESS);
  const RmpConfigurationCommand<uint32_t> RmpConfigurationCommandSet::SET_ETH_PORT_ADDRESS(CONFIG_CMD_ID_SET_ETH_PORT_NUMBER, 0x00000000, 0x0000FFFF, FRAM_ETH_PORT_NUMBER);
  const RmpConfigurationCommand<uint32_t> RmpConfigurationCommandSet::SET_ETH_SUBNET_MASK(CONFIG_CMD_ID_SET_ETH_SUBNET_MASK, 0x00000000, 0xFFFFFFFF, FRAM_ETH_SUBNET_MASK);
  const RmpConfigurationCommand<uint32_t> RmpConfigurationCommandSet::SET_ETH_GATEWAY(CONFIG_CMD_ID_SET_GATEWAY, 0x00000000, 0xFFFFFFFF, FRAM_ETH_GATEWAY);
  const RmpConfigurationCommand<uint32_t> RmpConfigurationCommandSet::SET_USER_FB_1_BITMAP(CONFIG_CMD_ID_SET_USER_FB_1_BITMAP, 0x00000000, 0xFFFFFFFF, INVALID_FEEDBACK);
  const RmpConfigurationCommand<uint32_t> RmpConfigurationCommandSet::SET_USER_FB_2_BITMAP(CONFIG_CMD_ID_SET_USER_FB_2_BITMAP, 0x00000000, 0xFFFFFFFF, INVALID_FEEDBACK);
  const RmpConfigurationCommand<uint32_t> RmpConfigurationCommandSet::SET_USER_FB_3_BITMAP(CONFIG_CMD_ID_SET_USER_FB_3_BITMAP, 0x00000000, 0xFFFFFFFF, INVALID_FEEDBACK);
  const RmpConfigurationCommand<uint32_t> RmpConfigurationCommandSet::SET_USER_FB_4_BITMAP(CONFIG_CMD_ID_SET_USER_FB_4_BITMAP, 0x00000000, 0x00000000, INVALID_FEEDBACK);
  const RmpConfigurationCommand<uint32_t> RmpConfigurationCommandSet::FORCE_CONFIG_FEEDBACK_BITMAPS(CONFIG_CMD_ID_FORCE_CONFIG_FEEDBACK_BITMAPS, 0, 1, INVALID_FEEDBACK);
  const RmpConfigurationCommand<uint32_t> RmpConfigurationCommandSet::SET_AUDIO_COMMAND(CONFIG_CMD_ID_SET_AUDIO_COMMAND, 0, 16, NO_FEEDBACK);
  const RmpConfigurationCommand<uint32_t> RmpConfigurationCommandSet::SET_OPERATIONAL_MODE(CONFIG_CMD_ID_SET_OPERATIONAL_MODE, 1, 5, INVALID_FEEDBACK);
  const RmpConfigurationCommand<uint32_t> RmpConfigurationCommandSet::SEND_SP_FAULT_LOG(CONFIG_CMD_ID_SEND_SP_FAULT_LOG, 0, 1, INVALID_FEEDBACK);
  const RmpConfigurationCommand<uint32_t> RmpConfigurationCommandSet::RESET_INTEGRATORS(CONFIG_CMD_ID_RESET_INTEGRATORS, 0x00000000, 0x0000001F, INVALID_FEEDBACK);
  const RmpConfigurationCommand<uint32_t> RmpConfigurationCommandSet::RESET_PARAMS_TO_DEFAULT(CONFIG_CMD_ID_RESET_PARAMS_TO_DEFAULT, 0, 0, INVALID_FEEDBACK);
} // namespace segway

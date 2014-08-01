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

#ifndef RMP_USER_DEFINED_FEEDBACK_TYPE_H
#define RMP_USER_DEFINED_FEEDBACK_TYPE_H

#include <cstring>
#include <cstdint>

namespace segway 
{
  static const size_t FEEDBACK_TYPE_NBR = 4;
  static const size_t MAX_ENTRIES_PER_FEEDBACK = 32;
  static const size_t MAX_FEEDBACK_ENTRIES = FEEDBACK_TYPE_NBR * MAX_ENTRIES_PER_FEEDBACK;

  static const size_t NVM_CONFIG_PARAM_NBR = 20;
  static const size_t NVM_CONFIG_PARAM_SIZE = NVM_CONFIG_PARAM_NBR * sizeof(uint32_t);

  static const size_t FORCE_CONFIG_FEEDBACK_1_IDX = 16;
  static const size_t FORCE_CONFIG_FEEDBACK_2_IDX = 17;
  static const size_t FORCE_CONFIG_FEEDBACK_3_IDX = 18;
  static const size_t FORCE_CONFIG_FEEDBACK_4_IDX = 19;

  static const uint32_t PSE1_VALID = 0x00000001;
  static const uint32_t PSE2_VALID = 0x00000002;
  static const uint32_t PSE_VALID = PSE1_VALID | PSE2_VALID;

  /**
   * Define User Defined Feedback Bitmap Types
   * Please read the rmp user manual (for instance, page 53 of the RMP 440LE manual)
   */
  enum UserDefinedFeedbackBitmapType
  {
    USER_DEFINED_FEEDBACK_BITMAP_1 = 0,
    USER_DEFINED_FEEDBACK_BITMAP_2,
    USER_DEFINED_FEEDBACK_BITMAP_3,
    USER_DEFINED_FEEDBACK_BITMAP_4
  };

  /**
   * Define Operational States
   * Please read the rmp user manual (for instance, page 61 of the RMP 440LE manual)
   */
  enum OperationalState
  {
    CCU_INIT = 0,
    INIT_PROPULSION,
    CHECK_STARTUP_ISSUES,
    STANDBY_MODE,
    TRACTOR_MODE,
    DISABLE_POWER
  };

  /**
   * Define User Defined Feedback Types
   * Please read the rmp user manual (for instance, page 61 of the RMP 440LE manual)
   */
  enum UserDefinedFeedbackType
  {
    NO_FEEDBACK = 0xFFFFFFFF,
    INVALID_FEEDBACK = 0xFFFFFFFE,
    // Bitmap 1
    FAULT_STATUS_WORD_1 = 0,
    FAULT_STATUS_WORD_2,
    FAULT_STATUS_WORD_3,
    FAULT_STATUS_WORD_4,
    MCU_0_FAULT_STATUS,
    MCU_1_FAULT_STATUS,
    MCU_2_FAULT_STATUS,
    MCU_3_FAULT_STATUS,
    FRAME_COUNT,
    OPERATIONAL_STATE,
    DYNAMIC_RESPONSE,
    MIN_PROPULSION_BATT_SOC,
    AUX_BATT_SOC,
    INERTIAL_X_ACC_G,
    INERTIAL_Y_ACC_G,
    INERTIAL_X_RATE_RPS,
    INERTIAL_Y_RATE_RPS,
    INERTIAL_Z_RATE_RPS,
    PSE_PITCH_DEG,
    PSE_PITCH_RATE_DPS,
    PSE_ROLL_DEG,
    PSE_ROLL_RATE_DPS,
    PSE_YAW_RATE_DPS,
    PSE_DATA_IS_VALID,
    YAW_RATE_LIMIT_RPS,
    VEL_LIMIT_MPS,
    LINEAR_ACCEL_MSP2,
    LINEAR_VEL_MPS,
    DIFFERENTIAL_WHEEL_VEL_RPS,
    RIGHT_FRONT_VEL_MPS,
    LEFT_FRONT_VEL_MPS,
    RIGHT_REAR_VEL_MPS,
    // Bitmap 2
    LEFT_REAR_VEL_MPS, // = 32
    RIGHT_FRONT_POS_M,
    LEFT_FRONT_POS_M,
    RIGHT_REAR_POS_M,
    LEFT_REAR_POS_M,
    LINEAR_POS_M,
    RIGHT_FRONT_CURRENT_A0PK,
    LEFT_FRONT_CURRENT_A0PK,
    RIGHT_REAR_CURRENT_A0PK,
    LEFT_REAR_CURRENT_A0PK,
    MAX_MOTOR_CURRENT_A0PK,
    RIGHT_FRONT_CURRENT_LIMIT_A0PK,
    LEFT_FRONT_CURRENT_LIMIT_A0PK,
    RIGHT_REAR_CURRENT_LIMIT_A0PK,
    LEFT_REAR_CURRENT_LIMIT_A0PK,
    MIN_MOTOR_CURRENT_LIMIT_A0PK,
    FRONT_BASE_BATT_1_SOC,
    FRONT_BASE_BATT_2_SOC,
    REAR_BASE_BATT_1_SOC,
    REAR_BASE_BATT_2_SOC,
    FRONT_BASE_BATT_1_TEMP_DEGC,
    FRONT_BASE_BATT_2_TEMP_DEGC,
    REAR_BASE_BATT_1_TEMP_DEGC,
    REAR_BASE_BATT_2_TEMP_DEGC,
    VEL_TARGET_MPS,
    YAW_RATE_TARGET_RPS,
    ANGLE_TARGET_DEG,
    AUX_BATT_VOLTAGE_V,
    AUX_BATT_CURRENT_A,
    AUX_BATT_TEMP_DEGC,
    ABB_SYSTEM_STATUS,
    AUX_BATT_STATUS,
    // Bitmap 3
    AUX_BATT_FAULTS, // = 64
    P72V_BATTERY_VOLTAGE,
    SP_SW_BUILD_ID,
    UIP_SW_BUILD_ID,
    MCU_0_INST_POWER_W,
    MCU_1_INST_POWER_W,
    MCU_2_INST_POWER_W,
    MCU_3_INST_POWER_W,
    MCU_0_TOTAL_ENERGY_WH,
    MCU_1_TOTAL_ENERGY_WH,
    MCU_2_TOTAL_ENERGY_WH,
    MCU_3_TOTAL_ENERGY_WH,
    FRAM_VEL_LIMIT_MPS,
    FRAM_ACCEL_LIMIT_MPS2,
    FRAM_DECEL_LIMIT_MPS2,
    FRAM_DTZ_DECEL_LIMIT_MPS2,
    FRAM_COASTDOWN_DECEL_MPS2,
    FRAM_YAW_RATE_LIMIT_RPS,
    FRAM_YAW_ACCEL_LIMIT_RPS2,
    FRAM_TIRE_DIAMETER_M,
    FRAM_WHEEL_BASE_LENGTH_M,
    FRAM_WHEEL_TRACK_WIDTH_M,
    FRAM_TRANSMISSION_RATIO,
    FRAM_CONFIG_BITMAP,
    FRAM_ETH_IP_ADDRESS,
    FRAM_ETH_PORT_NUMBER,
    FRAM_ETH_SUBNET_MASK,
    FRAM_ETH_GATEWAY,
    USER_FEEDBACK_BITMAP_1,
    USER_FEEDBACK_BITMAP_2,
    USER_FEEDBACK_BITMAP_3,
    USER_FEEDBACK_BITMAP_4
    // Bitmap 4
  };
} // namespace segway

#endif // RMP_USER_DEFINED_FEEDBACK_TYPE_H

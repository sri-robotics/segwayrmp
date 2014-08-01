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

#include <RmpFault.h>

#include <cstdarg>
#include <stdexcept>
#include <vector>

namespace segway
{
  void InitializeFaultDecodeList(FaultDecodeList& rDecodeList, unsigned int argumentCount, ...)
  {
    rDecodeList.clear();

    va_list argumentList;
    va_start(argumentList, argumentCount);

    for (unsigned int i = 0; i < argumentCount; ++i)
    {
      std::string argument(va_arg(argumentList, const char*));
      rDecodeList.push_back(argument);
    }

    va_end(argumentList);
  }

  const uint32_t& FaultPacking::GetShift() const
  {
    return m_Shift;
  }

  const uint32_t& FaultPacking::GetMask() const
  {
    return m_Mask;
  }

  const FaultGroup& FaultPacking::GetGroup() const
  {
    return m_Group;
  }
  
  struct RmpFault::Impl
  {
    static Impl& Instance()
    {
      static Impl instance;

      return instance;
    }

    Impl();

    ~Impl();

    FeedbackTypeList m_FeedbackTypeList;
    std::vector<FaultPackingList> m_FaultPackingTable;
    std::vector<FaultDecodeList>  m_FaultDecodeTable;
  };

  const FeedbackTypeList& RmpFault::GetFaultFeedbackEntryList()
  {
    return Impl::Instance().m_FeedbackTypeList;
  }

  const FaultPackingList& RmpFault::GetFaultPackingList(UserDefinedFeedbackType feedbackType)
  {
    return Impl::Instance().m_FaultPackingTable[feedbackType];
  }

  const FaultDecodeList& RmpFault::GetFaultDecodeList(FaultGroup faultGroup)
  {
    return Impl::Instance().m_FaultDecodeTable[faultGroup];
  }

  RmpFault::Impl::Impl()
  {
    m_FeedbackTypeList.push_back(FAULT_STATUS_WORD_1);
    m_FeedbackTypeList.push_back(FAULT_STATUS_WORD_2);
    m_FeedbackTypeList.push_back(FAULT_STATUS_WORD_3);
    m_FeedbackTypeList.push_back(FAULT_STATUS_WORD_4);

    FaultPackingList word0Packing;
    word0Packing.push_back(FaultPacking(0, 0x00000FFF, ARCHITECTURE));
    word0Packing.push_back(FaultPacking(12, 0xFFFFF000, CRITICAL));

    FaultPackingList word1Packing;
    word1Packing.push_back(FaultPacking(0, 0x0000FFFF, COMM));
    word1Packing.push_back(FaultPacking(16, 0x000F0000, INTERNAL));

    FaultPackingList word2Packing;
    word2Packing.push_back(FaultPacking(0, 0x0000FFFF, SENSORS));
    word2Packing.push_back(FaultPacking(16, 0xFFFF0000, BSA));

    FaultPackingList word3Packing;
    word3Packing.push_back(FaultPacking(0, 0xFFFFFFFF, MCU));

    m_FaultPackingTable.push_back(word0Packing);
    m_FaultPackingTable.push_back(word1Packing);
    m_FaultPackingTable.push_back(word2Packing);
    m_FaultPackingTable.push_back(word3Packing);

    FaultDecodeList transientFaultList;

    FaultDecodeList architectureFaultList;
    InitializeFaultDecodeList(architectureFaultList, 12,
                              "ARCHITECT_FAULT_SPI_RECEIVE",
                              "ARCHITECT_FAULT_SPI_TRANSMIT",
                              "ARCHITECT_FAULT_SPI_RECEIVE_OVERRUN",
                              "ARCHITECT_FAULT_SPI_RX_BUFFER_OBVERRUN",
                              "ARCHITECT_FAULT_COMMANDED_SAFETY_SHUTDOWN",
                              "ARCHITECT_FAULT_COMMANDED_DISABLE",
                              "ARCHITECT_FAULT_KILL_SWITCH_ACTIVE",
                              "ARCHITECT_FAULT_FRAM_CONFIG_INIT_FAILED",
                              "ARCHITECT_FAULT_FRAM_CONFIG_SET_FAILED",
                              "ARCHITECT_FAULT_BAD_MODEL_IDENTIFIER",
                              "ARCHITECT_FAULT_BAD_CCU_HW_REV",
                              "ARCHITECT_FAULT_DECEL_SWITCH_ACTIVE");

    FaultDecodeList criticalFaultList;
    InitializeFaultDecodeList(criticalFaultList, 13,
                              "CRITICAL_FAULT_INIT",
                              "CRITICAL_FAULT_INIT_UIP_COMM",
                              "CRITICAL_FAULT_INIT_PROPULSION",
                              "CRITICAL_FAULT_INIT_TIMEOUT",
                              "CRITICAL_FAULT_FORW_SPEED_LIMITER_HAZARD",
                              "CRITICAL_FAULT_AFT_SPEED_LIMITER_HAZARD",
                              "CRITICAL_FAULT_CHECK_STARTUP",
                              "CRITICAL_FAULT_APP_VELOCITY_CTL_FAILED",
                              "CRITICAL_FAULT_APP_POSITION_CTL_FAILED",
                              "CRITICAL_FAULT_ABB_SHUTDOWN",
                              "CRITICAL_FAULT_AP_MODE_TRANS_TIMEOUT",
                              "CRITICAL_FAULT_PITCH_ANGLE_EXCEEDED",
                              "CRITICAL_FAULT_ROLL_ANGLE_EXCEEDED");

    FaultDecodeList commFaultList;
    InitializeFaultDecodeList(commFaultList, 7,
                              "COMM_FAULT_UIP_MISSING_UIP_DATA",
                              "COMM_FAULT_UIP_UNKOWN_MESSAGE_RECEIVER",
                              "COMM_FAULT_UIP_BAD_CHECKSUM",
                              "COMM_FAULT_UIP_TRANSMIT",
                              "COMM_FAULT_UI_BAD_MOTION_CMD",
                              "COMM_FAULT_UI_UNKNOWN_CMD",
                              "COMM_FAULT_UI_BAD_PACKET_CHECKSUM");

    FaultDecodeList internalFaultList;
    InitializeFaultDecodeList(internalFaultList, 2,
                              "INTERNAL_FAULT_HIT_DEFAULT_CONDITION",
                              "INTERNAL_FAULT_HIT_SPECIAL_CASE");

    FaultDecodeList sensorFaultList;
    InitializeFaultDecodeList(sensorFaultList, 10,
                              "SENSOR_FAULT_2P5V_VREF_RANGE_FAULT",
                              "SENSOR_FAULT_7P2V_VBAT_RANGE_FAULT",
                              "SENSOR_FAULT_7P2V_VBAT_WARNING",
                              "SENSOR_FAULT_7P2V_BAT_INBALANCE_FAULT",
                              "SENSOR_FAULT_7P2V_BAT_TEMPARATURE_FAULT",
                              "SENSOR_FAULT_DIGITAL_INPUT",
                              "SENSOR_FAULT_RANGE",
                              "SENSOR_FAULT_DEFAULT",
                              "SENSOR_FAULT_5V_MONITOR_RANGE_FAULT",
                              "SENSOR_FAULT_12V_MONITOR_RANGE_FAULT");
    

    FaultDecodeList bsaFaultList;
    InitializeFaultDecodeList(bsaFaultList, 11,
                              "BSA_FAULT_SIDE_A_MISSING_BSA_DATA",
                              "BSA_FAULT_SIDE_B_MISSING_BSA_DATA",
                              "BSA_FAULT_UNKNOWN_MESSAGE_RECEIVED",
                              "BSA_FAULT_TRANSMIT_A_FAILED",
                              "BSA_FAULT_TRANSMIT_B_FAILED",
                              "BSA_FAULT_DEFAULT",
                              "BSA_FAULT_SIDE_A_RATE_SENSOR_SATURATED",
                              "BSA_FAULT_SIDE_B_RATE_SENSOR_SATURATED",
                              "BSA_FAULT_SIDE_A_TILT_SENSOR_SATURATED",
                              "BSA_FAULT_SIDE_B_TILT_SENSOR_SATURATED",
                              "PSE_FAULT_COMPARISON");

    FaultDecodeList mcuFaultList;
    InitializeFaultDecodeList(mcuFaultList, 21,
                              "MCU_FAULT_MCU_0_IS_DEGRADED",
                              "MCU_FAULT_MCU_0_IS_FAILED",
                              "MCU_FAULT_MCU_0_REQUESTS_REDUCE_PERFORMANCE",
                              "MCU_FAULT_MCU_0_REQUESTS_ZERO_SPEED",
                              "MCU_FAULT_MCU_1_IS_DEGRADED",
                              "MCU_FAULT_MCU_1_IS_FAILED",
                              "MCU_FAULT_MCU_1_REQUESTS_REDUCE_PERFORMANCE",
                              "MCU_FAULT_MCU_1_REQUESTS_ZERO_SPEED",
                              "MCU_FAULT_MCU_2_IS_DEGRADED",
                              "MCU_FAULT_MCU_2_IS_FAILED",
                              "MCU_FAULT_MCU_2_REQUESTS_REDUCE_PERFORMANCE",
                              "MCU_FAULT_MCU_2_REQUESTS_ZERO_SPEED",
                              "MCU_FAULT_MCU_3_IS_DEGRADED",
                              "MCU_FAULT_MCU_3_IS_FAILED",
                              "MCU_FAULT_MCU_3_REQUESTS_REDUCE_PERFORMANCE",
                              "MCU_FAULT_MCU_3_REQUESTS_ZERO_SPEED",
                              "MCU_FAULT_MISSING_MCU_0_DATA",
                              "MCU_FAULT_MISSING_MCU_1_DATA",
                              "MCU_FAULT_MISSING_MCU_2_DATA",
                              "MCU_FAULT_MISSING_MCU_3_DATA",
                              "MCU_FAULT_UNKNOWN_MESSAGE_RECEIVED");

    // must match fault group definition
    m_FaultDecodeTable.push_back(transientFaultList);
    m_FaultDecodeTable.push_back(criticalFaultList);
    m_FaultDecodeTable.push_back(commFaultList);
    m_FaultDecodeTable.push_back(sensorFaultList);
    m_FaultDecodeTable.push_back(bsaFaultList);
    m_FaultDecodeTable.push_back(mcuFaultList);
    m_FaultDecodeTable.push_back(architectureFaultList);
    m_FaultDecodeTable.push_back(internalFaultList);
  }

  RmpFault::Impl::~Impl()
  {}
  
} // namespace segway

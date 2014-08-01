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

#include <RmpInterface.h>

#include <stdexcept>
#include <iostream>
#include <thread>
#include <chrono>

#include <RmpMessage.h>
#include <RmpType.h>
#include <RmpHelper.h>
#include <RmpUdp.h>
#include <RmpUsb.h>
#include <RmpConfigurationCommand.h>
#include <RmpLogger.h>

namespace segway 
{
  static const size_t MAX_TRIAL_NBR = 7;
  
  RmpInterface::RmpInterface()
    : m_IsFeedbackBitmapSet(false)
  {
    Logger::SetMinLogLevel(INFO);
    Logger::AddLogFile(std::string("rmp_interface.log"));
    Logger::AddOutputStream(std::cout);
  }

  RmpInterface::~RmpInterface()
  {
    ShutDown();
  }

  void RmpInterface::Initialize(const std::string& rIpAddress, uint16_t portNumber)
  {
    if (m_pTransport != nullptr)
    {
      throw std::logic_error(std::string("Rmp already initialized."));
    }
    
    m_pTransport = RmpTransport::Ptr(new RmpUdp(rIpAddress, portNumber));

    Initialize();
  }

  void RmpInterface::Initialize(const std::string& rDevicePort)
  {
    if (m_pTransport != nullptr)
    {
      throw std::logic_error(std::string("Rmp already initialized."));
    }
    
    m_pTransport = RmpTransport::Ptr(new RmpUsb(rDevicePort));
    
    Initialize();
  }

  void RmpInterface::ShutDown()
  {
    if (m_pTransport != nullptr)
    {
      ChangeOperationalMode(STANDBY_REQUEST);
      m_pTransport.reset();
    }
  }

  template<typename T>
  bool RmpInterface::SetConfiguration(const RmpConfigurationCommand<T>& rConfigurationCommand, T value)
  {
    throw std::logic_error(std::string("Method not implemented for this template."));

    return false;
  }

  template<>
  bool RmpInterface::SetConfiguration(const RmpConfigurationCommand<uint32_t>& rConfigurationCommand, uint32_t value)
  {
    if ((value < rConfigurationCommand.m_Min) && (value > rConfigurationCommand.m_Max))
    {
      SEGWAY_LOG(ERROR, "Try to set configuration with invalid value: " << value << " should be between " << rConfigurationCommand.m_Min << " and " << rConfigurationCommand.m_Max);

      return false;
    }

    if (m_pTransport == nullptr)
    {
      throw std::runtime_error(std::string("Initialize communication before calling any methods."));
    }

    return SetConfiguration(static_cast<uint16_t>(rConfigurationCommand.m_CommandId), rConfigurationCommand.m_FeedbackType, value);
  }

  template<>
  bool RmpInterface::SetConfiguration(const RmpConfigurationCommand<float>& rConfigurationCommand, float value)
  {
    if ((value < rConfigurationCommand.m_Min) && (value > rConfigurationCommand.m_Max))
    {
      SEGWAY_LOG(ERROR, "Try to set configuration with invalid value: " << value << " should be between " << rConfigurationCommand.m_Min << " and " << rConfigurationCommand.m_Max);

      return false;
    }

    if (m_pTransport == nullptr)
    {
      throw std::runtime_error(std::string("Initialize communication before calling any methods."));
    }

    return SetConfiguration(static_cast<uint16_t>(rConfigurationCommand.m_CommandId), rConfigurationCommand.m_FeedbackType, ConvertFloatToUint32(value));
  }

  bool RmpInterface::Update(bool forceUpdate)
  {
    if (m_pTransport == nullptr)
    {
      throw std::runtime_error(std::string("Initialize communication before calling any methods."));
    }
    
    if (!m_IsFeedbackBitmapSet)
    {
      SEGWAY_LOG(ERROR, "Unable to update. Bitmap is not set.");
      
      return false;
    }

    bool result = false;
    size_t expectedUserFeedbackSize = GetFeedbackEntryCount() * sizeof(uint32_t);
    uint32_t attempt = 0;
    Bytes bytes;
    
    while (!result && (attempt < MAX_TRIAL_NBR))
    {
      result = m_pTransport->Receive(bytes, expectedUserFeedbackSize);

      if (!result && forceUpdate)
      {
        SetConfiguration(RmpConfigurationCommandSet::NONE, static_cast<uint32_t>(0));
      }

      ++attempt;
    }

    if (result)
    {
      UpdateUserDefinedFeedback(bytes);
    }

    return result;
  }

  template<typename T>
  T RmpInterface::GetUserDefinedFeedback(UserDefinedFeedbackType type)
  {
    throw std::runtime_error(std::string("Method not implemented for this template."));
        
    return static_cast<T>(0);
  }
  
  template<>
  uint32_t RmpInterface::GetUserDefinedFeedback(UserDefinedFeedbackType type)
  {
    if (!m_IsFeedbackBitmapSet)
    {
      throw std::logic_error(std::string("Unable to retrieve user defined feedback. Bitmap is not set."));
    }
    
    size_t feedbackType = type / MAX_ENTRIES_PER_FEEDBACK;
    size_t entry = type % MAX_ENTRIES_PER_FEEDBACK;

    uint32_t mask = 1 << entry;
    
    if (!(m_FeedbackBitmap[feedbackType] & mask))
    {
      std::stringstream stringStream;
      stringStream << "Unable to retrieve user defined feedback. Feedback is not set for type: " << type;
      throw std::runtime_error(stringStream.str());
    }

    return m_UserDefinedFeedback[feedbackType][entry];
  }

  template<>
  float RmpInterface::GetUserDefinedFeedback(UserDefinedFeedbackType type)
  {
    return ConvertUint32ToFloat(GetUserDefinedFeedback<uint32_t>(type));
  }

  bool RmpInterface::ResetIntegrators(uint32_t bitmap)
  {
    if (bitmap > RESET_ALL_POSITION_DATA)
    {
      throw std::logic_error(std::string("Invalid reset integrator bitmap."));
    }

    if (m_pTransport == nullptr)
    {
      throw std::runtime_error(std::string("Initialize communication before calling any methods."));
    }

    bool result = false;
    size_t attempt = 0;

    while (!result && (attempt < MAX_TRIAL_NBR))
    {
      result = SendConfigurationCommand(CONFIG_CMD_ID_RESET_INTEGRATORS, bitmap);

      if (result)
      {
        result = Update(true);

        if (result)
        {
          if (bitmap & RESET_LINEAR_POSITION)
          { 
            if (GetUserDefinedFeedback<float>(LINEAR_POS_M) != 0)
            {
              result = false;
            }
          }
          
          if (bitmap & RESET_RIGHT_FRONT_POSITION)
          {
            if (GetUserDefinedFeedback<float>(RIGHT_FRONT_POS_M) != 0)
            {
              result = false;
            }
          }
          
          if (bitmap & RESET_LEFT_FRONT_POSITION)
          {
            if (GetUserDefinedFeedback<float>(LEFT_FRONT_POS_M) != 0)
            {
              result = false;
            }
          }
          
          if (bitmap & RESET_RIGHT_REAR_POSITION)
          {
            if (GetUserDefinedFeedback<float>(RIGHT_REAR_POS_M) != 0)
            {
              result = false;
            }
          }

          if (bitmap & RESET_LEFT_REAR_POSITION)
          {
            if (GetUserDefinedFeedback<float>(LEFT_REAR_POS_M) != 0)
            {
              result = false;
            }
          }
        } 
      } // if (result)

      ++attempt;
    } // while (...)

    return result;
  }

  bool RmpInterface::ResetParamsToDefault()
  {
    if (m_pTransport == nullptr)
    {
      throw std::runtime_error(std::string("Initialize communication before calling any methods."));
    }

    bool result = false;
    size_t attempt = 0;

    while (!result && (attempt < MAX_TRIAL_NBR))
    {
      result = SendConfigurationCommand(static_cast<uint16_t>(CONFIG_CMD_ID_RESET_PARAMS_TO_DEFAULT), 0);

      if (result)
      {
        result = Update(true);
      }

      ++attempt;
    } // while (...)

    if (result)
    {
      InitializeFeedbackBitmaps();
    }

    return result;
  }

  bool RmpInterface::SetUserDefinedFeedbackBitmap(UserDefinedFeedbackBitmapType bitmapType, const uint32_t bitmap)
  {
    if (m_pTransport == nullptr)
    {
      throw std::runtime_error(std::string("Initialize communication before calling any methods."));
    }
    
    RmpConfigurationCommandId configurationCommandId;
    bool isValid = false;
    
    switch (bitmapType)
    {
      case USER_DEFINED_FEEDBACK_BITMAP_1:
      {
        configurationCommandId = CONFIG_CMD_ID_SET_USER_FB_1_BITMAP;
        isValid = IsValidFeedbackBitmap(RmpConfigurationCommandSet::SET_USER_FB_1_BITMAP, bitmap);
        
        break;
      }
      case USER_DEFINED_FEEDBACK_BITMAP_2:
      {
        configurationCommandId = CONFIG_CMD_ID_SET_USER_FB_2_BITMAP;
        isValid = IsValidFeedbackBitmap(RmpConfigurationCommandSet::SET_USER_FB_2_BITMAP, bitmap);
        
        break;
      }
      case USER_DEFINED_FEEDBACK_BITMAP_3:
      {
        configurationCommandId = CONFIG_CMD_ID_SET_USER_FB_3_BITMAP;
        isValid = IsValidFeedbackBitmap(RmpConfigurationCommandSet::SET_USER_FB_3_BITMAP, bitmap);
        
        break;
      }
      case USER_DEFINED_FEEDBACK_BITMAP_4:
      {
        configurationCommandId = CONFIG_CMD_ID_SET_USER_FB_4_BITMAP;
        isValid = IsValidFeedbackBitmap(RmpConfigurationCommandSet::SET_USER_FB_4_BITMAP, bitmap);
        
        break;
      }
      default:
      {
        throw std::logic_error(std::string("Invalid feedback bitmap type."));
      }
    }

    if (!isValid)
    {
      throw std::logic_error(std::string("Invalid feedback bitmap."));
    }

    bool result = false;
    size_t attempt = 0;

    while (!result && (attempt < MAX_TRIAL_NBR))
    {
      result = SendConfigurationCommand(static_cast<uint32_t>(configurationCommandId), bitmap);

      if (result)
      {
        result = Update(true);
      }

      ++attempt;
    } // while (...)

    if (result)
    {
      InitializeFeedbackBitmaps();

      if (bitmap != m_FeedbackBitmap[bitmapType])
      {
        result = false;
      }
    }

    return result;
  }

  bool RmpInterface::ChangeOperationalMode(RmpOperationalModeRequest modeRequest)
  {
    if (m_pTransport == nullptr)
    {
      throw std::runtime_error(std::string("Initialize communication before calling any methods."));
    }
    
    switch (modeRequest)
    {
      case POWERDOWN_REQUEST:
      case DTZ_REQUEST:
      {
        bool result = false;
        size_t attempt = 0;

        while (!result && (attempt < MAX_TRIAL_NBR))
        {
          result = SendConfigurationCommand(static_cast<uint16_t>(CONFIG_CMD_ID_SET_OPERATIONAL_MODE), modeRequest);

          ++attempt;
        }

        return result;
      }
      case DISABLE_REQUEST:
      case STANDBY_REQUEST:
      case TRACTOR_REQUEST:
      {
        bool result = false;
        size_t attempt = 0;

        while (!result && (attempt < MAX_TRIAL_NBR))
        {
          result = SendConfigurationCommand(static_cast<uint16_t>(CONFIG_CMD_ID_SET_OPERATIONAL_MODE), modeRequest);

          if (result)
          {
            result = Update(true);
            
            if (result)
            {
              uint32_t currentMode = GetUserDefinedFeedback<uint32_t>(OPERATIONAL_STATE);

              result = false;

              if ( ((modeRequest == TRACTOR_REQUEST) && (currentMode == TRACTOR_MODE)) ||
                   ((modeRequest == STANDBY_REQUEST) && (currentMode == STANDBY_MODE)) ||
                   ((modeRequest == DISABLE_REQUEST) && (currentMode == DISABLE_POWER)) )
              {
                result = true; 
              }
            }
          }

          ++attempt;
        } // while (...)

        return result;
      }
      default:
      {
        throw std::logic_error(std::string("Invalid mode request type."));
      }
    }

    return false;
  }

  void RmpInterface::GetFaultStatusDescription(FaultStatusDescription& rDescription)
  {
    rDescription.clear();

    const FeedbackTypeList& rFeedbackTypeList = RmpFault::GetFaultFeedbackEntryList();
    // w: word
    for (size_t w = 0; w < rFeedbackTypeList.size(); ++w)
    {
      UserDefinedFeedbackType feedbackType = rFeedbackTypeList[w];
      uint32_t faultWord = GetUserDefinedFeedback<uint32_t>(feedbackType);
      const FaultPackingList& rFaultPackingList = RmpFault::GetFaultPackingList(feedbackType);

      // p: fault packing
      for (size_t p = 0; p < rFaultPackingList.size(); ++p)
      {
        const FaultPacking& rFaultPacking = rFaultPackingList[p];

        if ((faultWord & rFaultPacking.GetMask()) != NO_FAULT)
        {
          const FaultDecodeList& rFaultDecodeList = RmpFault::GetFaultDecodeList(rFaultPacking.GetGroup());

          // b: bit
          for (size_t b = 0; b < rFaultDecodeList.size(); ++b)
          {
             uint32_t testBit = 1 << (b + rFaultPacking.GetShift());
          
             if (testBit & faultWord)
             {
               rDescription.push_back(rFaultDecodeList[b]);
             }
          } // b
        }
      } // p
    } // w
  }

  void RmpInterface::Initialize()
  {
    InitializeFeedbackBitmaps();

    bool result = ChangeOperationalMode(TRACTOR_REQUEST);
    if (!result)
    {
      throw std::runtime_error(std::string("Fail to set the rmp in tractor mode."));
    }
  }

  void RmpInterface::InitializeFeedbackBitmaps()
  {
    bool result = false;
    size_t attempt = 0;
    Bytes bytes;

    while (!result && (attempt < MAX_TRIAL_NBR))
    {
      result = SendConfigurationCommand(static_cast<uint16_t>(CONFIG_CMD_ID_FORCE_CONFIG_FEEDBACK_BITMAPS), 1);

      if (result)
      {
        result = m_pTransport->Receive(bytes, NVM_CONFIG_PARAM_SIZE);
      }

      ++attempt;
    }

    if (result)
    {
      m_IsFeedbackBitmapSet = true;
    }
    else
    {
      throw std::runtime_error(std::string("Unable to initialize feedback bitmap"));
    }

    for (size_t i = 0; i < FEEDBACK_TYPE_NBR; ++i)
    {
      m_FeedbackBitmap[i] = ConvertBytesToUint32(&bytes[(FORCE_CONFIG_FEEDBACK_1_IDX + i) * sizeof(uint32_t)], sizeof(uint32_t));

      SEGWAY_LOG(INFO, "Initialize feedback " << i << " with bitmap: " << std::hex << m_FeedbackBitmap[i]);
    }

    result = false;
    size_t expectedUserFeedbackSize = GetFeedbackEntryCount() * sizeof(uint32_t);
    attempt = 0;
    
    while (!result && (attempt < MAX_TRIAL_NBR))
    {
      result = SendConfigurationCommand(static_cast<uint16_t>(CONFIG_CMD_ID_FORCE_CONFIG_FEEDBACK_BITMAPS), 0);

      if (result)
      {
        result = m_pTransport->Receive(bytes, expectedUserFeedbackSize);
      }

      ++attempt;
    }

    if (result)
    {
      UpdateUserDefinedFeedback(bytes);
    }
    else
    {
      throw std::runtime_error(std::string("Unable to stop force config feedback command."));
    }
  }

  bool RmpInterface::SetConfiguration(uint16_t commandId, UserDefinedFeedbackType feedbackType, uint32_t value)
  {
    switch (feedbackType)
    {
      case NO_FEEDBACK:
      {
        bool result = false;
        size_t attempt = 0;

        while (!result && (attempt < MAX_TRIAL_NBR))
        {
          result = SendConfigurationCommand(commandId, value);

          ++attempt;
        }

        return result;
      }
      case INVALID_FEEDBACK:
      {
        throw std::runtime_error(std::string("Invalid use of configuration command, please use dedicated method."));
      }
      default:
      {
        bool result = false;
        size_t attempt = 0;

        while (!result && (attempt < MAX_TRIAL_NBR))
        {
          result = SendConfigurationCommand(commandId, value);

          if (result)
          {
            result = Update(true);

            if (result)
            {
              uint32_t feedback = GetUserDefinedFeedback<uint32_t>(feedbackType);

              if (feedback != value)
              {
                result = false;
              }
            }
          }

          ++attempt;
        }

        return result;
      } // default
    } // switch (...)

    return false;
  }
  
  bool RmpInterface::SendConfigurationCommand(uint16_t commandId, uint32_t value)
  {
    return m_pTransport->Send(CONFIGURATION_CMD, commandId, value);
  }


  size_t RmpInterface::GetFeedbackEntryCount()
  {
    size_t count = 0;
    
    for (size_t f = 0; f < FEEDBACK_TYPE_NBR; ++f)
    {
      for (size_t b = 0; b < MAX_ENTRIES_PER_FEEDBACK; ++b)
      {
        uint32_t mask = 1 << b;
        if (m_FeedbackBitmap[f] & mask)
        {
          ++count;
        }
      }
    }

    return count;
  }

  void RmpInterface::UpdateUserDefinedFeedback(const Bytes& rBytes)
  {
    size_t idx = 0;

    for (size_t f = 0; f < FEEDBACK_TYPE_NBR; ++f)
    {
      for (size_t b = 0; b < MAX_ENTRIES_PER_FEEDBACK; ++b)
      {
        uint32_t mask = 1 << b;
        if (m_FeedbackBitmap[f] & mask)
        {
          assert(idx * sizeof(uint32_t) < rBytes.size());

          m_UserDefinedFeedback[f][b] = ConvertBytesToUint32(&(rBytes[idx * sizeof(uint32_t)]), sizeof(uint32_t));
          ++idx;
        }
      }
    }
  }

   bool RmpInterface::IsValidFeedbackBitmap(const RmpConfigurationCommand<uint32_t>& rConfigurationCommand, const uint32_t bitmap)
   {
     if ((bitmap >= rConfigurationCommand.m_Min) && (bitmap <= rConfigurationCommand.m_Max))
     {
       return true;
     }
     
     return false;
   }
} // namespace segway

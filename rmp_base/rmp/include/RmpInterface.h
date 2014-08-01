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

#ifndef RMP_INTERFACE_H
#define RMP_INTERFACE_H

#include <string>

#include <RmpTransport.h>
#include <RmpUserDefinedFeedbackType.h>
#include <RmpConfigurationCommand.h>
#include <RmpFault.h>

namespace segway
{
  /**
   * This class provides an interface to a rmp platform.
   * Please refer to the rmp user manual to understand the technical terms.
   */
  class RmpInterface
  {
  public:
    /**
     * Constructor
     */
    RmpInterface();
    
    /**
     * Destructor
     */
    virtual ~RmpInterface();
    
    /**
     * Initialize a udp communication
     * @param rIpAddress ip address of the rmp
     * @param portNumber port number of the rmp
     * @throw std::logic_error if the transport has already been initialized and has not been shut down
     * @copydetails segway_RmpUdp_RmpUdp_throw
     * @copydetails segway_RmpInterface_Initialize_protected_throw
     */
    virtual void Initialize(const std::string& rIpAddress, uint16_t portNumber);
    
    /**
     * Initialize a usb communication
     * @param rDevicePort
     * @throw std::logic_error if the transport has already been initialized and has not been shut down
     * @copydetails segway_RmpUsb_RmpUsb_throw
     * @copydetails segway_RmpInterface_Initialize_protected_throw
     */
    virtual void Initialize(const std::string& rDevicePort);
    
    /**
     * Stop the communication
     */
    virtual void ShutDown();

    /**
     * @defgroup segway_RmpInterface_transport_not_init_throw ExceptionTitle
     * @throw std::runtime_error if the transport has not been initialized
     */

    /**
     * Execute configuration command
     * @param rConfigurationCommand configuration command object as defined in RmpConfigurationCommand.h
     * @param value parameter value. This templated function is only implemented for type uint32_t and float.
     * @result wether the rmp is properly configured
     * @throw std::logic_error if called with template parameter different than uint32_t or float
     * @copydetails segway_RmpInterface_transport_not_init_throw
     * @copydetails segway_RmpInterface_SetConfiguration_protected_throw
     */
    template<typename T>
    bool SetConfiguration(const RmpConfigurationCommand<T>& rConfigurationCommand, T value);
    
    /**
     * Update
     * Read and update the platform state
     * @param forceUpdate force update. If true, will send a dummy command to cause a response.
     * @result wether the update was successfull
     * @copydetails segway_RmpInterface_transport_not_init_throw
     */
    virtual bool Update(bool forceUpdate = false);

    /**
     * @defgroup segway_RmpInterface_GetUserDefinedFeedback_throw ExceptionTitle
     * @throw std::logic_error if feedback bitmap has not been retrieved yet
     * @throw std::runtime_error if the rmp has not been configured to send the requested feedback
     */
    
    /**
     * Get a user defined feedback
     * @param type type of user defined feedback. Defined in RmpUserDefinedFeedbackType.h.
     * @result feedback. This templated function is only implemented for type uint32_t and float.
     * @throw std::logic_error if called with template parameter different than uint32_t or float
     * @copydetails segway_RmpInterface_GetUserDefinedFeedback_throw
     */
    template<typename T>
    T GetUserDefinedFeedback(UserDefinedFeedbackType type);

    /**
     * Reset position data on the rmp
     * @param bitmap bitmap describing which position data to reset, should not be greater than RESET_ALL_POSITION_DATA (0x0000001F)
     * @result wether the integrator(s) was(were) properly reset
     * @throw std::logic_error if the bitmap is not valid.
     * Please read the rmp manual for more information about bitmap validity. 
     * @copydetails segway_RmpInterface_transport_not_init_throw
     * @copydetails segway_RmpInterface_GetUserDefinedFeedback_throw
     */
    bool ResetIntegrators(uint32_t bitmap);

    /**
     * Reset all parameters stored in NVM to their default values
     * @result wether the parameters were properly reset
     * @copydetails segway_RmpInterface_transport_not_init_throw
     * @copydetails segway_RmpInterface_InitializeFeedbackBitmaps_throw
     */
    bool ResetParamsToDefault();

    /**
     * Set user defined feedback bitmap
     * @param bitmapType type of user defined feedback bitmap
     * @param bitmap feedback bitmap
     * @result wether the bitmap was properyly set
     * @copydetails segway_RmpInterface_transport_not_init_throw
     * @throw std::logic_error if bitmapType is not valid
     * @throw std::logic_error if bitmap is not valid.
     * Please read the rmp manual for more information about bitmap validity.
     * @copydetails segway_RmpInterface_InitializeFeedbackBitmaps_throw
     */ 
    bool SetUserDefinedFeedbackBitmap(UserDefinedFeedbackBitmapType bitmapType, const uint32_t bitmap);

    /**
     * Change operational mode
     * @param modeRequest desired operational mode. Defined in RmpUserDefinedFeedbackType.h.
     * @result wether the mode was properly changed
     * @copydetails segway_RmpInterface_transport_not_init_throw
     * @throw std::logic_error if modeRequest is not valid
     */
    bool ChangeOperationalMode(RmpOperationalModeRequest modeRequest);

    /**
     * Get a fault status description. This method returns a list of string describing a fault.
     * An empty list means not fault.
     * @param rDescription returned fault status description
     */
    void GetFaultStatusDescription(FaultStatusDescription& rDescription);
    
  protected:
    /**
     * @defgroup segway_RmpInterface_Initialize_protected_throw ExceptionTitle
     * @copydetails segway_RmpInterface_InitializeFeedbackBitmaps_throw
     * @throw std::runtime_error if unable to set the rmp in tractor mode
     */
    
    /**
     * Protected initialization method
     * Initialize the user defined feedback bitmaps and switch to tractor mode
     * @copydetails segway_RmpInterface_Initialize_protected_throw
     */
    void Initialize();

    /**
     * @defgroup segway_RmpInterface_InitializeFeedbackBitmaps_throw ExceptionTitle
     * @throw std::runtime_error if unable to initialize feedback bitmpas properly
     */
      
    /**
     * Get the uder defined feedback bitmap (configuration) from the rmp and
     * update the state representation accordingly
     * @copydetails segway_RmpInterface_InitializeFeedbackBitmaps_throw
     */
    void InitializeFeedbackBitmaps();

    /**
     * @defgroup segway_RmpInterface_SetConfiguration_protected_throw ExceptionTitle
     * @throw std::runtime_error if the command is not valid (feedbackType of configuration command is INVALID_FEEDBACK)
     */

    /**
     * Set up configuration
     * @param commandId configuration command id, specified in RmpConfigurationCommand.h
     * @param feedbackType user defined feeback type, specified in RmpUserdefinedFeedbackType
     * @param value configuration value
     * @result wether the configuaration was properly set up.
     * @copydetails segway_RmpInterface_SetConfiguration_protected_throw
     */
    bool SetConfiguration(uint16_t commandId, UserDefinedFeedbackType feedbackType, uint32_t value);

    /**
     * Send configuration command
     * @param commandId configuration command id, specified in RmpConfigurationCommand.h
     * @param value configuration value
     * @result wether the command was properly sent.
     */
    bool SendConfigurationCommand(uint16_t commandId, uint32_t value);

    /**
     * Get the number of feedback entries
     * @result number of feedback entries
     */
    size_t GetFeedbackEntryCount();

    /**
     * Update user defined feedback
     * Convert a byte array to user defined feedback data.
     * @param rBytes byte array
     */
    void UpdateUserDefinedFeedback(const Bytes& rBytes);

    /**
     * Check wether the feedback bitmap is valid
     * @param rConfigurationCommand bitmap configuration command
     * @param bitmap bitmap
     * @result wether the feedback bitmap is valid
     */
    bool IsValidFeedbackBitmap(const RmpConfigurationCommand<uint32_t>& rConfigurationCommand, const uint32_t bitmap);

    /**
     * Transport interface
     */
    RmpTransport::Ptr m_pTransport;

    /**
     * Wether the user defined feedback bitmap is set (known)
     */
    bool m_IsFeedbackBitmapSet;

    /**
     * Store the user defined feedback bitmap information
     */
    uint32_t m_FeedbackBitmap[FEEDBACK_TYPE_NBR];

    /**
     * Store the user defined feedback
     */
    uint32_t m_UserDefinedFeedback[FEEDBACK_TYPE_NBR][MAX_ENTRIES_PER_FEEDBACK];
  }; // class RmpInterface
} // namespace segway

#endif // RMP_INTERFACE_H

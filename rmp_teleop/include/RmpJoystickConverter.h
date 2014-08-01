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

#ifndef RMP_JOYSTICK_CONVERTER_H
#define RMP_JOYSTICK_CONVERTER_H

#include <sensor_msgs/Joy.h>

#include <memory>

/**
 * This class is an abstract base class.
 * It converts joystick messages to Rmp commands.
 */
class JoystickConverter
{
public:
  typedef std::shared_ptr<JoystickConverter> Ptr;

  /**
   * Define Joystick Type(s)
   */
  enum JoystickType
  {
    XBOX_WIRELLESS
  };

  /**
   * Create a joystick converter
   */
  static Ptr Create(JoystickType joystickType);

  /**
   * Destructor
   */
  virtual ~JoystickConverter()
  {}

  /**
   * Get the translational velocity reading
   * @param rJoyMessage joystick message
   * @result axe/button reading
   */
  virtual double GetTranslationalVelocity(const sensor_msgs::Joy& rJoyMessage) = 0;

  /**
   * Get the rotational velocity reading
   * @param rJoyMessage joystick message
   * @result axe/button reading
   */
  virtual double GetRotationalVelocity(const sensor_msgs::Joy& rJoyMessage) = 0;

  /**
   * Get wether the deadman button is pressed
   * @param rJoyMessage joystick message
   * @result wether the button is pressed
   */
  virtual bool GetDeadman(const sensor_msgs::Joy& rJoyMessage) = 0;

  /**
   * Get wether the boost button is pressed
   * @param rJoyMessage joystick message
   * @result wether the button is pressed
   */
  virtual bool GetBoost(const sensor_msgs::Joy& rJoyMessage) = 0;

  /**
   * Get audio command
   * @param rJoyMessage joystick message
   * @result return audio command or -1 if no command
   */
  virtual int GetAudioCommand(const sensor_msgs::Joy& rJoyMessage) = 0;
}; // class JoystickConverter

#endif // RMP_JOYSTICK_CONVERTER_H

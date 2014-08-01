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

#include <RmpXboxConverter.h>

#include <stdint.h>
#include <sstream>
#include <exception>

#include <rmp_msgs/AudioCommand.h>

static const uint32_t WIRELESS_AXES_SIZE = 6;
static const uint32_t WIRELESS_BUTTONS_SIZE = 15;
static const uint32_t WIRELESS_TRANSLATIONAL_VELOCITY_IDX = 4;
static const uint32_t WIRELESS_ROTATIONAL_VELOCITY_IDX = 3;
static const uint32_t WIRELESS_DEADMAN_IDX = 5;
static const uint32_t WIRELESS_BOOST_IDX = 1;
static const uint32_t WIRELESS_AUDIO_SWEEP_IDX = 0;

XboxWirelessConverter::XboxWirelessConverter()
  : JoystickConverter()
{}

XboxWirelessConverter::~XboxWirelessConverter()
{}

double XboxWirelessConverter::GetTranslationalVelocity(const sensor_msgs::Joy& rJoyMessage)
{
  IsValid(rJoyMessage);
  
  return rJoyMessage.axes[WIRELESS_TRANSLATIONAL_VELOCITY_IDX];
}

double XboxWirelessConverter::GetRotationalVelocity(const sensor_msgs::Joy& rJoyMessage)
{
  IsValid(rJoyMessage);

  return rJoyMessage.axes[WIRELESS_ROTATIONAL_VELOCITY_IDX];
}

bool XboxWirelessConverter::GetDeadman(const sensor_msgs::Joy& rJoyMessage)
{
  IsValid(rJoyMessage);
  
  return (rJoyMessage.axes[WIRELESS_DEADMAN_IDX] < 0.0);
}

bool XboxWirelessConverter::GetBoost(const sensor_msgs::Joy& rJoyMessage)
{
  IsValid(rJoyMessage);
  
  return (rJoyMessage.axes[WIRELESS_BOOST_IDX] > 0.8);
}

int XboxWirelessConverter::GetAudioCommand(const sensor_msgs::Joy& rJoyMessage)
{
  IsValid(rJoyMessage);

  if (rJoyMessage.buttons[WIRELESS_AUDIO_SWEEP_IDX])
  {
    return rmp_msgs::AudioCommand::TEST_SWEEP;
  }

  return -1;
}

void  XboxWirelessConverter::IsValid(const sensor_msgs::Joy& rJoyMessage)
{
  if ( (rJoyMessage.axes.size() != WIRELESS_AXES_SIZE) ||
       (rJoyMessage.buttons.size() != WIRELESS_BUTTONS_SIZE) )
  {
    std::stringstream stringStream;
    stringStream << "Invalid joystick message. Expecting " << WIRELESS_AXES_SIZE << " axes and "
                 << WIRELESS_BUTTONS_SIZE << " buttons but got a meesage with " << rJoyMessage.axes.size()
                 << " axes and " << rJoyMessage.buttons.size() << " buttons.";

    throw std::logic_error(stringStream.str());
  }
}

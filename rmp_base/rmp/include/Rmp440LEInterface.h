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

#ifndef RMP_440LE_INTERFACE_H
#define RMP_440LE_INTERFACE_H

#include <RmpInterface.h>

namespace segway
{
  /**
   * This class represents an interface to a Segway RMP 440LE.
   * It implements a method to process velocity command for vehicles that respond to standard motion commands.
   */
  class Rmp440LEInterface: public RmpInterface
  {
  public:
    /**
     * Constructor
     */
    Rmp440LEInterface();
    
    /**
     * Destructor
     */
    virtual ~Rmp440LEInterface();
    
    /**
     * Send velocity command
     * @param normalizedVelocity normalized velocity [-1, 1]
     * @param normalizedYawRate normalized yaw rate [-1, 1]
     * @result wether the command was sent
     * @throw std::logic_error if one or both of the velocity inputs is out of range
     * @throw std::runtime_error if the communication has not been initialized 
     */
    virtual bool SetVelocity(float normalizedVelocity, float normalizedYawRate);
    
    /**
     * Get the maximum velocity in [m/s]
     * @result maximum velocity
     */
    float GetMaximumVelocity();
    
    /**
     * Get the maximum turn rate in [rad/s]
     * @result maximum turn rate
     */
    float GetMaximumTurnRate();

  }; // class Rmp440LEInterface
} // namespace segway

#endif // RMP_440LE_INTERFACE_H

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

#ifndef RMP_UDP_H
#define RMP_UDP_H

#include <string>
#include <memory>

#include <RmpTransport.h>

namespace segway
{
  /**
   * This class implements a udp interface to a Segway rmp.
   */
  class RmpUdp: public RmpTransport
  {
  public:
    /**
     * @defgroup segway_RmpUdp_RmpUdp_throw ExceptionTitle
     * @throw std::runtime_error if unable to connect to the socket
     */
    
    /**
     * Constructor
     * @param rIpAddress ip address of the rmp
     * @param portNumber port number of the rmp
     * @copydetails segway_RmpUdp_RmpUdp_throw
     */
    RmpUdp(const std::string& rIpAddress, uint16_t portNumber);
    
    /**
     * Destructor
     */
    virtual ~RmpUdp();
    
    /**
     * Send command to rmp
     * @param commandId command id
     * @param value1 first value
     * @param value2 second value
     * @result wether the command was properly sent
     */
    virtual bool Send(uint16_t commandId, uint32_t value1, uint32_t value2);
    
    /**
     * Receive response from rmp
     * @param rData response
     * @param size expected size of data to receive [in byte]
     * @result wether the data were properly received
     */
    virtual bool Receive(Bytes& rData, size_t size);
  
  private:
    /**
     * Pointer to implementation
     */
    class Impl;
    std::unique_ptr<Impl> m_pImpl;
  }; // class RmpUdp
} // namespace segway

#endif // RMP_UDP_H

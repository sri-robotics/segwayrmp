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

#include <RmpUdp.h>

#include <memory>
#include <utility>
#include <iostream>
#include <exception>

#include <boost/asio.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>

#include <RmpHelper.h>
#include <RmpLogger.h>

namespace segway
{
  using boost::asio::ip::udp;

  static const size_t MAX_RCV_SIZE = 1024;
  static const long TIMEOUT = 30; // [ms]
  
  class RmpUdp::Impl
  {
  public: 
    Impl(const udp::endpoint& rEndpoint);
    size_t Send(const Bytes& rData, float timeout, boost::system::error_code& rErrorCode);
    size_t Receive(Bytes& rData, float timeout, boost::system::error_code& rErrorCode);

  private:
    void CheckDeadline();
    static void Handle(const boost::system::error_code& rErrorCodeIn, size_t sizeIn, boost::system::error_code* pErrorCodeOut, std::size_t* pSizeOut);
    
    boost::asio::io_service m_IoService;
    udp::socket m_Socket;
    boost::asio::deadline_timer m_Timer;
    Bytes m_ReadBuffer;
  };

  RmpUdp::RmpUdp(const std::string& rIpAddress, uint16_t portNumber)
  {
    udp::endpoint endpoint(boost::asio::ip::address::from_string(rIpAddress.c_str()), portNumber);

    m_pImpl = std::unique_ptr<Impl>(new Impl(endpoint));
  }

  RmpUdp::~RmpUdp()
  {}

  bool RmpUdp::Send(uint16_t commandId, uint32_t value1, uint32_t value2)
  {
    Bytes message;
    ConvertCommandToBytes(commandId, value1, value2, message);
    AppendCrc16(message);

    boost::system::error_code errorCode;
    size_t bytesSent = m_pImpl->Send(message, TIMEOUT, errorCode);

    if (errorCode.value() != boost::system::errc::success)
    {
      SEGWAY_LOG(ERROR, "Fail to udp send: " << errorCode.message());

      return false;
    }
    
    if (bytesSent != message.size())
    {
      return false;
    }

    return true;
  }
  
  bool RmpUdp::Receive(Bytes& rData, size_t size)
  {
    rData.resize(size + CRC_FEEDBACK_SIZE);

    boost::system::error_code errorCode;
    size_t bytesReceived = m_pImpl->Receive(rData, TIMEOUT, errorCode);

    if (errorCode.value() != boost::system::errc::success)
    {
      SEGWAY_LOG(DEBUG, "Fail to receive udp packet: " << errorCode.message());

      return false;
    }
    
    if (bytesReceived != (size + CRC_FEEDBACK_SIZE))
    {
      SEGWAY_LOG(DEBUG, "Received udp packet of unexpected size: " << bytesReceived << ", expected: " << size + CRC_FEEDBACK_SIZE);
      
      return false;
    }

    if (!IsCrcValid(rData, bytesReceived))
    {
      SEGWAY_LOG(ERROR, "CRC mismatched on udp receive.");

      return false;
    }

    rData.erase(rData.begin() + size, rData.end());

    return true;
  }

  RmpUdp::Impl::Impl(const udp::endpoint& rEndpoint)
    : m_Socket(m_IoService)
    , m_Timer(m_IoService)
  {
    boost::system::error_code errorCode;
    m_Socket.connect(rEndpoint, errorCode);

    if (errorCode.value() != boost::system::errc::success)
    {
      std::stringstream stringStream;
      stringStream << "Unable to connect to socket: " << errorCode.message();
      
      throw std::runtime_error(stringStream.str());
    }

    m_ReadBuffer.resize(MAX_RCV_SIZE);

    m_Timer.expires_at(boost::posix_time::pos_infin);
         
    CheckDeadline();
  }

  size_t RmpUdp::Impl::Send(const Bytes& rData, float timeout, boost::system::error_code& rErrorCode)
  {
    m_Timer.expires_from_now(boost::posix_time::milliseconds(timeout));

    rErrorCode = boost::asio::error::would_block;
    size_t size = 0;

    m_Socket.async_send(boost::asio::buffer(rData), boost::bind(&Impl::Handle, _1, _2, &rErrorCode, &size));

    do
    {
      m_IoService.run_one();
    }
    while (rErrorCode == boost::asio::error::would_block);
    
    return size;
  }

  size_t RmpUdp::Impl::Receive(Bytes& rData, float timeout, boost::system::error_code& rErrorCode)
  {
    m_Timer.expires_from_now(boost::posix_time::milliseconds(timeout));

    rErrorCode = boost::asio::error::would_block;
    size_t size = 0;

    m_Socket.async_receive(boost::asio::buffer(m_ReadBuffer), boost::bind(&Impl::Handle, _1, _2, &rErrorCode, &size));

    do
    {
      m_IoService.run_one();
    }
    while (rErrorCode == boost::asio::error::would_block);

    if (size >= rData.size())
    {
      size_t startIdx = size - rData.size();
      rData.assign(m_ReadBuffer.begin() + startIdx, m_ReadBuffer.end() - 1);
    }
    else
    {
      rData.assign(m_ReadBuffer.begin(), m_ReadBuffer.end() - 1);
    }
    
    return size;
  }

  void RmpUdp::Impl::CheckDeadline()
  {
    if (m_Timer.expires_at() <= boost::asio::deadline_timer::traits_type::now())
    {
      m_Socket.cancel();

      m_Timer.expires_at(boost::posix_time::pos_infin);
    }

    m_Timer.async_wait(boost::bind(&Impl::CheckDeadline, this));
  }

  void RmpUdp::Impl::Handle(const boost::system::error_code& rErrorCodeIn, size_t sizeIn, boost::system::error_code* pErrorCodeOut, std::size_t* pSizeOut)
  {
    *pErrorCodeOut = rErrorCodeIn;
    *pSizeOut = sizeIn;
  }
  
} // namespace segway

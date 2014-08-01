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

#include <RmpUsb.h>

#include <exception>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>

#include <RmpHelper.h>
#include <RmpLogger.h>

namespace segway
{
  static const size_t MAX_RCV_SIZE = 1024;
  static const long TIMEOUT = 30; // [ms]
  static const unsigned int BAUD_RATE = 115200;
  static const unsigned int CHARACTER_SIZE = 8; // [bit]
  
  class RmpUsb::Impl
  {
  public:
    Impl(const std::string& rDevicePort);
    size_t Write(const Bytes& rData, float timeout, boost::system::error_code& rErrorCode);
    size_t Read(Bytes& rData, float timeout, boost::system::error_code& rErrorCode);

  private:
    void CheckDeadline();
    static void Handle(const boost::system::error_code& rErrorCodeIn, size_t sizeIn, boost::system::error_code* pErrorCodeOut, std::size_t* pSizeOut);

    boost::asio::io_service m_IoService;
    boost::asio::serial_port m_SerialPort;
    boost::asio::deadline_timer m_Timer;
  };

  RmpUsb::RmpUsb(const std::string& rDevicePort)
    : m_pImpl(new Impl(rDevicePort)) 
  {}

  RmpUsb::~RmpUsb()
  {}

  bool RmpUsb::Send(uint16_t commandId, uint32_t value1, uint32_t value2)
  {
    Bytes message;
    ConvertCommandToBytes(commandId, value1, value2, message);
    AppendCrc16(message);

    boost::system::error_code errorCode;
    size_t bytesSent = m_pImpl->Write(message, TIMEOUT, errorCode);

    if (errorCode.value() != boost::system::errc::success)
    {
      SEGWAY_LOG(ERROR, "Fail to usb send: " << errorCode.message());

      return false;
    }
    
    if (bytesSent != message.size())
    {
      return false;
    }

    return true;
  }

  bool RmpUsb::Receive(Bytes& rData, size_t size)
  {
    rData.resize(size + CRC_FEEDBACK_SIZE);

    boost::system::error_code errorCode;
    size_t bytesReceived = m_pImpl->Read(rData, TIMEOUT, errorCode);

    if (errorCode.value() != boost::system::errc::success)
    {
      SEGWAY_LOG(DEBUG, "Fail to receive usb packet: " << errorCode.message());

      return false;
    }
    
    if (bytesReceived != (size + CRC_FEEDBACK_SIZE))
    {
      SEGWAY_LOG(DEBUG, "Received usb packet of unexpected size: " << bytesReceived << ", expected: " << size + CRC_FEEDBACK_SIZE);
      
      return false;
    }
 
    if (!IsCrcValid(rData, bytesReceived))
    {
      SEGWAY_LOG(ERROR, "CRC mismatched on usb receive.");

      return false;
    }

    rData.erase(rData.begin() + size, rData.end());

    return true;
  }

  RmpUsb::Impl::Impl(const std::string& rDevicePort)
    : m_IoService()
    , m_SerialPort(m_IoService)
    , m_Timer(m_IoService)
  {
    boost::system::error_code errorCode;
    m_SerialPort.open(rDevicePort, errorCode);

    if (errorCode.value() != boost::system::errc::success)
    {
      std::stringstream stringStream;
      stringStream << "Unable to open serial port " << rDevicePort << ": " << errorCode.message();
      
      throw std::runtime_error(stringStream.str());
    }
    
    m_SerialPort.set_option(boost::asio::serial_port_base::baud_rate(BAUD_RATE));
    m_SerialPort.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    m_SerialPort.set_option(boost::asio::serial_port_base::character_size(CHARACTER_SIZE));
    m_SerialPort.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
    m_SerialPort.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    
    m_Timer.expires_at(boost::posix_time::pos_infin);
         
    CheckDeadline();
  }

  size_t RmpUsb::Impl::Write(const Bytes& rData, float timeout, boost::system::error_code& rErrorCode)
  {
    m_Timer.expires_from_now(boost::posix_time::milliseconds(timeout));

    rErrorCode = boost::asio::error::would_block;
    size_t size = 0;

    boost::asio::async_write(m_SerialPort, boost::asio::buffer(rData), boost::bind(&Impl::Handle, _1, _2, &rErrorCode, &size));

    do
    {
      m_IoService.run_one();
    }
    while (rErrorCode == boost::asio::error::would_block);
    
    return size;
  }

  size_t RmpUsb::Impl::Read(Bytes& rData, float timeout, boost::system::error_code& rErrorCode)
  {
    m_Timer.expires_from_now(boost::posix_time::milliseconds(timeout));

    rErrorCode = boost::asio::error::would_block;
    size_t size = 0;

    boost::asio::async_read(m_SerialPort, boost::asio::buffer(rData), boost::bind(&Impl::Handle, _1, _2, &rErrorCode, &size));

    do
    {
      m_IoService.run_one();
    }
    while (rErrorCode == boost::asio::error::would_block);

    return size;
  }

  void RmpUsb::Impl::CheckDeadline()
  {
    if (m_Timer.expires_at() <= boost::asio::deadline_timer::traits_type::now())
    {
      m_SerialPort.cancel();

      m_Timer.expires_at(boost::posix_time::pos_infin);
    }

    m_Timer.async_wait(boost::bind(&Impl::CheckDeadline, this));
  }

  void RmpUsb::Impl::Handle(const boost::system::error_code& rErrorCodeIn, size_t sizeIn, boost::system::error_code* pErrorCodeOut, std::size_t* pSizeOut)
  {
    *pErrorCodeOut = rErrorCodeIn;
    *pSizeOut = sizeIn;
  }

} // namespace segway

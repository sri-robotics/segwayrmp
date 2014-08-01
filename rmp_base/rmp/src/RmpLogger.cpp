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

#include <RmpLogger.h>

#include <exception>
#include <vector>
#include <fstream>

#include <boost/thread/mutex.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace segway
{
  static boost::mutex s_Mutex;

  std::string GetLogLevelString(LogLevel level);

  struct Logger::Impl
  {
    static Impl& Instance()
    {
      static Impl instance;

      return instance;
    }
    
    Impl();

    ~Impl();

    void AddOutputStream(std::ostream& rOutputStream);

    std::vector<std::ostream*> m_OutputStreamList;
    LogLevel m_MinLogLevel;
  };

  void Logger::SetMinLogLevel(LogLevel level)
  {
    boost::mutex::scoped_lock scoped_lock(s_Mutex);

    Logger::Impl::Instance().m_MinLogLevel = level;
  }

  void Logger::AddOutputStream(std::ostream& rOutputStream)
  {
    boost::mutex::scoped_lock scoped_lock(s_Mutex);

    Logger::Impl::Instance().AddOutputStream(rOutputStream);
  }

  void Logger::AddLogFile(const std::string& rFileName)
  {
    boost::mutex::scoped_lock scoped_lock(s_Mutex);

    std::ofstream* pFileStream = new std::ofstream();
    pFileStream->open(rFileName);

    Logger::Impl::Instance().AddOutputStream(*pFileStream);
  }

  void Logger::Log(LogLevel level, const std::string& rMessage)
  {
    boost::mutex::scoped_lock scoped_lock(s_Mutex);

    if (level >= Impl::Instance().m_MinLogLevel)
    {
      boost::posix_time::ptime logTime = boost::posix_time::microsec_clock::local_time();
      std::string logLevelString = GetLogLevelString(level);
      
      std::stringstream logMessage;
      logMessage << logTime << ", " << logLevelString << ":\t" << rMessage << std::endl; 

      for (size_t i = 0; i < Impl::Instance().m_OutputStreamList.size(); ++i)
      {
        std::ostream* pOutputStream = Impl::Instance().m_OutputStreamList[i];
        
        if (pOutputStream != nullptr)
        {
          *pOutputStream << logMessage.str();
          pOutputStream->flush();
        }
      }
    }
  }

  Logger::Impl::Impl()
    : m_MinLogLevel(DEBUG)
  {}

  Logger::Impl::~Impl()
  {
    for (size_t i = 0; i < m_OutputStreamList.size(); ++i)
    {
      std::ostream* pOutputStream = m_OutputStreamList[i];
      
      if (pOutputStream != nullptr)
      {
        std::ofstream* pOutputFileStream = dynamic_cast<std::ofstream*>(pOutputStream);
            
        if (pOutputFileStream != nullptr)
        {
          pOutputFileStream->close();
          delete pOutputFileStream;
        }
      }
    }
  }

  void Logger::Impl::AddOutputStream(std::ostream& rOutputStream)
  {
    bool present = false;
    
    for (size_t i = 0; i < m_OutputStreamList.size(); ++i)
    {
      if (m_OutputStreamList[i] != nullptr)
      {
        if (*(m_OutputStreamList[i]) == rOutputStream)
        {
          present = true;
          break;
        }
      }
    }

    if (!present)
    {
      m_OutputStreamList.push_back(&rOutputStream);
    }
  }

  std::string GetLogLevelString(LogLevel level)
  {
    switch (level)
    {
      case DEBUG:
      {
        return std::string("DEBUG");
      }
      case INFO:
      {
        return std::string("INFO");
      }
      case WARNING:
      {
        return std::string("WARNING");
      }
      case ERROR:
      {
        return std::string("ERROR");
      }
      case FATAL:
      {
        return std::string("FATAL");
      }
      default:
      {
        throw std::logic_error("LogLevel cannot be converted to string.");
      }
    }

    return std::string("Houston, we have a problem!");
  }
} // namespace segway


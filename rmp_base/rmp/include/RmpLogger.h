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

#ifndef RMP_LOGGER_H
#define RMP_LOGGER_H

#include <string>
#include <ostream>
#include <sstream>
#include <memory>

namespace segway
{
  /**
   * Descibe the severity level of the log
   */
  enum LogLevel
  {
    DEBUG = 0,
    INFO,
    WARNING,
    ERROR,
    FATAL
  };

  /**
   * This class is used for logging.
   */
  class Logger
  {
  public:
    /**
     * Set the minimum severity of the message to be logged.
     * @param level minimum severity level
     */
    static void SetMinLogLevel(LogLevel level);

    /**
     * Add an output stream to log to, such as std::cout, std::cerr...
     * @param rOutputStream output stream
     */
    static void AddOutputStream(std::ostream& rOutputStream);

    /**
     * Create a file for logging.
     * @param rFileName file name
     */
    static void AddLogFile(const std::string& rFileName);

    /**
     * Log a message
     * @param level log level
     * @param rMessage log message
     */
    static void Log(LogLevel level, const std::string& rMessage);
    
  private:
    /**
     * All constructors should be private.
     * The user should not be able to create/modify any objects of this class.
     */

    /**
     * Default constructor
     */
    Logger();

    /**
     * Copy constructor
     */
    Logger(const Logger&);

    /**
     * Assignment operator
     */
    Logger& operator=(const Logger&);

    /**
     * Destructor
     */
    ~Logger();

  private:
    /**
     * Pointer to implementation
     */
    class Impl;
    std::unique_ptr<Impl> m_pImpl;
  };
  
} // namespace segway

/**
 * Convenience macro for logging stream like messages.
 */
#define SEGWAY_LOG(level, args) \
  std::stringstream stringStream; \
  stringStream << args; \
  std::string message = stringStream.str(); \
  segway::Logger::Log(level, message);

#endif // RMP_LOGGER_H

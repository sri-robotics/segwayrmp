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

#ifndef RMP_FAULT_H
#define RMP_FAULT_H

#include <cstdint>
#include <string>
#include <vector>
#include <memory>

#include <RmpUserDefinedFeedbackType.h>

namespace segway
{
  static const uint32_t NO_FAULT = 0x00000000;
  
  typedef std::vector<std::string> FaultStatusDescription;

  /**
   * Define fault group type
   * Please read the rmp user manual (for instance, page 73 of the RMP 440LE manual)
   */
  enum FaultGroup
  {
    TRANSIENT = 0,
    CRITICAL,
    COMM,
    SENSORS,
    BSA,
    MCU,
    ARCHITECTURE,
    INTERNAL
  };

  class RmpFault;
  
  /**
   * This class describes how the status bits are packed in the feedback words
   * Please read the rmp user manual (for instance, page 77 of the RMP 440LE manual)
   */
  class FaultPacking
  {
    friend class RmpFault;

  public:
    /**
     * Copy constructor
     * @param rOther other instance of FaultPacking
     */ 
    FaultPacking(const FaultPacking& rOther)
      : m_Shift(rOther.m_Shift)
      , m_Mask(rOther.m_Mask)
      , m_Group(rOther.m_Group)
    {}

    /**
     * Assignment operator
     * @param rOther other instance of FaultPacking
     */
    FaultPacking& operator=(const FaultPacking& rOther)
    {
      m_Shift = rOther.m_Shift;
      m_Mask = rOther.m_Mask;
      m_Group = rOther.m_Group;
    }

    /**
     * Return shift member
     * @result bit shift
     */
    const uint32_t& GetShift() const;

    /**
     * Return mask member
     * @result bit mask
     */
    const uint32_t& GetMask() const;

    /**
     * Return fault group
     * @result fault group
     */
    const FaultGroup& GetGroup() const;

  private:
    /**
     * Constructor
     * @param shift number of bits to shift
     * @param mask bit mask
     * @param group fault group
     */
    FaultPacking(uint32_t shift, uint32_t mask, FaultGroup group)
      : m_Shift(shift)
      , m_Mask(mask)
      , m_Group(group)
    {}
  
    /**
     * Default constructor
     */ 
    FaultPacking()
    {}

    /**
     * Number of bits to shift in order to read the fault group
     */
    uint32_t m_Shift;

    /**
     * Mask for the fault group
     */
    uint32_t m_Mask;

    /**
     * Fault group
     */
    FaultGroup m_Group;
  };

  typedef std::vector<FaultPacking> FaultPackingList;
  typedef std::vector<UserDefinedFeedbackType> FeedbackTypeList;
  typedef std::vector<std::string> FaultDecodeList;

  /**
   * This class helps interpreting the feedback words
   * Please read the rmp user manual (for instance, page 73 of the RMP 440LE manual)
   */
  class RmpFault
  {
  public:
    /**
     * Get the list of feedback entries that correspond to a fault status
     * @result feedback list
     */
    static const FeedbackTypeList& GetFaultFeedbackEntryList();

    /**
     * Get the fault packing struct of a feedback entry
     * @param feedbackType feedback entry
     * @result corresponding packing struct
     */
    static const FaultPackingList& GetFaultPackingList(UserDefinedFeedbackType feedbackType);

    /**
     * Get the list of fault description for a given group
     * @param faultGroup fault group
     * @result list of fault description
     */
    static const FaultDecodeList& GetFaultDecodeList(FaultGroup faultGroup);

  private:
    /**
     * All constructors should be private.
     * The user should not be able to create/modify any objects of this class.
     */

    /**
     * Default constructor
     */
    RmpFault();

    /**
     * Copy constructor
     */
    RmpFault(const RmpFault&);

    /**
     * Assignment operator
     */
    RmpFault& operator=(const RmpFault&);

    /**
     * Constructor
     */
    ~RmpFault();

    /**
     * Pointer to implementation
     */
    struct Impl;
    std::unique_ptr<Impl> m_pImpl;
  };
  
} // namespace segway

#endif // RMP_FAULT_H

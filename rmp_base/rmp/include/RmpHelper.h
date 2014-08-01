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

#ifndef RMP_HELPER_H
#define RMP_HELPER_H

#include <assert.h>

#include <RmpType.h>

namespace segway
{
  static const size_t RMP_CMD_BODY_SIZE = sizeof(uint16_t) + 2 * sizeof(uint32_t);
  static const size_t RMP_CMD_HEADER_IDX = 0;
  static const size_t RMP_CMD_VAL1_IDX = sizeof(uint16_t);
  static const size_t RMP_CMD_VAL2_IDX = sizeof(uint16_t) + sizeof(uint32_t);

  static const size_t CRC_TABLE_SIZE = 256;
  static const uint16_t CRC_ADJUSTMENT = 0xA001;
  static const size_t CRC_SIZE = sizeof(uint16_t);
  static const size_t CRC_FEEDBACK_SIZE = sizeof(uint32_t);

  /**
   * Convert a 16 bits unsigned integer to a byte array
   * @param rData data to convert
   * @param pByte byte array (should already be allocated)
   * @param size size of the array
   */
  inline void ConvertUint16ToBytes(const uint16_t& rData, uint8_t* pByte, size_t size)
  {
    assert(sizeof(uint16_t) <= size);

    pByte[0] = static_cast<uint8_t>((rData & 0xFF00) >> 8);
    pByte[1] = static_cast<uint8_t>((rData & 0x00FF) >> 0);
  }
  
  /**
   * Convert a 32 bits unsigned integer to a byte array
   * @param rData data to convert
   * @param pByte byte array (should already be allocated)
   * @param size size of the array
   */
  inline void ConvertUint32ToBytes(const uint32_t& rData, uint8_t* pByte, size_t size)
  {
    assert(sizeof(uint32_t) <= size);

    pByte[0] = static_cast<uint8_t>((rData & 0xFF000000) >> 24);
    pByte[1] = static_cast<uint8_t>((rData & 0x00FF0000) >> 16);
    pByte[2] = static_cast<uint8_t>((rData & 0x0000FF00) >> 8);
    pByte[3] = static_cast<uint8_t>((rData & 0x000000FF) >> 0);
  }

  /**
   * Convert a byte array to a 16 bits unsigned integer
   * @param pByte byte array
   * @param size size of the array
   * @result 16 bits unsigned integer
   */
  inline uint16_t ConvertBytesToUint16(const uint8_t* pByte, size_t size)
  {
    assert(size >= sizeof(uint16_t));

    return ((static_cast<uint16_t>(pByte[0]) << 8) & 0xFF00) |
           ((static_cast<uint16_t>(pByte[1]) << 0) & 0x00FF);
  }

  /**
   * Convert a byte array to a 32 bits unsigned integer
   * @param pByte byte array
   * @param size size of the array
   * @result 32 bits unsigned integer
   */
  inline uint32_t ConvertBytesToUint32(const uint8_t* pByte, size_t size)
  {
    assert(size >= sizeof(uint32_t));

    return ((static_cast<uint32_t>(pByte[0]) << 24) & 0xFF000000) |
           ((static_cast<uint32_t>(pByte[1]) << 16) & 0x00FF0000) |
           ((static_cast<uint32_t>(pByte[2]) << 8)  & 0x0000FF00) |
           ((static_cast<uint32_t>(pByte[3]) << 0)  & 0x000000FF);
  }

  /**
   * Convert a float to a 32 bits unsigned integer
   * @param data float
   * @result 32 bits unsigned integer
   */
  inline uint32_t ConvertFloatToUint32(float data)
  {
    return (*((uint32_t*)&data));
  }

  /**
   * Convert a 32 bits unsigned integer to a float
   * @param data 32 bits unsigned integer
   * @result float
   */
  inline float ConvertUint32ToFloat(uint32_t data)
  {
    return (*((float*)&data));
  }
  
  /**
   * Convert a rmp command to a byte vector
   * @param commandId command id
   * @param value1 first value
   * @param value2 second value
   * @param rBytes returned byte vector
   */
  inline void ConvertCommandToBytes(uint16_t commandId, uint32_t value1, uint32_t value2, Bytes& rBytes)
  {
    rBytes.resize(RMP_CMD_BODY_SIZE);

    ConvertUint16ToBytes(commandId, &rBytes[RMP_CMD_HEADER_IDX], sizeof(uint16_t));
    ConvertUint32ToBytes(value1, &rBytes[RMP_CMD_VAL1_IDX], sizeof(uint32_t));
    ConvertUint32ToBytes(value2, &rBytes[RMP_CMD_VAL2_IDX], sizeof(uint32_t));
  }

  /**
   * This structure describes a Cyclic Redundance Check Table
   */
  struct CrcTable
  {
    /**
     * Constructor
     */
    CrcTable()
    {
      for (uint16_t byte = 0; byte < CRC_TABLE_SIZE; ++byte)
      {
        uint16_t value = 0;
        uint16_t k = byte;

        for (uint16_t j = 0; j < 8; ++j)
        {
          if (((value ^ k) & 0x0001) == 0x0001)
          {
            value = (value >> 1) ^ CRC_ADJUSTMENT;
          }
          else
          {
            value >>= 1;
          }

          k >>= 1;
        }
        
        m_Entries[byte] = value; 
      }
    }

    /**
     * Entry accessor
     * @param idx entry index
     * @result entry value
     */
    uint16_t& operator[](size_t idx)
    {
      assert(idx < CRC_TABLE_SIZE);
      
      return m_Entries[idx];
    }

    /**
     * Entry accessor
     * @param idx entry index
     * @result entry value
     */
    const uint16_t& operator[](size_t idx) const
    {
      assert(idx < CRC_TABLE_SIZE);
      
      return m_Entries[idx];
    }

  private:
    /**
     * Table entries
     */
    uint16_t m_Entries[CRC_TABLE_SIZE];
  };

  /**
   * Compute the checksum of a byte vector
   * @param rBytes byte vector
   * @param size number of bytes to check
   * @result crc value
   */
  inline uint16_t GetCrc(Bytes& rBytes, size_t size)
  {
    static const CrcTable crcTable;

    uint16_t crc = 0;

    assert(size <= rBytes.size());
    
    for (size_t i = 0; i < size; ++i)
    {
      uint64_t tmp = crc ^ rBytes[i];
      crc = (crc >> 8) ^ crcTable[tmp & 0x00FF];
    }

    return crc;
  }

  /**
   * Append checksum at the end of byte vector
   * @param rBytes byte vector
   */
  inline void AppendCrc16(Bytes& rBytes)
  {
    uint16_t crc = GetCrc(rBytes, rBytes.size());
    uint8_t crcBytes[2];
    ConvertUint16ToBytes(crc, crcBytes, 2);

    rBytes.push_back(crcBytes[0]);
    rBytes.push_back(crcBytes[1]);
  }

  /**
   * Check data integrity
   * @param rBytes byte vector
   * @param size size of the data to check
   * @result wether the Crc values matche
   */
  inline bool IsCrcValid(Bytes& rBytes, size_t size)
  {
    uint16_t computedCrc = GetCrc(rBytes, size - CRC_SIZE);
    uint16_t receivedCrc = ConvertBytesToUint16(&rBytes[size - CRC_SIZE], sizeof(uint16_t));

    if (computedCrc != receivedCrc)
    {
      return false;
    }

    return true;
  }
} // segway

#endif // RMP_HELPER_H

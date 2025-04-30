#include "Math.h"



/**
 * @param packedValue The short storing the X component in the first 8 bits
 * 
 * @brief Unpacks the X component of the packedValue
 * 
 * @returns The X component of the packedValue
 */
inline char Math::getX(short packedValue)
    {
    return static_cast<char>(packedValue);
    }



/**
 * @param packedValue The long storing the X component in the first 16 bits
 * 
 * @brief Unpacks the X component of the packedValue
 * 
 * @returns The X component of the packedValue
 */
inline short Math::getX(long packedValue)
    {
    return static_cast<short>(packedValue);
    }



/**
 * @param packedValue The short storing the Y component in the last 8 bits
 * 
 * @brief Unpacks the Y component of the packedValue
 * 
 * @returns The Y component of the packedValue
 */
inline char Math::getY(short packedValue)
    {
    return static_cast<char>((packedValue >> 8) & 0xFF);
    }



/**
 * @param packedValue The long storing the Y component in the last 16 bits
 * 
 * @brief Unpacks the Y component of the packedValue
 * 
 * @returns The Y component of the packedValue
 */
inline short Math::getY(long packedValue)
    {
    return static_cast<short>((packedValue >> 16) & 0xFFFF);
    }
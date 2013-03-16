/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Michael Moser
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <sys/time.h>
#include <time.h>
#include <string.h>
#include <stdio.h>
#include <gbxnovatelacfr/gbxnovatelutilacfr/crc32.h>
using namespace std;

//////////////////////////////////////////////////////////
// This is the CRC implementation from the novatel docs://
//////////////////////////////////////////////////////////
#define CRC32_POLYNOMIAL 0xEDB88320L
/* --------------------------------------------------------------------------
 * Calculate a CRC value to be used by CRC calculation functions.
 * -------------------------------------------------------------------------- */
unsigned long CRC32Value(int i)
{
    int j;
    unsigned long ulCRC;
    ulCRC = i;
    for ( j = 8 ; j > 0; j-- )
    {
        if ( ulCRC & 1 )
        ulCRC = ( ulCRC >> 1 ) ^ CRC32_POLYNOMIAL;
        else
        ulCRC >>= 1;
    }
    return ulCRC;
}
/* --------------------------------------------------------------------------
Calculates the CRC-32 of a block of data all at once
-------------------------------------------------------------------------- */
unsigned long CalculateBlockCRC32( unsigned long ulCount, /* Number of bytes in the data block */
                    unsigned char *ucBuffer ) /* Data block */
{
    unsigned long ulTemp1;
    unsigned long ulTemp2;
    unsigned long ulCRC = 0;
    while ( ulCount-- != 0 )
    {
        ulTemp1 = ( ulCRC >> 8 ) & 0x00FFFFFFL;
        ulTemp2 = CRC32Value( ((int) ulCRC ^ *ucBuffer++ ) & 0xff );
        ulCRC = ulTemp1 ^ ulTemp2;
    }
    return( ulCRC );
}

namespace gnua = gbxnovatelutilacfr;
int main(void){
    struct timeval tv;
    gettimeofday(&tv, NULL);
    static unsigned int randSeed = tv.tv_usec;

    int novatelCompareFailNum = 0;
    int falsePositivesNum = 0;
    int singleRandomBitFlipFailNum = 0;
    int doubleRandomBitFlipFailNum = 0;
    int tripleRandomBitFlipFailNum = 0;

    int k;
    for(k=0; k<10000; k++){
        //get a buffer of length 0-1000 and fill it with random crap
        int bufLen = (int) (1001.0 * (rand_r(&randSeed) / (RAND_MAX + 1.0)));
        unsigned char *buf = new unsigned char[bufLen];
        for(int i=0; i<bufLen; i++){
            buf[i] = (unsigned char) (256.0 * (rand_r(&randSeed) / (RAND_MAX + 1.0)));
        }
        // check that our implementation matches Novatel's
        uint32_t myCrc = gnua::crc(buf, bufLen);
        uint32_t novatelCrc = CalculateBlockCRC32(bufLen, buf);
        if (myCrc != novatelCrc){
            novatelCompareFailNum++;
        }

        size_t faultBufLen = bufLen + 4;
        unsigned char *faultBuf = new unsigned char[faultBufLen];
        int errorByte = 0, errorBit = 0;
        int errorByteTwo = 0, errorBitTwo = 0;
        int errorByteThree = 0, errorBitThree = 0;
        //no fault
        {
            memcpy(faultBuf, buf, bufLen);
            unsigned char *crcp = (unsigned char *)&myCrc;
            for(int j=0; j<4; j++){
                faultBuf[bufLen+j] = crcp[j];
            }
            uint32_t tmpCrc;
            if(0 != (tmpCrc = gnua::crc(faultBuf, faultBufLen))){
                falsePositivesNum++;
            }
        }
        //single bit flip
        {
            memcpy(faultBuf, buf, bufLen);
            errorByte = (int) ((double)faultBufLen * (rand_r(&randSeed) / (RAND_MAX + 1.0)));
            errorBit = (int) (8.0 * (rand_r(&randSeed) / (RAND_MAX + 1.0)));
            if(0 == faultBuf[errorByte] && (unsigned char)(1<<errorBit))
                faultBuf[errorByte] = faultBuf[errorByte] + ((unsigned char)0x01<<errorBit);
            else
                faultBuf[errorByte] = faultBuf[errorByte] - (unsigned char)(1<<errorBit);
            uint32_t tmpCrc;
            if(0 == (tmpCrc = gnua::crc(faultBuf, faultBufLen))){
                singleRandomBitFlipFailNum++;
            }
        }
        //flip a second bit
        {
            do{
                errorByteTwo = (int) ((double)faultBufLen * (rand_r(&randSeed) / (RAND_MAX + 1.0)));
                errorBitTwo = (int) (8.0 * (rand_r(&randSeed) / (RAND_MAX + 1.0)));
            }while (errorByte==errorByteTwo && errorBit==errorBitTwo);// make sure we don't flip the same bit twice
            if(0 == faultBuf[errorByteTwo] && (unsigned char)(1<<errorBitTwo))
                faultBuf[errorByteTwo] = faultBuf[errorByteTwo] + (unsigned char)(1<<errorBitTwo);
            else
                faultBuf[errorByteTwo] = faultBuf[errorByteTwo] - (unsigned char)(1<<errorBitTwo);
            uint32_t tmpCrc;
            if(0 == (tmpCrc = gnua::crc(faultBuf, faultBufLen))){
                doubleRandomBitFlipFailNum++;
            }
        }
        //flip a third bit
        {
            do{
                errorByteThree = (int) ((double)faultBufLen * (rand_r(&randSeed) / (RAND_MAX + 1.0)));
                errorBitThree = (int) (8.0 * (rand_r(&randSeed) / (RAND_MAX + 1.0)));
            }while ((errorByteThree==errorByte && errorBitThree==errorBit)
                    || (errorByteThree==errorByteTwo && errorBitThree==errorBitTwo));// make sure we don't flip the same bit twice
            if(0 == faultBuf[errorByteThree] && (unsigned char)(1<<errorBitThree))
                faultBuf[errorByteThree] = faultBuf[errorByteThree] + (unsigned char)(1<<errorBitThree);
            else
                faultBuf[errorByteThree] = faultBuf[errorByteThree] - (unsigned char)(1<<errorBitThree);
            uint32_t tmpCrc;
            if(0 == (tmpCrc = gnua::crc(faultBuf, faultBufLen))){
                tripleRandomBitFlipFailNum++;
            }
        }
        delete[] buf;
        delete[] faultBuf;
    }
    printf("Comparison to Novatel:\t%d tests / %d failures\n", k, novatelCompareFailNum);
    printf("False Positives:\t%d tests / %d failures\n", k, falsePositivesNum);
    printf("One Random bit flip:\t%d tests / %d failures\n", k, singleRandomBitFlipFailNum);
    printf("Two Random bit flips:\t%d tests / %d failures\n", k, doubleRandomBitFlipFailNum);
    printf("Three Random bit flips:\t%d tests / %d failures\n", k, tripleRandomBitFlipFailNum);
    if(novatelCompareFailNum
            || falsePositivesNum
            || singleRandomBitFlipFailNum
            || doubleRandomBitFlipFailNum
            || tripleRandomBitFlipFailNum
            ){
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}


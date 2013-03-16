/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Mathew Ridley, Alex Brooks, Alexei Makarenko, Tobias Kaupp
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */

#include <stdio.h>
#include <string>
#include <iostream>
#include <assert.h>
#include <sstream>
#include <gbxutilacfr/tokenise.h>

// //////////////////////////////

// // Ensure we have strnlen 
// // eg. Solaris doesn't define strnlen in string.h, so define it here.
// #if !HAVE_STRNLEN

// #include <cstring>

// // inline the fucker to guard against multiple inclusion, without the
// // hassle of a special lib.
// inline size_t strnlen(const char *s, size_t maxlen) 
// {
//     char *p;
//     if (s == NULL) {
//         return maxlen;
//     }
//     p = (char *)memchr(s, 0, maxlen);
//     if (p == NULL) {
//         return maxlen;
//     }
//     return ((p - s) + 1);
// }
// #endif

// //////////////////////////////

#include "nmeasentence.h"

using namespace std;
using namespace gbxgpsutilacfr;

const char NMEAStartOfSentence  = '$';
const char NMEAChecksumDelim    = '*';


//The blank constructor
NmeaSentence::NmeaSentence()
{
    init();
}

void NmeaSentence::init()
{
    haveCheckSum_ = false;
    checkSumOK_   = false;

    // Now clear the internal data store
    sentence_.clear();
    dataTokens_.clear();
}

NmeaSentence::NmeaSentence(const std::string &sentence, NmeaSentenceOptions addOrTestCheckSum)
{
    init();
    setSentence(sentence,addOrTestCheckSum);
}


//Load the data as requested and test the checksum if we are asked to.
void 
NmeaSentence::setSentence(const std::string &data, NmeaSentenceOptions addOrTestCheckSum)
{
    init();

    sentence_ = data;

    switch ( addOrTestCheckSum )
    {
        case TestChecksum:  { 
            // This is for Rx'd data that we need to test for correct reception
            // (internally it will also call addCheckSum())
            testChecksumOk(); 
            break;
        }
        case AddChecksum: {  
            // This is for Tx data that needs to checksummed before sending
            addCheckSum(); 
            checkSumOK_ = true; 
            break;
        }
        case DontTestOrAddChecksum: 
            break;
        default:
            assert( false && "unrecognized message option" );
    }
}

bool 
NmeaSentence::testChecksumOk()
{
    haveCheckSum_ = true;
    checkSumOK_   = false;

    //First save the existing two checksum chars from the message
    //These are straight after the '*' character
    const size_t starPos = sentence_.find( NMEAChecksumDelim );
    if ( starPos == std::string::npos )
    {
        // cout<<"device: no checksum delimiter"<<endl;
        return false;
    }

    if ( starPos+2 >= sentence_.size() )
    {
        // cout<<"device: no checksum after delimiter"<<endl;
        return false;
    }
   
    //save the high and low bytes of the checksum
    //Make sure they are in upper case!
    const int checksumPos = starPos+1;
    const char chksum_HIB = (char)toupper(sentence_[checksumPos]);
    const char chksum_LOB = (char)toupper(sentence_[checksumPos+1]);   

    //invalidate the existing checksum
    sentence_[checksumPos]   = 'x';
    sentence_[checksumPos+1] = 'x';

    //Re-calculate our own copy of the checksum
    addCheckSum();

    //Now compare our saved version with our new ones
    if( (chksum_HIB == sentence_[checksumPos]) && (chksum_LOB == sentence_[checksumPos+1]) ) {
        //all looked good!
        checkSumOK_ = true;
        return true;
    }
   
    //failed the checksum!
//     cout<<"device: '"<<chksum_HIB<<chksum_LOB   <<"' ("<<std::hex<<(unsigned int)chksum_HIB<<","<<(unsigned int)chksum_LOB<<std::dec<<") "
//         <<"driver: '"<<*ptr<<*(ptr+1)<<"'"      <<"' ("<<std::hex<<(unsigned int)*ptr<<","<<(unsigned int)*(ptr+1)<<std::dec<<") "<<endl;
    return false;
}

// Add the checksum chars to an existing message
// NOTE: this assumes that there is allready space in the message for
// the checksum, and that the checksum delimiter is there
void 
NmeaSentence::addCheckSum()
{
    assert( haveSentence() && "calling addCheckSum() without a sentence" );

    haveCheckSum_ = true;

    //check that we have the '$' at the start
    if ( sentence_[0]!= NMEAStartOfSentence ) {
        throw NmeaException("cannot calculate checksum, missing leading '$'");
    }

    unsigned char chkRunning = 0;
    
    // we start from 1 to skip the leading '$'
    int loopCount;
    for ( loopCount = 1; loopCount < (int)(sentence_.size()); loopCount++ )
    { 
        unsigned char nextChar = static_cast<unsigned char>(sentence_[loopCount]);
    
        // no delimiter uh oh
        if( (nextChar=='\r') || (nextChar=='\n') || (nextChar=='\0') ) {
            throw NmeaException("cannot calculate checksum, missing final '*'");
        }
		
        // goodie we found it (the '*' is not included into the checksum)
        if ( nextChar==NMEAChecksumDelim ) {
            break;
        }
    
        // alexm: gcc-4.3 is very strict with the next line.
        // not sure what to do about it yet.
        // http://gcc.gnu.org/ml/gcc/2008-05/msg00363.html

        // Keep the running XOR total
        chkRunning ^= nextChar;
    }

    if ( loopCount+2 >= (int)(sentence_.size()) )
    {
        throw NmeaException("addCheckSum(): no space for checksum of '*' not found.");
    }
        
    //Put the byte values as upper case HEX back into the message
    sprintf( &(sentence_[loopCount + 1]),"%02X", chkRunning );
}

// Parse the data fields of our message...
void 
NmeaSentence::parseTokens()
{
    //We should not attempt to be parsing a message twice...
    assert( numDataTokens()==0 && "calling parseTokens() with tokens" );

    //Split the message at the commas
    //TODO cope with missing fields
    dataTokens_ = gbxutilacfr::tokenise(sentence_, ",");
    
    //Now discard the $ and the * from the first and last tokens...
    //TODO : - dataTokens_[0] = 

    //keep track of what we have done.
    haveTokens_ = true;
}

bool
NmeaSentence::isDataTokenEmpty(int i) const
{
    if ( i >= (int)(dataTokens_.size()) )
    {
        stringstream ss;
        ss << "NmeaSentence::" << __func__ 
           << ": attempt to getDataToken("<<i<<") but only " << dataTokens_.size() << " exist in sentence: "
           << sentence_;
        throw NmeaException( ss.str() );
    }
    return dataTokens_[i].empty();    
}

const std::string &
NmeaSentence::getDataToken(int i) const
{
    if ( i >= (int)(dataTokens_.size()) )
    {
        stringstream ss;
        ss << "NmeaSentence::" << __func__ 
           << ": attempt to getDataToken("<<i<<") but only " << dataTokens_.size() << " exist in sentence: "
           << sentence_;
        throw NmeaException( ss.str() );
    }
    if ( dataTokens_[i].empty() )
    {
        stringstream ss;
        ss << "NmeaSentence::" << __func__ 
           << ": attempt to getDataToken("<<i<<") but this token is empty in sentence: "
           << sentence_;
        throw NmeaException( ss.str() );        
    }
    return dataTokens_[i];
}

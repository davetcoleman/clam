/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010  Alex Brooks, Alexei Makarenko, Tobias Kaupp, Duncan Mercer
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */

#ifndef GBXGPSUTILACFR_NMEA_H
#define GBXGPSUTILACFR_NMEA_H

#include <vector>
#include <string>

/*

for further info:

http://www.kh-gps.de/nmea-faq.htm
http://vancouver-webpages.com/peter/nmeafaq.txt

NMEA-0183 sentence

$aaccc,c--c*hh<CR><LF>
||    ||   || |
||    ||   || \________ <CR><LF> - End of sentence (0xOD 0xOA)
||    ||   |\__________ hh    - Checksum field hexadecimal [optional]
||    ||   \___________ *     - Checksum delimiter (0x2A) [optional]
||    |\_______________ c--c  - Data sentence block
||    \________________ ,     - Field delimiter (0x2c)
|\_____________________ aaccc - Address field/Command
\______________________ $     - Start of sentence

        The optional checksum field consists of a "*" and two hex digits
        representing the exclusive OR of all characters between, but not
        including, the "$" and "*".  A checksum is required on some
        sentences.

*/

namespace gbxgpsutilacfr {


// class SOEXPORT NmeaException : public std::exception
class NmeaException : public std::exception
{
public:
    NmeaException(const char *message)
        : message_(message) {}

    NmeaException(const std::string &message)
        : message_(message) {}

    virtual ~NmeaException() throw() {}

    virtual const char* what() const throw() { return message_.c_str(); }

protected:
    std::string  message_;
};


#define MAX_SENTENCE_LEN 256

// When using class to send data, need to add checksum, when reciving data need to test checksum
// Checksums are usually optional 
enum NmeaSentenceOptions
{
    TestChecksum, 
    AddChecksum, 
    DontTestOrAddChecksum
};

// class SOEXPORT NmeaSentence{
class NmeaSentence
{
public:
    NmeaSentence();
    NmeaSentence(const std::string &sentence, NmeaSentenceOptions addOrTestCheckSum=DontTestOrAddChecksum );

    // Set up the internal data for a sentence.
    // May throw NmeaException if TestChecksum is specified.
    void setSentence(const std::string &data, NmeaSentenceOptions addOrTestCheckSum=DontTestOrAddChecksum );

    // Do we have the raw string?
    bool haveSentence() const { return !sentence_.empty(); };

    // Do we have parsed fields?
    bool haveTokens() const { return haveTokens_; };

    // Do we have a valid checksum?
    bool haveValidChecksum() const { return checkSumOK_; };  
  
    // Have we checked the checksum?
    bool haveTestedChecksum()const { return haveCheckSum_; };  
  
    // calculate the checksum from sentence
    // May throw NmeaException.
    bool testChecksumOk();

    // Return the raw sentence string
    const std::string &sentence() const { return sentence_; };

    // Return a single data token as a string.
    // Throws an exception if that token is empty (see 'isDataTokenEmpty()')
    const std::string &getDataToken(int i) const;

    bool isDataTokenEmpty(int i) const;

    // Return the number of fields
    int numDataTokens() const { return dataTokens_.size(); };

    // Tokenise the string that we received
    void parseTokens();

private:
    void init();
    // May throw NmeaException.
    void addCheckSum();
    // Have we parsed data into tokens ?
    bool haveTokens_;
    // Have we a checksum and is it valid?
    bool haveCheckSum_;
    bool checkSumOK_;
    // The raw sentence, allow for terminator
//    char sentence_[MAX_SENTENCE_LEN+1];
    std::string sentence_;
    // The tokenised data
    std::vector<std::string> dataTokens_;
};

}

#endif

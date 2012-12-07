/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Michael Moser
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */
#include <stdlib.h>
#include <unistd.h>

#include <gbxserialacfr/gbxserialacfr.h>
#include <string>
#include <vector>
#include <iostream>
#include <assert.h>

#include "serialconnectivity.h"

using namespace gbxserialacfr;

namespace gbxnovatelutilacfr{
// test connectivity to a [serial] device at a [baudrate];
// Assumes that you can figure out a [challenge] (e.g. a command) to
// which the device will answer with a unique [ack] in [timeOutMsec] milliseconds.
//
// If [successThresh] or more [challenge]es are answered by an [ack] we assume
// we got the correct baudrate.
//
// returns 0 for Success; -1 for failure
bool
testConnectivity(
        std::string &challenge,
        std::string &ack,
        gbxserialacfr::Serial& serial,
        int timeOutMsec,
        int numTry,
        int successThresh,
        int baudrate){
    std::cout << "Testing connectivity at " << baudrate << " bps: ";
    try{
        serial.setBaudRate(baudrate);
        serial.flush();
    }
    catch( SerialException &e ){
        std::cerr <<"Caught SerialException: " << e.what()<<"\n";
        return false;
    }
    catch( std::exception &e ){
        std::cerr <<"Caught std::exception: " << e.what()<<"\n";
        return false;
    }
    catch(...){
        std::cerr <<"Caught unknown Exception :-(\n";
        return false;
    }

    int successCnt = 0;
    std::string errorResponse;
    for(int i=0; i<numTry; i++){
        if(true == sendCmdWaitForResponse( challenge, ack, errorResponse, serial, timeOutMsec)){
            successCnt++;
        }
    }
    std::cout << successCnt << "/"<< numTry << " ";
    if(successThresh <= successCnt){
        std::cout << "OK\n";
        return true;
    }else{
        std::cout << "Fail\n";
        return false;
    }
}

bool sendCmdWaitForResponse(
        std::string &challenge,
        std::string &ack,
        std::string &errorResponse,
        gbxserialacfr::Serial& serial,
        int timeOutMsec){
    // send challenge
    serial.writeString(challenge.c_str());
    usleep(timeOutMsec*1000);

    //read response
    int available;
    bool success = false;
    available = serial.bytesAvailable();
    if(0<available){
        char *buf;
        buf = new char[available+1];
        int read = serial.read(buf, available);
        assert( read != -1 );

        //and look for an ACK
        //it's possible that we have read binary data with embedded '\0'; we want one at the end (string delimeter), but replace the rest with '\1'
        buf[read] = '\0';
        while(0<read){
            read--;
            if('\0' == buf[read]){
                buf[read] = '\1';
            }
        }
        std::string response(buf);
        size_t found = response.find(ack);
        if(std::string::npos != found){
            success = true;
        }else{
            errorResponse = response;
        }
        delete []buf;
    }
    return success;
}
}//namespace

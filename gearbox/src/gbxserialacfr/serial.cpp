/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Alex Brooks, Mathew Ridley
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */

#include <termios.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>    // read and write to ports
#include <string.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <assert.h>
#include "serial.h"

#if __linux
#  include <linux/serial.h>
#endif

#include <config.h>
#if HAVE_FILIO_H
// FIONREAD
#  include <sys/filio.h>
#endif

//////////////////////////////

// Ensure we have strnlen
// eg. Solaris doesn't define strnlen in string.h, so define it here.
#if !HAVE_STRNLEN

#include <cstring>

// inline the fucker to guard against multiple inclusion, without the
// hassle of a special lib.
inline size_t strnlen(const char *s, size_t maxlen)
{
    char *p;
    if (s == NULL) {
        return maxlen;
    }
    p = (char *)memchr(s, 0, maxlen);
    if (p == NULL) {
        return maxlen;
    }
    return ((p - s) + 1);
}
#endif

//////////////////////////////

using namespace std;

namespace gbxserialacfr {

    namespace {

        std::string toHexAndAscii( const char *str, int len )
        {
            stringstream ss;
            ss << "'";
            for ( int i=0; i < len; i++ )
            {
                ss << str[i];
            }
            ss << "' ( ";
            // ss << "' ("<<std::hex<<std::setfill('0')<<std::setw(3);
            for ( int i=0; i < len; i++ )
            {
                char buf[3];
                sprintf(buf,"%02x",((int)((unsigned char)(str[i]))));
                ss << buf << " ";
            }
            ss << ")";
            return ss.str();
        }

        // Converts an integer baud-rate into a c-style '#define'd baudrate
        int cBaudrate( int baudRate )
        {
            switch(baudRate)
            {
            case 0:
                return B0;
            case 50:
                return B50;
            case 75:
                return B75;
            case 110:
                return B110;
            case 134:
                return B134;
            case 150:
                return B150;
            case 200:
                return B200;
            case 300:
                return B300;
            case 600:
                return B600;
            case 1200:
                return B1200;
            case 1800:
                return B1800;
            case 2400:
                return B2400;
            case 4800:
                return B4800;
            case 9600:
                return B9600;
            case 19200:
                return B19200;
            case 38400:
                return B38400;
            case 57600:
                return B57600;
            case 115200:
                return B115200;
#ifdef __linux
            case 230400:
                return B230400;
            case 460800:
                return B460800;
            case 500000:
                return B500000;
            case 576000:
                return B576000;
            case 921600:
                return B921600;
            case 1000000:
                return B1000000;
            case 1152000:
                return B1152000;
            case 1500000:
                return B1500000;
            case 2000000:
                return B2000000;
            case 2500000:
                return B2500000;
            case 3000000:
                return B3000000;
            case 3500000:
                return B3500000;
            case 4000000:
                return B4000000;
#endif
            default:
                stringstream ss;
                ss << "Serial::"<<__func__<<"() Invalid baud rate: " << baudRate;
                throw SerialException( ss.str() );
            }
        }

        int iBaudrate( int baudRate )
        {
            switch(baudRate)
            {
            case B0:
                return 0;
            case B50:
                return 50;
            case B75:
                return 75;
            case B110:
                return 110;
            case B134:
                return 134;
            case B150:
                return 150;
            case B200:
                return 200;
            case B300:
                return 300;
            case B600:
                return 600;
            case B1200:
                return 1200;
            case B1800:
                return 1800;
            case B2400:
                return 2400;
            case B4800:
                return 4800;
            case B9600:
                return 9600;
            case B19200:
                return 19200;
            case B38400:
                return 38400;
            case B57600:
                return 57600;
            case B115200:
                return 115200;
            case B230400:
                return 230400;
            case B460800:
                return 460800;
            case B500000:
                return 500000;
            case B576000:
                return 576000;
            case B921600:
                return 921600;
            case B1000000:
                return 1000000;
            case B1152000:
                return 1152000;
            case B1500000:
                return 1500000;
            case B2000000:
                return 2000000;
            case B2500000:
                return 2500000;
            case B3000000:
                return 3000000;
            case B3500000:
                return 3500000;
            case B4000000:
                return 4000000;
            default:
                stringstream ss;
                ss << "Serial::"<<__func__<<"() Invalid baud rate: " << baudRate;
                throw SerialException( ss.str() );
            }
        }

        //Allow streaming of the term status structure type
        std::ostream &operator<<( std::ostream &s, struct termios &stat )
        {
            s.setf(ios::hex,ios::basefield);
            s.fill('0');

            s << "c_iflag -> 0x"  << setw(4) <<  stat.c_iflag << endl;
            s << "c_oflag -> 0x"  << setw(4) <<  stat.c_oflag << endl;
            s << "c_cflag -> 0x"  << setw(4) <<  stat.c_cflag << endl;
            s << "c_lflag -> 0x"  << setw(4) <<  stat.c_lflag << endl;

            s << "c_cc_array ->" << endl;
            s << "VINTR 0x"      << setw(2) <<  static_cast<int>(stat.c_cc[VINTR])
              << ", VQUIT 0x"    << setw(2) <<  static_cast<int>(stat.c_cc[VQUIT])
              << ", VERASE 0x"   << setw(2) <<  static_cast<int>(stat.c_cc[VERASE])
              << ", VKILL 0x"    << setw(2) <<  static_cast<int>(stat.c_cc[VKILL]) << endl ;
            s << "VEOF 0x"       << setw(2) <<  static_cast<int>(stat.c_cc[VEOF])
              << ", VEOL 0x"     << setw(2) <<  static_cast<int>(stat.c_cc[VEOL])
              << ", VEOL2 0x"    << setw(2) <<  static_cast<int>(stat.c_cc[VEOL2])
              << ", VSTART 0x"   << setw(2) <<  static_cast<int>(stat.c_cc[VSTART]) << endl ;
            s << "VSTOP 0x"      << setw(2) <<  static_cast<int>(stat.c_cc[VSTOP])
              << ", VSUSP 0x"    << setw(2) <<  static_cast<int>(stat.c_cc[VSUSP])
              << ", VREPRINT 0x" << setw(2) <<  static_cast<int>(stat.c_cc[VREPRINT])
              << ", VLNEXT 0x"   << setw(2) <<  static_cast<int>(stat.c_cc[VLNEXT]) << endl;
            s << "VMIN 0x"       << setw(2) <<  static_cast<int>(stat.c_cc[VMIN])
              << ", VTIME 0x"    << setw(2) <<  static_cast<int>(stat.c_cc[VTIME]) << endl;


            s << "c_iflag, Bits set" << endl;
            if (stat.c_iflag & IGNBRK) { s << "IGNBRK,";}
            if (stat.c_iflag & BRKINT) { s << "BRKINT,";}
            if (stat.c_iflag & IGNPAR) { s << "IGNPAR,";}
            if (stat.c_iflag & PARMRK) { s << "PARMRK,";}
            if (stat.c_iflag & INPCK)  { s << "INPCK,";}
            if (stat.c_iflag & ISTRIP) { s << "ISTRIP,";}
            if (stat.c_iflag & INLCR)  { s << "INLCR,";}
            if (stat.c_iflag & IGNCR)  { s << "IGNCR,";}
            if (stat.c_iflag & ICRNL)  { s << "ICRNL,";}
            if (stat.c_iflag & IUCLC)  { s << "IUCLC,";}
            if (stat.c_iflag & IXON)   { s << "IXON,";}
            if (stat.c_iflag & IXANY)  { s << "IXANY,";}
            if (stat.c_iflag & IXOFF)  { s << "IXOFF,";}
            if (stat.c_iflag & IMAXBEL){ s << "IMAXBEL,";}
            s << endl;

            s << "c_oflag, Bits set" << endl;
            if (stat.c_oflag & OPOST){ s << "OPOST,";}
            if (stat.c_oflag & OLCUC){ s << "OLCUC,";}
            if (stat.c_oflag & ONLCR){ s << "ONLCR,";}
            if (stat.c_oflag & OCRNL){ s << "OCRNL,";}
            if (stat.c_oflag & ONOCR){ s << "ONOCR,";}
            if (stat.c_oflag & ONLRET){ s << "ONLRET,";}
            if (stat.c_oflag & OFILL){ s << "OFILL,";}
            if (stat.c_oflag & OFDEL){ s << "OFDEL,";}
            if (stat.c_oflag & NLDLY){ s << "NLDLY,";}
            if (stat.c_oflag & CRDLY){ s << "CRDLY,";}
            if (stat.c_oflag & TABDLY){ s << "TABDLY,";}
            if (stat.c_oflag & BSDLY){ s << "BSDLY,";}
            if (stat.c_oflag & VTDLY){ s << "VTDLY,";}
            if (stat.c_oflag & FFDLY){ s << "FFDLY,";}
            s << endl;

            s << "c_cflag, Bits set" << endl;
            if (stat.c_cflag & CSTOPB){ s << "CSTOPB,";}
            if (stat.c_cflag & CREAD){ s << "CREAD,";}
            if (stat.c_cflag & PARENB){ s << "PARENB,";}
            if (stat.c_cflag & PARODD){ s << "PARODD,";}
            if (stat.c_cflag & HUPCL){ s << "HUPCL,";}
            if (stat.c_cflag & CLOCAL){ s << "CLOCAL,";}
#if defined(CIBAUD)
            if (stat.c_cflag & CIBAUD){ s << "CIBAUD,";}
#endif
            s << endl;


            s << "c_lflag, Bits set" << endl;
            if (stat.c_lflag & ISIG){ s << "ISIG,";}
            if (stat.c_lflag & ICANON){ s << "ICANON,";}
            if (stat.c_lflag & PARENB){ s << "PARENB,";}
            if (stat.c_lflag & XCASE){ s << "XCASE,";}
            if (stat.c_lflag & NOFLSH){ s << "NOFLSH,";}
            if (stat.c_lflag & FLUSHO){ s << "FLUSHO,";}
            if (stat.c_lflag & PENDIN){ s << "PENDIN,";}
            if (stat.c_lflag & IEXTEN){ s << "IEXTEN,";}
            if (stat.c_lflag & TOSTOP){ s << "TOSTOP,";}
            if (stat.c_lflag & ECHO){ s << "ECHO,";}
            if (stat.c_lflag & ECHOE){ s << "ECHOE,";}
            if (stat.c_lflag & ECHOPRT){ s << "ECHOPRT,";}
            if (stat.c_lflag & ECHOKE){ s << "ECHOKE,";}
            if (stat.c_lflag & NOFLSH){ s << "ECHONL,";}
            if (stat.c_lflag & FLUSHO){ s << "ECHOCTL,";}
            s << endl;

            s << endl;
            s.setf(ios::dec,ios::basefield);
            return s;
        }


        std::ostream &operator<<( std::ostream &s, struct serial_struct &serinfo )
        {
            s.setf(ios::hex,ios::basefield);

            s << "TIOCGSERIAL:-> " << endl;
            s << "Flags: 0x"            << setw(2) << serinfo.flags
              << ", Type: 0x"           << setw(2) << serinfo.type
              << ", Line: 0x"           << setw(2) << serinfo.line << endl;
            s << "Port: 0x"             << setw(2) << serinfo.port
              << ", IRQ: 0x"            << setw(2) << serinfo.irq
              << ", xmit_fifo_size: 0x" << setw(2) << serinfo.xmit_fifo_size << endl;
            s << "Baud_base: 0x"        << setw(2) << serinfo.baud_base
              << ", Custom_divisor: 0x" << setw(2) << serinfo.custom_divisor
              << ", Io_type: 0x"        << setw(2) << serinfo.io_type << endl;

            s.setf(ios::dec,ios::basefield);
            return s;
        }

    }

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

Serial::Serial( const std::string &dev,
                int                baudRate,
                const Timeout     &timeout,
                int                debuglevel,
                bool               useLockFile )
    : dev_(dev),
      portFd_(-1),
      timeout_(timeout),
      debugLevel_(debuglevel)
{
    if ( useLockFile )
    {
        try {
            lockFile_ = new lockfile::LockFile( dev );
        }
        catch ( const lockfile::LockedByOtherProcessException &e )
        {
            stringstream ss;
            ss << "Couldn't get lock for device " << dev << ": " << e.what();
            throw lockfile::LockedByOtherProcessException( ss.str() );
        }
        catch ( const lockfile::LockFileException &e )
        {
            stringstream ss;
            ss << "Couldn't get lock for device " << dev << ": " << e.what();
            throw lockfile::LockFileException( ss.str() );
        }
    }

    try {
        open();
        setBaudRate( baudRate );

        if(debugLevel_ > 2){
            cout << "At end of Serial::Serial(): " << getStatusString();
        }
    }
    catch ( const SerialException &e )
    {
        if ( lockFile_ ) delete lockFile_;
        throw;
    }
}

Serial::~Serial()
{
    close();
    if ( lockFile_ ) delete lockFile_;
}

void
Serial::setTimeout( const Timeout &timeout )
{
    bool newTimeoutsEnabled = !( timeout.sec == 0 && timeout.usec == 0 );
    if ( timeoutsEnabled() != newTimeoutsEnabled )
    {
        stringstream ss;
        ss << "setTimeout() for port " << dev_
           <<" tried to change enabled state of timeouts!  Timeouts previously enabled: " << timeoutsEnabled();
        throw SerialException( ss.str() );
    }
    timeout_ = timeout;
}

void
Serial::close() throw()
{
    if ( debugLevel_ > 0 )
    {
        // AlexB: Not sure if cout::operator<< can throw, be careful just in case.
        try {
            cout<<"TRACE(serial.cpp): "<<__func__<<"()" << endl;
        }
        catch ( ... ) {}
    }

    assert( portFd_ != -1 );

    if(tcdrain(portFd_))
    {
        stringstream ss;
        ss << "Serial::"<<__func__<<"():tcdrain()";
        perror(ss.str().c_str());
    }
    if (::close(portFd_))
    {
        stringstream ss;
        ss << "Serial::"<<__func__<<"():close()";
        perror(ss.str().c_str());
    }

    // Make sure that we force this back to an invalid state
    portFd_ = -1;
}

void
Serial::setBaudRate(int baud)
{
    struct termios localOptions;

    if ( debugLevel_ > 0 )
        cout<<"TRACE(serial.cpp): "<<__func__<<"("<<baud<<")" << endl;

    if(portFd_==-1)
    {
        stringstream ss;
        ss << "Serial::"<<__func__<<"(): no valid device open";
        throw SerialException( ss.str() );
    }
    if(tcgetattr(portFd_, &localOptions) == -1)
    {
        stringstream ss;
        ss << "Serial::"<<__func__<<"():tcgetattr() Error reading attr: " << strerror(errno);
        throw SerialException( ss.str() );
    }

    cfsetispeed(&localOptions, cBaudrate(baud));
    cfsetospeed(&localOptions, cBaudrate(baud));

    //
    // AlexB: This code doesn't seem to work for USB devices...
    //        See: http://ozlabs.org/pipermail/linuxppc-embedded/2005-February/016848.html
    //
    if ( strstr( dev_.c_str(), "USB" ) == 0 )
    {
        //
        // AlexB: TODO: fix this properly:
        //        For reasons I don't fully understand, this chunk is required to make the
        //        laser work at standard baud rates.
        //
        struct serial_struct  serinfo;
        serinfo.reserved_char[0] = 0;
        if (ioctl(portFd_, TIOCGSERIAL, &serinfo) < 0)
        {
            stringstream ss;
            ss << "Serial::"<<__func__<<"("<<baud<<"): error calling 'ioctl(portFd_, TIOCGSERIAL, &serinfo)': "<<strerror(errno);
            throw SerialException( ss.str() );
        }

        serinfo.flags &= ~ASYNC_SPD_CUST;
        serinfo.custom_divisor = 0;

        if (ioctl(portFd_, TIOCSSERIAL, &serinfo) < 0)
        {
            stringstream ss;
            ss << "Serial::"<<__func__<<"("<<baud<<"): error calling 'ioctl(portFd_, TIOCSSERIAL, &serinfo)': "<<strerror(errno);
            throw SerialException( ss.str() );
        }
    }

    if ( tcsetattr(portFd_, TCSAFLUSH, &localOptions) == -1 )
    {
        stringstream ss;
        ss << "Serial::"<<__func__<<"():tcsetattr() Error setting attr: " << strerror(errno);
        throw SerialException( ss.str() );
    }
}

void
Serial::open(int flags)
{
    struct termios localOptions;

    if ( timeoutsEnabled() )
        flags |= O_NONBLOCK;

    portFd_ = ::open(dev_.c_str(), flags|O_RDWR|O_NOCTTY);
    if ( portFd_ == -1 )
    {
        stringstream ss;
        ss << "Serial::"<<__func__<<"(): failed to open '"<<dev_<<"': "<<strerror(errno);
        throw SerialException( ss.str() );
    }


    if(tcgetattr(portFd_, &localOptions) == -1)
    {
        close();
        stringstream ss;
        ss << "Serial::"<<__func__<<"(): tcgetattr() failed for '"<<dev_<<"': "<<strerror(errno);
        throw SerialException( ss.str() );
    }

    if(debugLevel_ > 2){
        cout << "At beginning of "<<__func__<<"(): "<<getStatusString()<<endl;
    }

    // enable receiver & ignore control lines
    localOptions.c_cflag |=  (CLOCAL | CREAD) ;

    // set 8 data bits
    localOptions.c_cflag &= ~CSIZE;
    localOptions.c_cflag |= CS8;

    // set parity to none with no stop bit
    localOptions.c_cflag &= ~PARENB;
    localOptions.c_cflag &= ~CSTOPB;

    // disable hardware flow control
    localOptions.c_cflag &= ~CRTSCTS;

#if defined(__sun)
    // http://www.sunmanagers.org/pipermail/summaries/2005-October/006871.html
    localOptions.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL|IXON);
    localOptions.c_oflag &= ~OPOST;
    localOptions.c_lflag &= ~(ECHO|ECHONL|ICANON|ISIG|IEXTEN);
    localOptions.c_cflag &= ~(CSIZE|PARENB);
    localOptions.c_cflag |= CS8;
#else
    cfmakeraw(&localOptions);
#endif

    if(tcsetattr(portFd_, TCSAFLUSH, &localOptions) == -1)
    {
        close();
        stringstream ss;
        ss << "Serial::"<<__func__<<"(): tcsetattr() failed for '"<<dev_<<"': "<<strerror(errno);
        throw SerialException( ss.str() );
    }

    if ( debugLevel_ > 2 ){
        cout << "At end of "<<__func__<<"():" << getStatusString();
    }
}

int
Serial::read(void *buf, int count)
{
    if ( debugLevel_ > 0 )
        cout<<"TRACE(serial.cpp): "<<__func__<<"()" << endl;

    int got = ::read(portFd_, buf, count);
    if ( got < 0 )
    {
        stringstream ss;
        ss << "Serial::"<<__func__<<"(): " << strerror(errno);
        throw SerialException( ss.str() );
    }

    if ( debugLevel_ > 1 )
    {
        cout<<"TRACE(serial.cpp): just read: "<<toHexAndAscii((char *)buf,count)<<endl;
//         for ( int i=0; i < count; i++ )
//             cout << ((char*)(buf))[i];
//         cout << "'" << endl;
    }

    return got;
}



int
Serial::readFull(void *buf, int count)
{
    if ( debugLevel_ > 0 )
        cout<<"TRACE(serial.cpp): "<<__func__<<"(): count=" << count << endl;

    char* bufPtr = static_cast<char*>(buf);

    int got=0;
    while ( got < count )
    {
        char *offset = bufPtr + got;
        int ret = ::read(portFd_, offset, count-got);
        if ( ret >= 0 )
        {
            got += ret;
        }
        else if (timeoutsEnabled() && (errno == EAGAIN) )
        {
            if ( waitForDataOrTimeout() == TIMED_OUT )
            {
                // select timed out: no data
                return -1;
            }
        }

        else
        {
            stringstream ss;
            ss << "Serial::"<<__func__<<": read(): "<<strerror(errno);
            throw SerialException( ss.str() );
        }

    }

    return got; //The number of bytes that we read.
}

int
Serial::readStringUntil( std::string &str, char termchar )
{
    if ( debugLevel_ > 0 ){
        cout<<"TRACE(serial.cpp): "<<__func__<<"(): ";
        if(timeoutsEnabled()){
            cout << "timeouts enabled"<<endl;
        }else{
            cout << "timeouts not enabled"<<endl;
        }
    }

    // clear the string
    str="";

    while ( true )
    {
        // Read at most a single character
        char c;
        int ret = ::read( portFd_, &c, 1 );
        if ( ret == 1 )
        {
            str += c;
            if ( c == termchar )
                return str.size();
        }
        else if ( ret == 0 )
        {
            // Nothing to read yet
            if ( ( timeoutsEnabled() && waitForDataOrTimeout() == TIMED_OUT ) ||
                 !timeoutsEnabled() )
            {
                // Timed out
                return -1;
            }
        }
        else // ret==-1: error
        {
            if ( timeoutsEnabled() && errno == EAGAIN )
            {
                if ( waitForDataOrTimeout() == TIMED_OUT )
                    return -1;
            }
            else
            {
                stringstream ss;
                ss << "Serial::"<<__func__<<"(): "<<strerror(errno);
                throw SerialException( ss.str() );
            }
        }
    }
    return str.size();
}

int
Serial::bytesAvailable()
{
    int n_read;
    int ret = ioctl(portFd_,FIONREAD,&n_read);

    if(ret==-1)
    {
        stringstream ss;
        ss << "Serial::"<<__func__<<": "<< strerror(errno);
        throw SerialException( ss.str() );
    }
    return n_read;
}

int
Serial::bytesAvailableWait()
{
    if ( waitForDataOrTimeout() == TIMED_OUT )
    {
        return -1;
    }

    return bytesAvailable();
}


Serial::WaitStatus
Serial::waitForDataOrTimeout()
{
    fd_set rfds;
    struct timeval tv;
    FD_ZERO(&rfds);
    FD_SET(portFd_, &rfds);
    tv.tv_sec = timeout_.sec;
    tv.tv_usec = timeout_.usec;
    int selval = select(portFd_+1, &rfds, NULL, NULL, &tv);
    if(selval==0)
    {
        // select timed out: no data
        return TIMED_OUT;
    }
    if(selval<0)
    {
        stringstream ss;
        ss << "Serial::"<<__func__<<": select(): "<<strerror(errno);
        throw SerialException( ss.str() );
    }

    return DATA_AVAILABLE;
}


int
Serial::writeString(const char *str)
{
    if ( debugLevel_ > 0 )
        cout<<"TRACE(serial.cpp): "<<__func__<<"(): writing: "<<toHexAndAscii(str,strlen(str))<<endl;

    int put;
    put = ::write(portFd_, str, strlen(str) );
    if ( put < 0 )
    {
        stringstream ss;
        ss << "Serial::"<<__func__<<"(): " << strerror(errno);
        throw SerialException( ss.str() );
    }
    else if ( put == 0 )
    {
        stringstream ss;
        ss << "Serial::"<<__func__<<"(): ::write() returned 0";
        throw SerialException( ss.str() );
    }
    else if ( put < (int)(strlen(str)) )
    {
        // AlexB: Not sure what to do here...  This can happen eg if the buffer is full.
        //        I'm not convinced that we want to throw an exception, but chances are
        //        lots of users won't check the return code.
        cout << "WARNING: Serial::"<<__func__<<": only wrote " << put << " of " << strlen(str) << " bytes." << endl;
    }

    if ( debugLevel_ > 1 )
    {
        cout<<"TRACE(serial.cpp): wrote " << put << " bytes" << endl;
    }

    return put;
}


std::string
Serial::getStatusString()
{
    struct termios status;
    if(tcgetattr(portFd_, &status) == -1)
    {
        close();
        stringstream ss;
        ss << "Serial::"<<__func__<<"(): tcgetattr():"<<strerror(errno);
        throw SerialException( ss.str() );
    }

    stringstream ss;
    ss << endl << " Device " << dev_ << " Status:-" << endl;
    ss << "In baud_rate: " << iBaudrate(cfgetispeed(&status))
         << " Out baud_rate: " << iBaudrate(cfgetospeed(&status)) << endl;
    ss << status;

    struct serial_struct  serinfo;
    if (ioctl(portFd_, TIOCGSERIAL, &serinfo) < 0)
    {
        stringstream ss;
        ss << "Serial::"<<__func__<<"(): error calling 'ioctl(portFd_, TIOCGSERIAL, &serinfo)': "<<strerror(errno);
        throw SerialException( ss.str() );
    }
    ss << serinfo;

    return ss.str();
}


void
Serial::flush()
{
    int ret = tcflush(portFd_,TCIOFLUSH);
    if ( ret < 0 )
    {
        stringstream ss;
        ss << "Serial::"<<__func__<<"(): "<<strerror(errno);
        throw SerialException( ss.str() );
    }
}

void
Serial::drain()
{
    // wait till all output sent
    if(tcdrain(portFd_))
    {
        stringstream ss;
        ss << "Serial::"<<__func__<<"(): tcdrain: "<<strerror(errno);
        throw SerialException( ss.str() );
    }
}

int
Serial::write(const void *buf, int count)
{
    if ( debugLevel_ > 0 )
        cout<<"TRACE(serial.cpp): "<<__func__<<"()" << endl;

    if ( count == 0 )
    {
        stringstream ss;
        ss << "Serial:: "<<__func__<<"() was called with zero bytes";
        throw SerialException( ss.str() );
    }

    int put = ::write(portFd_, buf, count);
    if ( put < 0 )
    {
        stringstream ss;
        ss << "Serial:: "<<__func__<<"(): "<<strerror(errno);
        throw SerialException( ss.str() );
    }
    else if ( put == 0 )
    {
        stringstream ss;
        ss << "Serial::"<<__func__<<"(): ::write() returned 0";
        throw SerialException( ss.str() );
    }
    if ( debugLevel_ > 1 )
    {
        cout<<"TRACE(serial.cpp): wrote " << put << " bytes: "<<toHexAndAscii((char *)buf,count)<<endl;
    }

    return put;
}

} // namespace

/*
 * GearBox Project: Peer-Reviewed Open-Source Libraries for Robotics
 *               http://gearbox.sf.net/
 * Copyright (c) 2004-2010 Alex Brooks
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */
#include "lockfile.h"
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <signal.h>
#include <sys/types.h>
#include <errno.h>
#include <sstream>
#include <libgen.h>
#include <stdio.h>

using namespace std;

namespace gbxserialacfr {
namespace lockfile {

namespace {

    const char *LOCK_DIR = "/var/lock";

    // This function is inefficient, but avoids
    // all the weirdness of basename().
    std::string getBasename( const char *path )
    {
        char *pathCopy = strdup( path );
        char *base = basename( pathCopy );
        std::string baseString = base;
        free( pathCopy );
        return baseString;
    }
    
    void
    lock_dev(const char *dev, int lpid)
    {
        FILE *fd;
        char  pbuf[260], lbuf[260];
        int   ret;

        std::string devBase = getBasename( dev );

        sprintf(pbuf, "%s/PID..%d", LOCK_DIR, getpid());
        sprintf(lbuf, "%s/LCK..%s", LOCK_DIR, devBase.c_str());

        // Create a file with our PID
        fd = fopen(pbuf, "w");
        if (fd == 0)
        {
            stringstream ss;
            ss << "Failed to open file '"<<pbuf<<"' for writing: "<<strerror(errno);
            throw LockFileException( ss.str() );
        }
        fprintf(fd, "%10d\n", lpid);
        ret = fclose(fd);
        if ( ret != 0 )
        {
            stringstream ss; ss << "Problem closing lockfile '"<<pbuf<<"': " << strerror(errno);
            throw LockFileException( ss.str() );
        }

        // Create a file for the device
        if (link(pbuf, lbuf) < 0)
        {
            // A file already exists for the device...  Have a look at the PID
            fd = fopen(lbuf, "r");
            if ( fd == 0 )
            {
                unlink(pbuf);
                stringstream ss;
                ss << "Couldn't open file '"<<lbuf<<"' for reading: " << strerror(errno);
                throw LockFileException( ss.str() );
            }

            // Read the PID of the locker
            int pidOfLocker;
            ret = fscanf( fd, "%d", &pidOfLocker );
            fclose(fd);

            if ( ret != 1 )
            {
                unlink(pbuf);
                throw LockFileException( "Couldn't read PID of locker" );
            }
            if ( pidOfLocker <= 0 )
            {
                unlink(pbuf);
                stringstream ss; ss << "Invalid PID of locker: " << pidOfLocker;
                throw LockFileException( ss.str() );
            }

            if ( pidOfLocker == lpid )
            {
                // I'm the locker ?!?!
                unlink(pbuf);
                stringstream ss; ss << "device " << dev << " is already locked by me! (pid "<<lpid<<")";
                throw LockFileException( ss.str() );
            }

            // Look for the process which owns the lock
            if ( kill(pidOfLocker, 0) == 0 || errno == EPERM)
            {
                unlink(pbuf);
                stringstream ss; ss << "device " << dev << " is already locked by process PID " << pidOfLocker;
                // Throw a special exception which indicates this case
                throw LockedByOtherProcessException( ss.str() );
            }
            else if ( errno == ESRCH )
            {
                // The process which owns the lock no longer exists.  Clean up.
                ret = unlink( lbuf );
                if ( ret != 0 )
                {
                    unlink(pbuf);
                    stringstream ss; ss << "Couldn't unlink " << lbuf << ": " << strerror(errno);
                    throw LockFileException( ss.str() );
                }

                // Now create our own
                ret = link( pbuf, lbuf );
                if ( ret < 0 )
                {
                    unlink(pbuf);
                    stringstream ss; ss << "Couldn't link("<<pbuf<<","<<lbuf<<"): " << strerror(errno);
                    throw LockFileException( ss.str() );
                }
            }
            else
            {
                stringstream ss;
                ss << "lock_dev: Don't expect to see this errno after kill: " << strerror(errno);
                throw LockFileException( ss.str() );
            }
        }

        ret = unlink(pbuf);
        if ( ret < 0 )
        {
            stringstream ss;
            ss << "Couldn't unlink pbuf: " << strerror(errno);
            throw LockFileException( ss.str() );
        }
    }

    void
    unlock_dev(const char *dev, int lpid)
    {
        FILE *fd;
        char  lbuf[260];
        int   pidOfLocker = 0;

        const char *lastSlashPos = strrchr(dev, '/');
        if ( lastSlashPos )
            dev = lastSlashPos + 1;
        sprintf(lbuf, "%s/LCK..%s", LOCK_DIR, dev);

        fd = fopen(lbuf, "r");
        if (fd && fscanf(fd, "%d", &pidOfLocker) == 1 && pidOfLocker == lpid)
        {
            fclose(fd);
            unlink(lbuf);
        }
        else if (fd)
            fclose(fd);
    }
}

//////////////////////////////////////////////////////////////////////

LockFile::LockFile( const std::string &dev,
                    int lockPid )
    : dev_(dev),
      lockPid_(lockPid)
{
    lock_dev( dev_.c_str(), lockPid_ );
}

LockFile::~LockFile()
{
    // Don't allow exceptions from destructor
    try {
        unlock_dev( dev_.c_str(), lockPid_ );
    }
    catch ( ... ) {}
}

}
}

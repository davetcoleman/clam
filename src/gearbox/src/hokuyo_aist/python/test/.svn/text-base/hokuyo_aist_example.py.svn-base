#!/usr/bin/env python

import hokuyo_aist
from optparse import OptionParser

def main():
    parser = OptionParser()
    parser.add_option('-c', '--clustercount', dest='cluster_count',
            type='int', default='1',
            help='Cluster count [default: %default]')
    parser.add_option('-e', '--endangle', dest='end_angle', type='float',
            default='0',
            help='End angle to get ranges to [default: %default]')
    parser.add_option('-f', '--firststep', dest='first_step', type='int',
            default='-1',
            help='First step to get ranges from [default: %default]')
    parser.add_option('-l', '--laststep', dest='last_step', type='int',
            default='-1',
            help='Last step to get ranges to [default: %default]')
    parser.add_option('-n', '--new', dest='get_new', action='store_true',
            default='False', help='Get new ranges instead of latest \
ranges [default: %default]')
    parser.add_option('-o', '--portoptions', dest='port_options', type='string',
            default='type=serial,device=/dev/ttyACM0,timeout=1',
            help='Port options (see flexiport library) [default: %default]')
    parser.add_option('-s', '--startangle', dest='start_angle', type='float',
            default='0',
            help='Start angle to get ranges from [default: %default]')
    parser.add_option('-v', '--verbose', dest='verbose', action='store_true',
            default='False',
            help='Put the hokuyo_aist library into verbose mode \
[Default: %default]')

    # Scan command line arguments
    options, args = parser.parse_args()

    try:
        # Create an instance of a laser scanner object
        laser = hokuyo_aist.Sensor()
        if options.verbose == True:
            # Set verbose mode so we see more information in stderr
            laser.set_verbose(True)

        # Open the laser
        laser.open(options.port_options)
        # Turn the laser on
        laser.set_power(True)

        # Get some laser info
        #print 'Laser sensor information:'
        #info = hokuyo_aist.SensorInfo info()
        #laser.get_sensor_info(info)
        #print info.as_string()

        # Get range data
        data = hokuyo_aist.ScanData()
        if (options.first_step == -1 and options.last_step == -1) and \
                (options.start_angle == 0 and options.end_angle == 0):
            # Get all ranges
            if options.get_new == True:
                laser.get_new_ranges(data, -1, -1, options.cluster_count)
            else:
                laser.get_ranges(data, -1, -1, options.cluster_count)
        elif options.first_step != -1 or options.last_step != -1:
            # Get by step
            if options.get_new == True:
                laser.get_new_ranges(data, options.first_step, options.last_step, options.cluster_count)
            else:
                laser.get_ranges(data, options.first_step, options.last_step, options.cluster_count)
        else:
            # Get by angle
            if options.get_new == True:
                laser.get_new_ranges_by_angle(data, options.start_angle, options.end_angle, options.cluster_count)
            else:
                laser.get_ranges_by_angle(data, options.start_angle, options.end_angle, options.cluster_count)

        print 'Laser range data:'
        print data.as_string()

        # Close the laser
        laser.close()

    except hokuyo_aist.BaseError, e:
        print 'Caught exception: ' + e.what()
        return 1
    return 0


if __name__ == '__main__':
    main()


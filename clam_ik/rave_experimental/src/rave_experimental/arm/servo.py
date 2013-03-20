import serial
import time
import sys, optparse
import math
import servo_constants as const

MAX_PACKET_LENGTH = 32

class servo(object):
    '''class to use a single robotis servo
    '''
	
    def __init__(self, dev_name, servo_id, baudrate=1000000, max_angle=300, angle_steps=1024, speed_factor=1):

        ''' dev_name - name of serial device of the servo controller (e.g. '/dev/robot/servo0')
        servo_id - 1,2,3,4 ... (1 to 254)
        baudrate - for the servo controller.
        '''
        self.__dev_name = dev_name
        self.__servo_dev = None
        self.__baudrate = baudrate
        self.__open_serial()
        self.__servo_id = servo_id
        self.__max_angle = float(max_angle)
        self.__angle_register_max = angle_steps
        self.__speed_factor = speed_factor

        self.__read_funcs = {"model number":self.__read_model_number, \
                           "firmware version":self.__read_version_of_firmware, \
                           "id":self.__read_ID, \
                           "baud":self.__read_baudrate, \
                           "delay":self.__read_delay_time, \
                           "cw limit":self.__read_cw_angle_limit, \
                           "ccw limit":self.__read_ccw_angle_limit, \
                           "temperature limit":self.__read_temperature_limit, \
                           "low voltage limit":self.__read_voltage_low_limit, \
                           "high voltage limit":self.__read_voltage_high_limit, \
                           "EEPROM torque limit":self.__read_torque_limit_EEPROM, \
                           "status return level":self.__read_status_return_level, \
                           "alarm led flags":self.__read_alarm_led, \
                           "alarm shutdown flags":self.__read_alarm_shutdown, \
                           "down calibration":self.__read_down_calibration, \
                           "up calibration":self.__read_up_calibration, \
                           "torque enable":self.__read_torque_enable, \
                           "LED":self.__read_LED, \
                           "cw compliance margin":self.__read_cw_compliance_margin, \
                           "ccw compliance margin":self.__read_ccw_compliance_margin, \
                           "cw compliance slope":self.__read_cw_compliance_slope, \
                           "ccw compliance slope":self.__read_ccw_compliance_slope, \
                           "goal":self.__read_goal_position, \
                           "speed":self.__read_moving_speed, \
                           "RAM torque limit":self.__read_torque_limit_RAM, \
                           "pos":self.__read_present_position, \
                           "present speed":self.__read_present_speed, \
                           "present load":self.__read_present_load, \
                           "present voltage":self.__read_present_voltage, \
                           "present temperature":self.__read_present_temperature, \
                           "registered instruction":self.__read_registered_instruction, \
                           "is moving":self.__read_moving, \
                           "lock":self.__read_lock, \
                           "punch":self.__read_punch}

        self.__write_funcs = {"id":self.__write_ID, \
                           "baud":self.__write_baudrate, \
                           "delay":self.__write_delay_time, \
                           "cw limit":self.__write_cw_angle_limit, \
                           "ccw limit":self.__write_ccw_angle_limit, \
                           "temperature limit":self.__write_temperature_limit, \
                           "low voltage limit":self.__write_voltage_low_limit, \
                           "high voltage limit":self.__write_voltage_high_limit, \
                           "EEPROM torque limit":self.__write_torque_limit_EEPROM, \
                           "status return level":self.__write_status_return_level, \
                           "alarm led flags":self.__write_alarm_led, \
                           "alarm shutdown flags":self.__write_alarm_shutdown, \
                           "torque enable":self.__write_torque_enable, \
                           "LED":self.__write_LED, \
                           "cw compliance margin":self.__write_cw_compliance_margin, \
                           "ccw compliance margin":self.__write_ccw_compliance_margin, \
                           "cw compliance slope":self.__write_cw_compliance_slope, \
                           "ccw compliance slope":self.__write_ccw_compliance_slope, \
                           "goal":self.__write_goal_position, \
                           "speed":self.__write_moving_speed, \
                           "RAM torque limit":self.__write_torque_limit_RAM, \
                           "registered instruction":self.__write_registered_instruction, \
                           "lock":self.__write_lock, \
                           "punch":self.__write_punch}
        
        
        if servo_id != 254:
            self.write("torque enable",True)
            read_id = self.read("id")
                
            if read_id == None:
                print 'warning: invalid servo ID'

            
        
    #Accessor Functions
    def get_dev_name(self):
        return self.__dev_name

    def get_servo_id(self):
        return self.__servo_id

    #Read Functions
    def get_read_commands(self):
        ''' returns list of read commands
        '''
        return self.__read_funcs.keys()

    def read(self, command):
        ''' runs read command
        '''
        if self.__read_funcs.has_key(command):
            return self.__read_funcs[command]()
        else:
            print "ERROR: read_servo failed. Command not recognized"

    def print_command_help(self, command):
        ''' prints __doc__ string for command
        '''
        if self.__read_funcs.has_key(command):
            print self.__read_funcs[command].__doc__
        else:
            print "ERROR: read_servo failed. Command not recognized" 

    def print_read_all(self):
        ''' prints all read function outputs
        '''
        for command in self.__read_funcs.keys():
            print command, ":", self.__read_funcs[command]()

    def print_read_help_all(self):
        ''' prints all read function __doc__
        '''
        for command in self.__read_funcs.keys():
            print command, ":", self.__read_funcs[command].__doc__

    #Write Functions
    def get_write_commands(self):
        ''' returns list of write commands
        '''
        return self.__write_funcs.keys()

    def write(self, command, data):
        ''' runs write command
        '''
        if self.__write_funcs.has_key(command):
            add, data = self.__write_funcs[command](data)
            self.__write_location(add, data)
        else:
            print "ERROR: write_servo failed. Command not recognized"

    def print_write_command_help(self, command):
        ''' prints write command __doc__
        '''
        if self.__write_funcs.has_key(command):
            print self.__write_funcs[command].__doc__
        else:
            print "ERROR: write_servo failed. Command not recognized"

    def print_write_help_all(self):
        ''' prints all read function __doc__
        '''
        for command in self.__write_funcs.keys():
            print command, ":", self.__write_funcs[command].__doc__

    #Register Write Functions
    def get_reg_write_commands(self):
        ''' returns list of write commands
        '''
        return self.__write_funcs.keys()

    def reg_write_command(self, command, data):
        ''' runs write command
        '''
        if self.__write_funcs.has_key(command):
            add, data = self.__write_funcs[command](data)
            self.__write_register(add, data)
        else:
            print "ERROR: write_servo failed. Command not recognized"

    def print_reg_write_command_help(self, command):
        ''' prints write command __doc__
        '''
        if self.__write_funcs.has_key(command):
            print self.__write_funcs[command].__doc__
        else:
            print "ERROR: write_servo failed. Command not recognized"

    def print_reg_write_help_all(self):
        ''' prints all read function __doc__
        '''
        for command in self.__write_funcs.keys():
            print command, ":", self.__write_funcs[command].__doc__



    #Private Read Functions    
    def __read_model_number(self):
        '''if model known, returns string containing model name
        '''
        data,err = self.__read_location([const.MODEL_NUMBER],2)
        code = data[1]*0x100+data[0]
        if code == 0x000C:
            return "AX-12"
        else:
            return "Model not known: "+str(data[0])

    def __read_version_of_firmware(self):
        '''returns version of firmware as int
        '''
        data,err = self.__read_location([const.VERSION_OF_FIRMWARE])
        return data[0]

    def __read_ID(self):
        '''returns ID read from the servo
        '''
        data,err = self.__read_location([const.ID])
        if data[0] == []:
            return None
        return data[0]
    
    def __read_baudrate(self):
        '''returns baudrate read from the servo
        '''
        data,err = self.__read_location([const.BAUD_RATE])
        return 2000000/(data[0]+1)

    def __read_delay_time(self):
        '''returns the return time delay (microseconds)
        '''
        data,err = self.__read_location([const.RETURN_DELAY_TIME])
        ans = data[0]*2
        return ans
    
    def __data_to_angle(self,data):
#        print data[0]+data[1]*256.0
        return (data[0]+data[1]*256.0)/self.__angle_register_max * self.__max_angle
    
    def __read_cw_angle_limit(self):
        '''returns angle limit on servo (degrees)
        '''
        data,err = self.__read_location([const.CW_ANGLE_LIMIT],2)
        
        return self.__data_to_angle(data)

    def __read_ccw_angle_limit(self):
        '''returns angle limit on servo (degrees)
        '''
        data,err = self.__read_location([const.CCW_ANGLE_LIMIT],2)
        
        return self.__data_to_angle(data)
    
    def __read_temperature_limit(self):
        '''returns the temperature limit at which the Overheat Error Bit will be set (C)
        '''
        data,err = self.__read_location([const.HIGHEST_TEMPERATURE])
        return data[0]
    
    def __read_voltage_low_limit(self):
        '''returns the low voltage limit at which the Voltage Range Error Bit will be set (V)
        '''
        data,err = self.__read_location([const.LOWEST_VOLTAGE])
        return data[0]/10

    def __read_voltage_high_limit(self):
        '''returns the high voltage limit at which the Voltage Range Error Bit will be set (V)
        '''
        data,err = self.__read_location([const.HIGHEST_VOLTAGE])
        return data[0]/10

    def __read_torque_limit_EEPROM(self):
        '''EEPROM torque limit value
        '''
        data,err = self.__read_location([const.MAX_TORQUE],2)
        return data[1]*0x100+data[0]

    def __read_status_return_level(self):
        '''Status reaturn level,
            0 - Do not respond to any instructions
            1 - respond only to READ_DATA instructions
            2 - respond to all instructions
        '''
        data,err = self.__read_location([const.STATUS_RETURN_LEVEL])
        return data[0]

    def __read_alarm_led(self):
        '''Returns byte, LED blinks on Error of bit if bit set to 1
            Bit 7 - 0
            Bit 6 - Instruction Error
            Bit 5 - Overload Error
            Bit 4 - Checksum Error
            Bit 3 - Range Error
            Bit 2 - Overheating Error
            bit 1 - Angle Limit Error
            Bit 0 - Voltage Error
        '''
        data,err = self.__read_location([const.ALARM_LED])
        return bin(data[0])
    
    def __read_alarm_shutdown(self):
        '''Returns byte, torque shutdown on Error of bit if bit set to 1
            Bit 7 - 0
            Bit 6 - Instruction Error
            Bit 5 - Overload Error
            Bit 4 - Checksum Error
            Bit 3 - Range Error
            Bit 2 - Overheating Error
            bit 1 - Angle Limit Error
            Bit 0 - Voltage Error
        '''
        data,err = self.__read_location([const.ALARM_SHUTDOWN])
        return bin(data[0])

    def __read_down_calibration(self):
        '''Reutrns factory set down calibration (user can't modify)
        '''
        data,err = self.__read_location([const.DOWN_CALIBRATION],2)
        return data

    def __read_up_calibration(self):
        '''Returns factory set up calibration (user can't modify)
	'''
        data,err = self.__read_location([const.UP_CALIBRATION],2)
        return data

    def __read_torque_enable(self):
        '''Returns true if torque is enabled
	'''
        data,err = self.__read_location([const.TORQUE_ENABLE])
        return data[0] == 1
    
    def __read_LED(self):
        '''Returns true if LED is on
	'''
        data,err = self.__read_location([const.LED])
        return data[0] == 1
    
    def __read_cw_compliance_margin(self):
        data,err = self.__read_location([const.CW_COMPLIANCE_MARGIN])
        return data[0]

    def __read_ccw_compliance_margin(self):
        data,err = self.__read_location([const.CCW_COMPLIANCE_MARGIN])
        return data[0]

    def __read_cw_compliance_slope(self):
        data,err = self.__read_location([const.CW_COMPLIANCE_SLOPE])
        return data[0]

    def __read_ccw_compliance_slope(self):
        data,err = self.__read_location([const.CCW_COMPLIANCE_SLOPE])
        return data[0]

    def __read_goal_position(self):
        '''Returns goal position (degrees)
	'''
        data,err = self.__read_location([const.GOAL_POSITION],2)
        
        return self.__data_to_angle(data)
    
    def __read_moving_speed(self):
        '''Returns moving speed (rpm)
        '''
        data,err = self.__read_location([const.MOVING_SPEED],2)
        speed = data[1]*256+data[0]
        speed = speed*0.111*self.__speed_factor
        return speed

    def __read_torque_limit_RAM(self):
        '''RAM torque limit value
        '''
        data,err = self.__read_location([const.TORQUE_LIMIT],2)
        return data[1]*0x100+data[0]

    def __read_present_position(self):
        '''Returns present position (degrees)
	'''
        data,err = self.__read_location([const.PRESENT_POSITION],2)
#        print const.PRESENT_POSITION, data[0], data[1]
        
        return self.__data_to_angle(data)

    def __read_present_speed(self):
        '''Returns present speed (rpm)
	'''
        data,err = self.__read_location([const.PRESENT_SPEED],2)
        speed = data[1]*256+data[0]
        speed = speed*0.111*self.__speed_factor
        return speed       

    def __read_present_load(self):
        ''' number proportional to the torque applied by the servo.
            sign etc. might vary with how the servo is mounted.
        '''
        data,err = self.__read_location([const.PRESENT_LOAD],2)
        load = data[0]+(data[1]>>6)*256
        if data[1]>>2 & 1 == 0:
            return -load
        else:
            return load      

    def __read_present_voltage(self):
        '''Returns present voltage (V)
        '''
        data,err = self.__read_location([const.PRESENT_VOLTAGE],2)
        return data[0]/10.        

    def __read_present_temperature(self):
        '''Returns present temperature (C)
	'''
        data,err = self.__read_location([const.PRESENT_TEMPERATURE],2)
        return data[0]

    def __read_registered_instruction(self):
        '''Returns true if there is a instruction assigned to REG_WRITE
	'''
        data,err = self.__read_location([const.REGISTERED_INSTRUCTION],2)
        return data[0] == 1
    
    def __read_moving(self):
        '''Returns true if servo is moving by its own power
	'''
        data,err = self.__read_location([const.MOVING])
        return data[0] == 1
    
    def __read_lock(self):
        '''Returns true if servo is locked
        '''
        data,err = self.__read_location([const.LOCK])
        return data[0] == 1

    def __read_punch(self):
        '''Returns the minimum current supplied to motor during operation
	'''
        data,err = self.__read_location([const.PUNCH],2)
        return data[0]


    #Private Write Functions            
    def __write_ID(self, id):
        '''Writes ID to the servo, ID number must be between 0 and 253
        WARNING: function does not check for other servos with the same id on network
        '''
        id = int(id)
        if 0 > id or id > 253:
            print "write id number out of range (0-253). id: ", id
        else:
            return([const.ID], [id])
            self.id = id
    
    def __write_baudrate(self, baudrate):
        '''write baudrate to the servo, input range 2000000 to 7874 and supported by pyserial 
        '''
        if 7874 > baudrate or baudrate > 2000000:
            print "write baudrate number out of servo range (7874-2000000). baudrate: ", baudrate
        elif not baudrate in self.__servo_dev.BAUDRATES:
            print "baudrate not supported by pyserial. baudrate: ", baudrate
        else:        
#            self.__servo_dev.setBaudrate(baudrate)
            data = 2000000/baudrate - 1
            return ([const.BAUD_RATE], [data])

    def __write_delay_time(self, delay_time):
        '''write the return time delay (microseconds), input range 0 to 308 and even
        '''
        if delay_time < 0 or delay_time > 308 or delay_time % 2 == 1:
            print "write delay time fail to meet requirements (0 to 308 and even). delay time: ", delay_time
        else:
            data = delay_rate/2
            return ([const.RETURN_DELAY_RATE], [data])
    
    def __angle_to_data(self,angle):
        bytes = int((angle * self.__angle_register_max / self.__max_angle)+.5)
        if bytes == self.__angle_register_max:
            bytes -= 1
#        print bytes
        data = [0, 0]
        data[1] = bytes / 256
        data[0] = bytes % 256
        return data
     
    def __write_cw_angle_limit(self, angle):
        '''writes angle limit on servo - no offset(0 to self.__max_angle)(degrees)
        '''
        
        data=self.__angle_to_data(angle)
        
        if data [1] > 3 or angle < 0:
            print "write cw angle limit fail to meet requirements (0.0 to "+str(self.__max_angle)+"). angle: ", angle
        else:
            return ([const.CW_ANGLE_LIMIT], data)
       
    def __write_ccw_angle_limit(self, angle):
        '''writes angle limit on servo (0 to 299)(degrees)
        '''
        
        data=self.__angle_to_data(angle)

        if data [1] > 3 or angle < 0:
            print "write ccw angle limit fail to meet requirements (0.0 to "+str(self.__max_angle)+"). angle: ", angle
        else:
            return ([const.CCW_ANGLE_LIMIT], data)
    
    def __write_temperature_limit(self, temperature):
        '''writes the temperature limit at which the Overheat Error Bit will be set (0 to 150)(C)
        '''
        if temperature < 0  or temperature > 150:
            print "write temperature limit fail to meet requirements (0 to 150). temperature: ", temperature
        else:
            return ([const.HIGHEST_TEMPERATURE], [temperature])
    
    def __write_voltage_low_limit(self, voltage):
        '''writes the low voltage limit at which the Voltage Range Error Bit will be set (5.0 to 25.0)(V)
        '''
        if voltage < 5 or voltage > 25:
            print "write voltage limit fail to meet requirements (5.0 to 25.0). voltage: ", voltage
        else:
            voltage = round(voltage, 1)
            return ([const.LOWEST_VOLTAGE], [int(voltage*10)])
        

    def __write_voltage_high_limit(self, voltage):
        '''writes the high voltage limit at which the Voltage Range Error Bit will be set (5.0 to 25.0)(V)
        '''
        if voltage < 5 or voltage > 25:
            print "write voltage limit fail to meet requirements (5.0 to 25.0). voltage: ", voltage
        else:
            voltage = round(voltage, 1)
            return ([const.HIGHEST_VOLTAGE], [int(voltage*10)])

    def __write_torque_limit_EEPROM(self, torque):
        '''EEPROM torque limit value (0 to 1023)
        '''
        if torque < 0  or torque > 1023:
            print "write torque limit fail to meet requirements (0 to 1023). torque: ", torque
        else:
            data = [0, 0]
            data[1] = int(torque / 256)
            data[0] = int(torque % 256)
            
            print data
            return ([const.MAX_TORQUE], data)

    def __write_status_return_level(self, level):
        '''Write status reaturn level,
            0 - Do not respond to any instructions
            1 - respond only to READ_DATA instructions
            2 - respond to all instructions
        '''
        if level < 0 or level > 2 or not (level % 1 == 0):
            print "write status reaturn level fail to meet requirements (1, 2, 3). level: ", level
        else:
            return ([const.STATUS_RETURN_LEVEL], [level])

    def __write_alarm_led(self, byte):
        '''Writes byte, LED blinks on Error of bit if bit set to 1 (give in int form)
            Bit 7 - 0
            Bit 6 - Instruction Error
            Bit 5 - Overload Error
            Bit 4 - Checksum Error
            Bit 3 - Range Error
            Bit 2 - Overheating Error
            bit 1 - Angle Limit Error
            Bit 0 - Voltage Error
        '''
        if byte > 127 or not (byte % 1 == 0):
            print "write alarm led fail input. input:", byte
        else:
            return ([const.ALARM_LED], [byte])
    
    def __write_alarm_shutdown(self, byte):
        '''Returns byte, torque shutdown on Error of bit if bit set to 1
            Bit 7 - 0
            Bit 6 - Instruction Error
            Bit 5 - Overload Error
            Bit 4 - Checksum Error
            Bit 3 - Range Error
            Bit 2 - Overheating Error
            bit 1 - Angle Limit Error
            Bit 0 - Voltage Error
        '''
        if byte > 127 or not (byte % 1 == 0):
            print "write alarm shutdown fail input. input:", byte
        else:
            return ([const.ALARM_SHUTDOWN], [byte])

    def __write_torque_enable(self, enable=True):
        '''enables torque if ture, disables if false
        '''
        return ([const.TORQUE_ENABLE], [int(enable)])
	
    
    def __write_LED(self, enable=True):
        '''if true, LED is on. if false, LED is off
	'''
        return ([const.LED], [int(enable)])
    
    def __write_cw_compliance_margin(self, margin):
        '''range 0  to 254
        '''
        if margin < 0 or margin > 254:
            print "write cw compliance margin input invalid. (0 to 254) input:", margin
        else:
            return ([const.CW_COMPLIANCE_MARGIN], [int(margin)])

    def __write_ccw_compliance_margin(self, margin):
        '''range 0  to 254
        '''
        if margin < 0 or margin > 254:
            print "write ccw compliance margin input invalid. (0 to 254) input:", margin
        else:
            return ([const.CCW_COMPLIANCE_MARGIN], [int(margin)])

    def __write_cw_compliance_slope(self, slope):
        '''range 1  to 254
        '''
        if slope < 1 or slope > 254:
            print "write cw compliance slope input invalid. (1 to 254) input:", slope
        else:
            return ([const.CW_COMPLIANCE_SLOPE], [int(slope)])

    def __write_ccw_compliance_slope(self, slope):
        '''range 1  to 254
        '''
        if slope < 1 or slope > 254:
            print "write ccw compliance slope input invalid. (1 to 254) input:", slope
        else:
            return ([const.CCW_COMPLIANCE_SLOPE], [int(slope)])

    def __write_goal_position(self, angle):
        '''writes goal position to servo (0 to 299)
	'''
        data = self.__angle_to_data(angle)
        if data[1] > self.__angle_register_max/256 or angle < 0:
            raise ValueError("write goal angle limit fail to meet requirements (0.0 to "+str(self.__max_angle)+"). angle: " + str(angle))
        
        return ([const.GOAL_POSITION], data)
    
    def __write_moving_speed(self, rpm):
        '''writes moving speed (rpm) (0 to 113.5)
        if zero, velocity is as high as voltage allows, no voltage control used.
	'''
        rpm *= self.__speed_factor
        if rpm < 0 or rpm > 113.5:
            print "write moving speed fail to meet requirements (0 to 113.5). rpm: ", rpm
        else:
            speed = rpm/0.111
            speed = max(speed,1)
            data = [int(speed%256), int(speed/256)]
            return ([const.MOVING_SPEED],data)


    def __write_torque_limit_RAM(self, torque):
        '''RAM torque limit value (0 to 1023)
        '''
        if torque < 0  or torque > 1023:
            print "write torque limit fail to meet requirements (0 to 1023). torque: ", torque
        else:
            data = [0, 0]
            data[1] = int(torque / 256)
            data[0] = int(torque % 256)
            return ([const.TORQUE_LIMIT], data)

    def __write_registered_instruction(self, enable):
        '''if true, there is a instruction assigned to REG_WRITE. if false, there is not an instruction
	'''
        if not (enable == True or enable == False):
            print "write registered instruction invalid input, not a boolean. input:", enable
        else:
            return ([const.REGISTERED_INSTRUCTION], [int(enable)])
    
    def __write_lock(self, lock=True):
        '''locks servo, only Address 0x18 to 0x23 can be written to and other areas cannot.
        (Torque Enable, LED, compliance, goal position, moving speed)
        Once locked, it can only be unlocked by turning the power off.
        '''
        return ([const.LOCK], [1])

    def __write_punch(self, punch):
        '''writes the minimum current supplied to motor during operation (0 to 1023)
	'''
        if punch < 0  or punch > 1023:
            print "write punch invaild input. (0 to 1023) input:", punch
        else:
            data = [int(punch%256), int(punch / 256)]
            return ([const.PUNCH], data)

    #Basic Instruction Functions
    def __read_location(self, address, nBytes=1):
        ''' reads nBytes from address on the servo.
            returns [n1,n2 ...], error
            list of parameters, error byte.
        '''
        msg = [const.READ_DATA]+address+[nBytes] 
#        self.__servo_dev.flushOutput()       
#        self.__servo_dev.flushInput()
        to=self.__servo_dev.timeout
        self.__servo_dev.timeout=0.001
        s=self.__servo_dev.read(1024)
#        print list(ord(c) for c in  s)
        self.__servo_dev.timeout=to
        self.__send_serial(msg,self.__servo_id)      
        return self.__read_serial(6+nBytes)

    def __write_location(self, address, data, status=False):    
        ''' writes data at the [address].
            data = [n1,n2 ...] list of numbers.
        '''
        msg = [const.WRITE_DATA] + address + data
        self.__send_serial(msg,self.__servo_id) 
        if status:
            if self.__servo_id != 254:
                self.__read_serial(6)
#            else:
#        self.__servo_dev.flushOutput()

    def __write_register(self, address, data):
        ''' writes data to the buffer at address (reg_write)
            data = [n1,n2 ...] list of numbers.
        '''
        msg = [const.REG_WRITE] + address + data
        self.__send_serial(msg,self.__servo_id)

    def action (self):
        ''' excutes instructions in reg_write buffer
        '''
        self.__send_serial([const.ACTION],self.__servo_id)

    def ping (self):
        ''' excutes ping, return status
        '''
        self.__send_serial([const.PING],self.__servo_id)
        return self.__read_serial(6)

    def reset (self):
        ''' Changes the control table values to the Factory Default Value settings
        '''
        self.__send_serial([const.RESET],self.__servo_id)  
    

    #Serial Functions
    def __calc_checksum(self, msg):
        chksum = 0
        for m in msg:
            chksum += m
        chksum = (~chksum)%256
        return chksum
    
    def __send_serial(self, message, id):
        ''' sends the command to the servo
            msg - list
        '''
        msg = [id,len(message)+1]+message
        #instruction includes the command (1 byte + parameters. length = parameters+2)
        chksum = self.__calc_checksum(msg)
        msg = [0xff,0xff]+msg+[chksum]
        
        str = ''
        for m in msg:
            str += chr(m)
            
        self.__servo_dev.flushInput()
        self.__servo_dev.write(str)

    def __read_serial(self, nBytes=0):
        '''reads a number of bytes from the servo
            nBytes - number of bytes to read
        '''
        s = self.__servo_dev.read(1)
        new = s
        while new and new != '\xff':
            new = self.__servo_dev.read(1)
            s += new
        if s == '':
#            print '__read_serial: Could not read from the servo.'
#            print 'Ensure that the 3-way switch on the USB2Dynamixel is at RS485.'
#            print 'Exiting...'
            raise RuntimeError("robotis_servo: Serial port not found!\n")
                
        while s[-1] == '\xff':
            new = self.__servo_dev.read(1)
            s += new # read until we read the ID (first byte after 0xFF)
        new = self.__servo_dev.read(1) # read length
        s += new
        length = ord(s[-1])
        
        if length > MAX_PACKET_LENGTH:
            raise RuntimeError("__read_serial: invalid response packet length\n")
        
        for i in range(length):
#            print list(ord(a) for a in s)
            new = self.__servo_dev.read(1)
            if new == '':
                raise RuntimeError("__read_serial: incomplete response packet: expected "+length + ", got: "+i+"\n")
            s += new # rest of message
        
        l = list(ord(a) for a in s)
#        print l
        start = s.find('\xff')
        if start < 0:
            raise RuntimeError("__read_serial: improperly formatted response packet\n")
        
        s = s[start:].lstrip('\xff')
        err=ord(s[2])
        
        data=list(ord(x) for x in s[3:-1])
        if len(data) < nBytes-6:
            raise RuntimeError("__read_serial: not enough data found in response packet "+str(l)+"\n")
        
        return data,err

    def __open_serial(self):
        '''opens comunication with serial
        '''
        
        try:
            self.__servo_dev = serial.Serial(self.__dev_name, timeout=.01)
            self.__servo_dev.setBaudrate(self.__baudrate)
            self.__servo_dev.setParity('N')
            self.__servo_dev.setStopbits(1)
            self.__servo_dev.open()

            self.__servo_dev.flushOutput()
            self.__servo_dev.flushInput()

        except (serial.serialutil.SerialException), e:
            raise RuntimeError("robotis_servo: Serial port not found!\n")
        if(self.__servo_dev == None):
            raise RuntimeError("robotis_servo: Serial port not found!\n")   

    
        

class servo_ex106(servo):
    def __init__(self, dev_name, servo_id, baudrate=1000000, max_angle=254.2, angle_steps=4096,speed_factor=1.28):
        servo.__init__(self, dev_name=dev_name, servo_id=servo_id, baudrate=baudrate, max_angle=max_angle, angle_steps=angle_steps, speed_factor=speed_factor)

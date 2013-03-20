import servo
import servo_constants as const
import time

softBlocking,hardBlocking,velocity = range(3)

class single_joint:
    '''class to control joint of one servo
    '''
    def __init__(self, dev_name, servo_id, baudrate = 1000000, offset = 0, min_angle = 0, max_angle = 299, max_rpm = 30, flipped=1, type=None):
        ''' dev_name - name of serial device of the servo controller (e.g. '/dev/ttyUSB0')
        servo_id - 1,2,3,4 ... (range: 1 to 253)
        baudrate - for the servo controller. (default: 1000000)
        offset - offset of robotis 0 angle to input 0 angle (default: 0, range: 0-300) 
        min_angle - min valid input angle. (default: 0, range: 0-300)
        max_angle - max valid input angle. (default: 300, range: 0-300)
        max_rpm - max valid rpm input speed (default: 113, range: 0-113)
        flipped - Set -1 if you wish to reverse angle orienation (default: 1)
        '''

        # offset within hardware angle bounds
        # min is less then max
        # min is above or equal the hardware lower bound
        # max is below or equal the hardware upper bound
        # max rpm falls within hardware bounds
        
        if (offset >= 0 and offset<300 and 
            min_angle >= 0 and min_angle<360 and 
            max_angle >= 0 and max_angle<360 and 
            min_angle < max_angle and 
            max_rpm >= 0 and max_rpm <= 113 and 
            (flipped == 1 or flipped == -1)):
            
            limits = [(min_angle-offset)*flipped+offset,(max_angle-offset)*flipped+offset]
            self.offset = offset   
            self.min_angle = min(limits)
            self.max_angle = max(limits)
            self.max_rpm = max_rpm
            self.flipped = flipped

        else:
            print "WARNING: parameters are invalid. Halting operation."
            raise ValueError('invalid servo parameters')
        
        if type == 'EX-106':
            self.servo = servo.servo_ex106(dev_name, servo_id)
        else:
            self.servo = servo.servo(dev_name, servo_id)
        
        self.servo_write("speed", self.max_rpm)
        self.softBlockingTolerance = 3
        self.id=servo_id
    #Accessor Functions
    def get_dev_name(self):
        return self.servo.dev_name

    def get_servo_id(self):
        return self.servo.servo_id

    def get_offset(self):
        return self.offset

    def get_min_angle(self):
        return self.min_angle

    def get_max_angle(self):
        return self.max_angle

    def get_max_rpm(self):
        return self.max_rpm

    def get_flipped(self):
        return self.flipped

    #Modifier Functions
    def set_offset(self, offset):
        ''' set offset. (range: 0-300)
        '''
        if offset >= 0 and offset<300:           
            self.offset = offset
        else:
            print "WARNING: offset invalid. Using previous settings."

    def set_min_angle(self, min_angle):
        ''' set min angle. (range: 0-300)
        '''        
        if min_angle >= 0 and min_angle<300 and \
           min_angle < self.max_angle:
        
            self.min_angle = min_angle
        else:
            print "WARNING: min angle invalid. Using previous settings."

    def set_max_angle(self, max_angle):
        ''' set min angle. (range: 0-300)
        '''        
        if max_angle >= 0 and max_angle<=300 and \
           self.min_angle < max_angle:
               
            self.max_angle = max_angle
        else:
            print "WARNING: max angle invalid. Using previous settings."

    def set_max_rpm(self, rpm):
        '''set max rpm. (range: 0-133)
        '''
        if rpm >= 0 and rpm <= 113:
            self.max_rpm = rpm
        else:
            print "WARNING: rpm invalid. Using previous settings."

    def set_flipped(self, flipped):
        if (flipped == 1 or flipped == -1):
            self.flipped = flipped
        else:
            print "WARNING: flipped invalid. Using previous settings."

    def set_parameters (self, offset=None, min_angle=None, max_angle=None, max_rpm=None, flipped=None):
        if not (offset == None):
            self.set_offset(offset)
        if not (min_angle == None):
            self.set_min_angle(min_angle)
        if not (max_angle == None):
            self.set_max_angle(max_angle)
        if not (max_rpm == None):
            self.set_max_rpm(max_rpm)
        if not (flipped == None):
            self.set_flipped(flipped)
            
    #SERVO commands
    def servo_read_commands(self):
        return self.servo.get_read_commands()

    def servo_read(self, command):
        return self.servo.read(command)

    def servo_print_read_all(self):
        self.servo.print_read_all()

    def servo_write_commands(self):
        return self.servo.get_write_commands()

    def servo_write(self, command, data):
        self.servo.write(command, data)

    def servo_reg_write_commands(self):
        return self.servo.get_reg_write_commands()

    def servo_reg_write(self, command, data):
        self.servo.reg_write_command(command, data)

    def servo_action(self):
        self.servo.action()
        
        time.sleep(0.05)
        #while(self.servo_read("is moving")):
        #    continue
        
    def servo_reset(self):
        self.servo.reset()

    #JOINT INSTRUCTIONS
    def goal_angle(self, angle, rpm=None, blocking=None):
        ''' move to angle (degrees)
            if rpm left at None, rpm left at last value
        ''' 
        deg = angle*self.flipped +self.offset
        
        
        
        if deg>self.max_angle or deg<self.min_angle:
            print 'goal_angle: angle out of range- ', angle
            return

        if not rpm == None:
            if not self.speed(rpm):
                return

#        print self.id, angle,deg
        
        self.servo_write("goal", deg)

        #IMPROVE --------------------------------------------------------
#        time.sleep(0.05)
#        #while(self.servo_read("is moving")):
#        #    continue
        
        if blocking == hardBlocking:
            while self.is_moving():
                time.sleep(.01)
        elif blocking == softBlocking:
            curr = self.servo_read("pos")
            err = deg - curr      
            print self.id, deg, curr, err,
            while abs(err) > self.softBlockingTolerance:
                time.sleep(.01)
                curr = self.servo_read("pos")
                err = deg - curr
                print '\r'+str(self.id),deg, curr, err,
            print
                
        
    def store_goal_angle(self, angle):
        ''' move instruction stored in register (degrees)
        '''
        deg = angle*self.flipped +self.offset
        
        while deg > 360:
            deg = deg - 360
        while deg < 0:
            deg = deg + 360
        
        if deg>self.max_angle or deg<self.min_angle:
            print 'store_goal_angle: angle out of range- ', angle
            return
        
        self.servo_reg_write("goal position", deg)

    def speed(self, rpm):
        ''' sets the moving speed of the servo (rpm)
        '''

        if rpm>self.max_rpm:
            print 'move_angle: rpm too high -', rpm
            print 'ignoring command.'
            return False

        self.servo_write("speed", rpm)
        
        return True
    
    def torque(self,enable):
        ''' enabe/disable torque on the servo
        '''
        self.servo_write("torque enable", enable)
        
    def read_position(self):
        ''' returns current position with offset taken into account
        '''
#        print 'reading pos'
        deg = self.servo_read("pos")
#        print 'read pos', deg
        return (deg - self.offset)*self.flipped
    
    def is_moving(self):
        ''' returns whether or not the servo is moving
        '''

        moving = self.servo_read("is moving")
        return moving

class double_joint:
    '''class to control joint of double servo
    '''
    def __init__(self, dev_name, servo_id1, servo_id2, \
                 baudrate = 1000000, offset1 = 0, min_angle = 0, \
                 max_angle = 299, max_rpm = 30, flipped1=1, \
                 offset2 = 0, flipped2=1):
        ''' dev_name - name of serial device of the servo controller (e.g. '/dev/ttyUSB0')
        servo_id - 1,2,3,4 ... (range: 1 to 253)
        baudrate - for the servo controller. (default: 1000000)
        offset - offset of robotis 0 angle to input 0 angle (default: 0, range: 0-300) 
        min_angle - min valid input angle. (default: 0, range: 0-300)
        max_angle - max valid input angle. (default: 300, range: 0-300)
        max_rpm - max valid rpm input speed (default: 113, range: 0-113)
        flipped - Set -1 if you wish to reverse angle orienation (default: 1)
        '''

        servo_0 = single_joint(dev_name, servo_id1, baudrate, offset1, min_angle, max_angle, max_rpm, flipped1)
        servo_1 = single_joint(dev_name, servo_id2, baudrate, offset2, min_angle, max_angle, max_rpm, flipped2)

        self.servo = [servo_0, servo_1]
        self.softBlockingTolerance = 3

        
    #Accessor Functions
    def get_dev_name(self):
        return self.servo[0].get_dev_name()

    def get_servo_id(self, servo):
        ''' servo = 0 or 1
        '''
        if servo == 0 or servo == 1:
            return self.servo[servo].get_dev_name()
        else:
            print "ERROR: invalid servo number. ignoring get_servo_id request"

    def get_offset(self, servo):
        ''' servo = 0 or 1
        '''
        if servo == 0 or servo == 1:
            return self.servo[servo].get_offset()
        else:
            print "ERROR: invalid servo number. ignoring get_offset request"
            
    def get_min_angle(self, servo):
        ''' servo = 0 or 1
        '''
        if servo == 0 or servo == 1:
            return self.servo[servo].get_min_angle()
        else:
            print "ERROR: invalid servo number. ignoring get_min_angle request"

    def get_max_angle(self, servo):
        ''' servo = 0 or 1
        '''
        if servo == 0 or servo == 1:
            return self.servo[servo].get_max_angle()
        else:
            print "ERROR: invalid servo number. ignoring get_max_angle request"

    def get_max_rpm(self, servo):
        ''' servo = 0 or 1
        '''
        if servo == 0 or servo == 1:
            return self.servo[servo].get_max_rpm()
        else:
            print "ERROR: invalid servo number. ignoring get_max_rpm request"

    def get_flipped(self, servo):
        ''' servo = 0 or 1
        '''
        if servo == 0 or servo == 1:
            return self.servo[servo].get_flipped()
        else:
            print "ERROR: invalid servo number. ignoring get_flipped request"

    #Modifier Functions
    def set_offset(self, offset, servo):
        ''' set offset. (range: 0-300)
            servo = 0 or 1
        '''
        if servo == 0 or servo == 1:
            return self.servo[servo].set_offset(offset)
        else:
            print "ERROR: invalid servo number. ignoring set_offset request"

    def set_min_angle(self, min_angle, servo):
        ''' set min angle. (range: 0-300)
            servo = 0 or 1
        '''
        if servo == 0 or servo == 1:
            return self.servo[servo].set_min_angle(min_angle)
        else:
            print "ERROR: invalid servo number. ignoring set_min_angle request"

    def set_max_angle(self, max_angle, servo):
        ''' set max angle. (range: 0-300)
            servo = 0 or 1
        '''
        if servo == 0 or servo == 1:
            return self.servo[servo].set_max_angle(max_angle)
        else:
            print "ERROR: invalid servo number. ignoring set_max_angle request"
            

    def set_max_rpm(self, rpm, servo):
        '''set max rpm. (range: 0-133)
            servo = 0 or 1
        '''
        if servo == 0 or servo == 1:
            return self.servo[servo].set_max_rpm(rpm)
        else:
            print "ERROR: invalid servo number. ignoring set_max_rpm request"

    def set_flipped(self, flipped, servo):
        ''' servo = 0 or 1
        '''
        if servo == 0 or servo ==1:
            return self.servo[servo].set_flipped(flipped)
        else:
            print "ERROR: invalid servo number. ignoring set_flipped request"

    def set_parameters (self, servo, offset=None, min_angle=None, max_angle=None, max_rpm=None, flipped=None):
        if servo == 0 or servo == 1:
            self.servo[servo].set_parameters(offset, min_angle, max_angle, max_rpm, flipped)
        else:
            print "ERROR: invalid servo number. ignoring set_parameters request"
            
    #SERVO commands
    def servo_read_commands(self):
        return self.servo[0].servo_read_commands()

    def servo_read(self, command, servo=None):
        '''reads command to servo, both if set to None
        '''
        if servo == 0 or servo == 1:
            return self.servo[servo].servo_read(command)
        elif servo == None:
            return (self.servo[0].servo_read(command), self.servo[1].servo_read(command))
        else:
            print "ERROR: invalid servo number. ignoring servo_read request"

    def servo_print_read_all(self):
        print "servo 0:"
        self.servo[0].servo_print_read_all()
        print "\nservo 1:"
        self.servo[1].servo_print_read_all()

    def servo_write_commands(self):
        return self.servo[0].servo_write_commands()

    def servo_write(self, command, data, servo=None):
        '''writes commmand to servo
        '''
        if servo == 0 or servo == 1:
            self.servo[servo].servo_write(command, data)
        else:
            print "ERROR: invalid servo number. ignoring servo_write request"

    def servo_reg_write_commands(self):
        return self.servo[0].servo_reg_write_commands()

    def servo_reg_write(self, command, data, servo):
        if servo == 0 or servo ==1:
            self.servo[servo].servo_reg_write(command, data)
        else:
            print "ERROR: invalid servo number. ignoring servo_write request"

    def servo_action(self):
        self.servo[0].servo_action()
        self.servo[1].servo_action()
        
    def servo_reset(self):
        self.servo[0].servo_reset()
        self.servo[1].servo_reset()

    #JOINT INSTRUCTIONS
    def goal_angle(self, angle, rpm=None, blocking=None):
        ''' move to angle (degrees)
            if rpm left at None, rpm left at last value
        ''' 
        
        if not rpm == None:
            if not self.speed(rpm):
                return

        self.servo[0].store_goal_angle(angle)
        self.servo[1].store_goal_angle(angle)

        self.servo_action()
        
        if blocking == hardBlocking:
            while self.is_moving():
                time.sleep(.01)
        elif blocking == softBlocking:
            while abs(angle-self.read_position()) > self.softBlockingTolerance:
                time.sleep(.01)
        

    def store_goal_angle(self, angle):
        ''' move instruction stored in register (degrees)
        '''
        self.servo[0].store_goal_angle(angle)
        self.servo[1].store_goal_angle(angle)

    def speed(self, rpm):
        ''' sets the moving speed of the servo (rpm)
        '''
        return self.servo[0].speed(rpm) and self.servo[1].speed(rpm)
    
    def torque(self,enable):
        ''' enabe/disable torque on the servo
        '''
        self.servo[0].torque(enable)
        self.servo[1].torque(enable)
        
    def read_position(self):
        ''' returns current position with offset taken into account
        '''
        return self.servo[0].read_position()
        
    def is_moving(self):
        ''' returns whether or not the servo is moving
        '''
        return self.servo[0].is_moving()
        



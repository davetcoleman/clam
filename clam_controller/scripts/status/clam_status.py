__version__ = "0.1"
__author__ = "Dave Coleman"

import roslib
roslib.load_manifest('clam_controller')

import rospy
import locale
locale.setlocale(locale.LC_NUMERIC, "")
from std_msgs.msg import String
from dynamixel_msgs.msg import JointState
import padnums as pp
import sys
import time

joint_names = ['shoulder_pan',
               'shoulder_pitch',
               'elbow_roll',
               'elbow_pitch',
               'wrist_roll',
               'wrist_pitch',
               'gripper_roll',
               'l_gripper_aft']
joint_model = ['AX-12',
               'EX-106',
               'RX-64',
               'RX-64',
               'RX-28',
               'RX-28',
               'AX-12+',
               'AX-12+']

table = [] # where all the data is stored to be output

# Indexes of data within table
table_ids = {"speed": 0, "motor init": 0, "motor max": 0, "motor min": 0, 
             "motor temp": 0, "goal pos":0, "current pos": 0, 
             "error pos": 0, "velocity": 0, "load": 0, "is moving": 0}

display_counter = 0 # counter for when to rediplay the table

out = sys.stdout # used for printing to console


def joint_callback(data):
    global table_ids, table, display_counter

    col_id = data.motor_ids[0] + 1

    table[  table_ids["motor temp"]  ][ col_id ] = data.motor_temps[0]
    table[  table_ids["goal pos"]  ][ col_id ] = data.goal_pos
    table[  table_ids["current pos"]  ][ col_id ] = data.current_pos
    table[  table_ids["error pos"]  ][ col_id ] = data.error
    table[  table_ids["velocity"]  ][ col_id ] = data.velocity
    table[  table_ids["load"]  ][ col_id ] = data.load
    table[  table_ids["is moving"]  ][ col_id ] = data.is_moving
    
    display_counter += 1

    if (display_counter % 10 == 0):
        print_table()

def print_table():

    # Update the displayed speed from parameter server
    table[table_ids["speed"]] = getParam('joint_speed','speed')
    
    print "\x1B[2J" 

    print display_counter
    pp.pprint_table(out, table)

    
def main():
    global table_ids, table

    print '\n\n'

    # Column Header
    first_column = joint_names[:]
    first_column.insert(0, '')
    table = [first_column]

    # Joint Motor ID
    table.append(getParam('motor/id','id'))

    # Servo Model
    joint_model.insert(0, 'model')
    table.append(joint_model);

    # Joint Speed
    table_ids["speed"] = len(table) # remember which row id belongs to this property
    table.append(getParam('joint_speed','speed'))

    # Joint Motor Init
    table_ids["motor init"] = len(table) # remember which row id belongs to this property
    table.append(getParam('motor/init','motor init'))

    # Joint Motor Max
    table_ids["motor max"] = len(table) # remember which row id belongs to this property
    table.append(getParam('motor/max','motor max'))

    # Joint Motor Min
    table_ids["motor min"] = len(table) # remember which row id belongs to this property
    table.append(getParam('motor/min','motor min'))    

    # Motor Temps
    table_ids["motor temp"] = len(table) # remember which row id belongs to this property
    table.append(makeBlank('motor temp'))

    # Goal Position
    table_ids["goal pos"] = len(table) # remember which row id belongs to this property
    table.append(makeBlank('goal pos'))

    # Current Pos
    table_ids["current pos"] = len(table) # remember which row id belongs to this property
    table.append(makeBlank('current pos'))

    # Error Pos
    table_ids["error pos"] = len(table) # remember which row id belongs to this property
    table.append(makeBlank('error pos'))

    # Velocity
    table_ids["velocity"] = len(table) # remember which row id belongs to this property
    table.append(makeBlank('velocity'))

    # Load
    table_ids["load"] = len(table) # remember which row id belongs to this property
    table.append(makeBlank('load'))

    # Is Moving
    table_ids["is moving"] = len(table) # remember which row id belongs to this property
    table.append(makeBlank('is moving'))



    # ------------------------------------------------------

    # Setup listener for every joint node
    rospy.init_node('clam_status', anonymous=True)
    for joint_name in joint_names:
        rospy.Subscriber("/"+joint_name+"_controller/state", JointState, joint_callback)

    print "Waiting for response from joint nodes..."

    rospy.spin()

    # ----------------------------------------------------

def makeBlank(row_name):
    # First column:
    row = [row_name]

    for joint_name in joint_names:
        row.append( 0 )

    return row    

def getParam(param_name, row_name):
    # First column:
    row = [row_name]

    for joint_name in joint_names:
        row.append( rospy.get_param('/'+joint_name+'_controller/'+param_name) )

    return row

# Where program starts

if __name__ == "__main__":
    main()

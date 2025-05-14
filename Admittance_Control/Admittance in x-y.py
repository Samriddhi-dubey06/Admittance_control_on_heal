#!/usr/bin/env python3

import rospy
import numpy as np
import serial
import re
import PyKDL as kdl
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState  
from kdl_parser_py.urdf import treeFromParam
from urdf_parser_py.urdf import URDF

# Initialize ROS Node
rospy.init_node("force_to_velocity_xy")

# Serial Communication Setup
SERIAL_PORT_X = "/dev/ttyUSB1"
BAUD_RATE_X = 115200
SERIAL_PORT_Y = "/dev/ttyUSB0"
BAUD_RATE_Y = 9600

y_sensor = serial.Serial(SERIAL_PORT_Y, baudrate=BAUD_RATE_Y, timeout=1)

# Publisher for Joint Velocities
joint_velocity_pub = rospy.Publisher("/velocity_controller/command", Float64MultiArray, queue_size=10)

# Conversion factors , zero offsets and scaling factors
LBS_TO_N = 4.44822
ZERO_OFFSET_FX = -0.18416
ZERO_OFFSET_FY = -0.69850
force_to_velocity_scale_x = 0.00545
force_to_velocity_scale_y = 0.007

# Moving average filter buffer for X-axis force
force_buffer_x = []

def get_robot_kdl_chain():
    if not rospy.has_param("/robot_description"):
        rospy.logerr("URDF not found on parameter server!")
        return None, None

    robot = URDF.from_parameter_server() # Load the URDF Model
    success, tree = treeFromParam("/robot_description") #Convert the URDF to a KDL Tree
    
    if not success:
        rospy.logerr("Failed to parse URDF into KDL tree!")
        return None, None

    base_link = robot.get_root() #root of the URDF tree
    end_effector = "tool"  
    
    if end_effector not in [l.name for l in robot.links]:
        rospy.logerr("End-effector link not found in URDF!")
        return None, None
    
    chain = tree.getChain(base_link, end_effector) #extracts the KDL chain between the base link and the end-effector
    return chain, chain.getNrOfJoints()

# Compute Jacobian Matrix
def compute_jacobian(joint_angles):
    chain, num_joints = get_robot_kdl_chain()
    if chain is None:
        return None

    jac_solver = kdl.ChainJntToJacSolver(chain)
    jnt_array = kdl.JntArray(num_joints) #creates a KDL array to hold the joint angles
    for i in range(num_joints):
        jnt_array[i] = joint_angles[i] #The loop assigns each joint angle from the joint_angles list to the corresponding index in jnt_array

    jacobian = kdl.Jacobian(num_joints)
    jac_solver.JntToJac(jnt_array, jacobian)
    return np.array([[jacobian[i, j] for j in range(num_joints)] for i in range(6)])

# Read force from X-axis sensor
def read_ft_sensor_x():
    global force_buffer_x # stores the last few force readings to smooth out noise
    try:
        with serial.Serial(SERIAL_PORT_X, BAUD_RATE_X, timeout=1) as ser:
            ser.flush() #Clears any remaining unread data from the serial buffer to ensure fresh data is read
            
            #Read Incoming Data Line by Line
            while True:
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    match = re.search(r"[-+]?\d*\.\d+|\d+", line) #Uses regular expressions (regex) to extract force values from the string
                    if match:
                        x_force = float(match.group(0)) * LBS_TO_N
                        #Apply Moving Average Filter
                        force_buffer_x.append(x_force) #Appends the new force reading to force_buffer_x
                        if len(force_buffer_x) > 5:  #ensures that only the last 5 readings are stored (smoothing the data)
                            force_buffer_x.pop(0)
                        return np.mean(force_buffer_x) #Computes the average of the last 5 force readings
                    return 0.0
    except serial.SerialException as e:
        rospy.logerr("Serial error (X-axis): %s", e)
        return 0.0

# Read force from Y-axis sensor
def read_force_y():
    try:
        line = y_sensor.readline().decode('utf-8').strip()
        match = re.search(r'[-+]?\d*\.?\d+([eE][-+]?\d+)?', line)
        return float(match.group()) if match else 0.0
    except Exception as e:
        rospy.logwarn("Error reading sensor (Y-axis): %s", e)
        return 0.0

def compute_and_publish():
    rospy.loginfo("Force to Cartesian Velocity Mapping Started!")  #prints a ROS log message at the INFO level. This helps track when the function starts running
    rate = rospy.Rate(100)
    velocity_msg = Float64MultiArray()
    
    while not rospy.is_shutdown():
        fx_raw = read_ft_sensor_x()
        fy_raw = read_force_y() * LBS_TO_N
        
        if abs(fx_raw) > 10 or abs(fy_raw) > 60:
            rospy.logwarn("Force value too high, ignoring! Fx = %.2f N, Fy = %.2f N", fx_raw, fy_raw)
            continue

        fx_corrected = fx_raw - ZERO_OFFSET_FX
        fy_corrected = fy_raw - ZERO_OFFSET_FY
        #fx_corrected and fy_corrected are displayed with 5 decimal places (.5f)
        rospy.loginfo(f"Corrected Forces: Fx = {fx_corrected:.5f} N, Fy = {fy_corrected:.5f} N")
        
        # Admittamnce control law
        K_d = np.diag([1e2, 1e2, 1e2, 1e2, 1e2, 1e2])  # Virtual damping matrix
        K_d_inv = np.linalg.inv(K_d)
        h_e = np.array([fx_corrected , fy_corrected , 0, 0, 0, 0]) 

        # cartesian_velocity = np.array([fx_corrected * force_to_velocity_scale_x , -fy_corrected* force_to_velocity_scale_y , 0, 0, 0, 0]) # For cartesian force to velocity mapping
        try:
            joint_state_msg = rospy.wait_for_message("/joint_states", JointState, timeout=2.0)
            joint_angles = np.array(joint_state_msg.position)
        except rospy.ROSException:
            rospy.logerr("Failed to get joint states!")
            continue

        J = compute_jacobian(joint_angles)
        if J is None:
            continue

        try:
            q_dot = np.dot(np.linalg.pinv(J, rcond=1e-4), np.dot(K_d_inv, h_e)) # Admittance Control
            # q_dot = np.dot(np.linalg.pinv(J, rcond=1e-4), np.dot(cartesian_velocity)) # F to cart_vel scaling
        except np.linalg.LinAlgError:
            rospy.logerr("Singular Jacobian!")
            continue
        
        q_dot[np.abs(q_dot) < 1e-3] = 0
        q_dot[-1] = 0  # Ensure last joint has zero velocity
        rospy.loginfo(f"Joint Velocities: {q_dot}")

        velocity_msg.data = q_dot.tolist() #.tolist() converts the NumPy array into a standard Python list
        joint_velocity_pub.publish(velocity_msg)
        rate.sleep()

if __name__ == "__main__":
    try:
        compute_and_publish()
    except rospy.ROSInterruptException:
        pass

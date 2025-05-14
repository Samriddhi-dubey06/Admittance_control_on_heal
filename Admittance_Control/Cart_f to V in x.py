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
rospy.init_node("force_to_velocity_x")

# Serial Communication Setup
SERIAL_PORT = "/dev/ttyUSB1"
BAUD_RATE = 115200

# Publisher for Joint Velocities
joint_velocity_pub = rospy.Publisher("/velocity_controller/command", Float64MultiArray, queue_size=10)

# Conversion factor and zero offsets
LBS_TO_N = 4.44822
ZERO_OFFSET_FX = -0.18416
force_to_velocity_scale_x = 0.00545

# Moving average filter buffer
force_buffer = []

# Load URDF and Get KDL Chain
def get_robot_kdl_chain():
    if not rospy.has_param("/robot_description"):
        rospy.logerr("URDF not found on parameter server!")
        return None, None

    robot = URDF.from_parameter_server()
    success, tree = treeFromParam("/robot_description")
    
    if not success:
        rospy.logerr("Failed to parse URDF into KDL tree!")
        return None, None

    base_link = robot.get_root()
    end_effector = "tool"  
    
    if end_effector not in [l.name for l in robot.links]:
        rospy.logerr("End-effector link not found in URDF!")
        return None, None
    
    chain = tree.getChain(base_link, end_effector)
    return chain, chain.getNrOfJoints()

# Compute Jacobian Matrix
def compute_jacobian(joint_angles):
    chain, num_joints = get_robot_kdl_chain()
    if chain is None:
        return None

    jac_solver = kdl.ChainJntToJacSolver(chain)
    jnt_array = kdl.JntArray(num_joints)
    for i in range(num_joints):
        jnt_array[i] = joint_angles[i]

    jacobian = kdl.Jacobian(num_joints)
    jac_solver.JntToJac(jnt_array, jacobian)
    return np.array([[jacobian[i, j] for j in range(num_joints)] for i in range(6)])

# Read force from X-axis sensor
def read_ft_sensor_x():
    global force_buffer

    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            ser.flush()  
            while True:
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    
                    # Extract numerical values using regex
                    match = re.search(r"[-+]?\d*\.\d+|\d+", line)
                    if match:
                        x_force = float(match.group(0)) * LBS_TO_N
                        
                        # Store latest 5 values for moving average filter
                        force_buffer.append(x_force)
                        if len(force_buffer) > 5:
                            force_buffer.pop(0)

                        # Return filtered force
                        return np.mean(force_buffer)  
                    
                    rospy.logwarn("Invalid data format received: %s", line)
                    return 0.0
    except serial.SerialException as e:
        rospy.logerr("Serial error: %s", e)
        return 0.0

def compute_and_publish():
    rospy.loginfo("Force to Cartesian Velocity Mapping Started!")
    rate = rospy.Rate(100)
    velocity_msg = Float64MultiArray()
    
    while not rospy.is_shutdown():
        fx_raw = read_ft_sensor_x()
        
        # Ignore unrealistic force values
        if abs(fx_raw) > 60:
            rospy.logwarn("Force value too high, ignoring! Fx = %.2f N", fx_raw)
            continue

        fx_corrected = fx_raw - ZERO_OFFSET_FX
        rospy.loginfo(f"Corrected Force: Fx = {fx_corrected:.5f} N")

        cartesian_vel = np.array([fx_corrected * force_to_velocity_scale_x, 0, 0, 0, 0, 0]) 
        
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
            q_dot = np.dot(np.linalg.pinv(J, rcond=1e-4), cartesian_vel)
        except np.linalg.LinAlgError:
            rospy.logerr("Singular Jacobian!")
            continue
        
        q_dot[np.abs(q_dot) < 1e-3] = 0
        q_dot[-1] = 0  # Ensure last joint has zero velocity
        rospy.loginfo(f"Joint Velocities: {q_dot}")

        velocity_msg.data = q_dot.tolist()
        joint_velocity_pub.publish(velocity_msg)
        rate.sleep()

if __name__ == "__main__":
    try:
        compute_and_publish()
    except rospy.ROSInterruptException:
        pass

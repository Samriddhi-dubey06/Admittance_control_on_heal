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
rospy.init_node("force_to_velocity")

# Serial Communication Setup
y_sensor = serial.Serial('/dev/ttyUSB0', baudrate=9600, timeout=1)
x_sensor = serial.Serial('/dev/ttyUSB1', baudrate=115200)

# Publisher for Joint Velocities
joint_velocity_pub = rospy.Publisher("/velocity_controller/command", Float64MultiArray, queue_size=10)

# Conversion factor and zero offsets
LBS_TO_N = 4.44822
ZERO_OFFSET_FX = 0.02609   # N
ZERO_OFFSET_FY = -0.69850 # N  
force_to_velocity_scale_1 = 0.007
force_to_velocity_scale_2 = 0.00345

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
    end_effector = "tool"  # Adjust according to your URDF
    
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
    # print("Jacobian:", jacobian)
    return np.array([[jacobian[i, j] for j in range(num_joints)] for i in range(6)])
    

# Read force from serial sensor
def read_force(sensor):
    try:
        line = sensor.readline().decode('utf-8').strip()
        match = re.search(r'[-+]?\d*\.?\d+([eE][-+]?\d+)?', line)
        return float(match.group()) if match else 0.0
    except Exception as e:
        rospy.logwarn("Error reading sensor: %s", e)
        return 0.0

def compute_and_publish():
    rospy.loginfo("Force to Cartesian Velocity Mapping Started!")
    rate = rospy.Rate(100)
    velocity_msg = Float64MultiArray()
    
    while not rospy.is_shutdown():
        fx_raw = read_force(x_sensor) * LBS_TO_N
        fy_raw = read_force(y_sensor) * LBS_TO_N

        fx_corrected = fx_raw - ZERO_OFFSET_FX
        fy_corrected = fy_raw - ZERO_OFFSET_FY

        # Print raw and corrected forces
        # rospy.loginfo(f"Raw Forces: Fx = {fx_raw:.5f} N, Fy = {fy_raw:.5f} N")
        rospy.loginfo(f"Corrected Forces: Fx = {fx_corrected:.5f} N, Fy = {fy_corrected:.5f} N")

        cartesian_vel = np.array([fx_corrected* force_to_velocity_scale_2, -fy_corrected * force_to_velocity_scale_1  , 0, 0, 0, 0]) 
        # rospy.loginfo(f"Cartesian Velocities: {cartesian_vel}")

        try:
            joint_state_msg = rospy.wait_for_message("/joint_states", JointState, timeout=2.0)
            joint_angles = np.array(joint_state_msg.position)
        except rospy.ROSException:
            rospy.logerr("Failed to get joint states!")
            continue

        J = compute_jacobian(joint_angles)
        if J is None:
            continue

        # Extract first row (X-direction influence)
        # rospy.loginfo(f"Jacobian First Row (X direction): {J[0, :]}")  # Debugging step

        try:
            q_dot = np.dot(np.linalg.pinv(J), cartesian_vel)
        except np.linalg.LinAlgError:
            rospy.logerr("Singular Jacobian!")
            continue
        
        q_dot[np.abs(q_dot) < 1e-3] = 0
        rospy.loginfo(f"Joint Velocities: {q_dot}")

        velocity_msg.data = q_dot.tolist()
        joint_velocity_pub.publish(velocity_msg)
        rate.sleep()

if __name__ == "__main__":
    try:
        compute_and_publish()
    except rospy.ROSInterruptException:
        pass

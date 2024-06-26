import os
import pybullet as p
import pybullet_data
import time
from approxeng.input.selectbinder import ControllerResource

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

urdf_file = "BiStable.urdf"

# Check if the URDF file exists before trying to load it
if os.path.isfile(urdf_file):
   robot_id = p.loadURDF(urdf_file)
else:
   print(f"URDF file '{urdf_file}' not found")

# Load a plane
plane_id = p.loadURDF("plane.urdf")


# Set gravity
p.setGravity(0, 0, -9.81)

# Find the index of the joints
num_joints = p.getNumJoints(robot_id)
left_wheel_joint_index = -1
right_wheel_joint_index = -1

# Set friction for the wheels
# for i in range(num_joints):
#     p.changeDynamics(robot_id, i, lateralFriction=0.5)

for i in range(num_joints):
   joint_info = p.getJointInfo(robot_id, i)
   if joint_info[1].decode() == "Revolute 19":
       left_wheel_joint_index = i
   elif joint_info[1].decode() == "Revolute 20":
       right_wheel_joint_index = i

# Function to control the velocity of the left wheel
def set_left_wheel_velocity(velocity):
   if left_wheel_joint_index != -1:
       p.setJointMotorControl2(robot_id, left_wheel_joint_index, p.VELOCITY_CONTROL, targetVelocity=-velocity)

# Function to control the velocity of the right wheel
def set_right_wheel_velocity(velocity):
   if right_wheel_joint_index != -1:
       p.setJointMotorControl2(robot_id, right_wheel_joint_index, p.VELOCITY_CONTROL, targetVelocity=velocity)
   else:
       print("Right wheel joint not found")

# PID controller function
def pid_controller(target_angle, current_angle, kp, ki, kd, prev_error, integral):
   error = target_angle - current_angle
   integral += error
   derivative = error - prev_error
   control_signal = kp * error + ki * integral + kd * derivative
   prev_error = error
   return control_signal, prev_error, integral

def map(fromLB, fromUB, toLB, toUB):
   return (toUB-fromUB)/(toLB - fromLB)

def velocity_map(y, max_inclination):
   return y*max_inclination

def steer_map(x, max_steer):
   return x*max_steer

# Set the target angle and the gains
init_angle = 0.295 # 0.29 is the angle to make the robot stay at rest
#steer = 2
kp = 5
ki = 0.2
kd = 0.2

# Initialize variables for PID control
prev_error = 0
integral = 0

p.resetDebugVisualizerCamera(1.0, 30, -40, [0.0, -0.0, -0.0])

while True:

   try:
       with ControllerResource() as joystick:
           print('Found a joystick and connected')
           while joystick.connected:
               left_x = joystick['lx']
               left_y = joystick['ly']
               robot_position, orientation_quat = p.getBasePositionAndOrientation(robot_id)

               # Convert the quaternion to Euler angles
               roll, pitch, yaw = p.getEulerFromQuaternion(orientation_quat)
               roll = roll * 180 / 3.14159265359
               pitch = pitch * 180 / 3.14159265359
               yaw = yaw * 180 / 3.14159265359

               # print(f"Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")
               target_angle = velocity_map(-left_y, 8) - init_angle
               print(f"error: {target_angle - roll}")

               # Control the wheel velocities using PID control
               control_signal, prev_error, integral = pid_controller(target_angle, roll, kp, ki, kd, prev_error, integral)
               steer = steer_map(left_x, 3)
               set_left_wheel_velocity(control_signal-steer)
               set_right_wheel_velocity(control_signal+steer)

               # Set the camera to follow the robot
               p.resetDebugVisualizerCamera(0.75, 30, -40, robot_position)

               p.stepSimulation()
               time.sleep(0.005)

       print('Connection to joystick lost')
   except IOError:
       # No joystick found, wait for a bit before trying again
       print('Unable to find any joysticks')
       time.sleep(1.0)



p.disconnect()
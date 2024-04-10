import urx
import time
import numpy as np
from pyfirmata import Arduino

def normalize(v):
    """Normalize a vector."""
    norm = np.linalg.norm(v) #length of vector
    if norm == 0:
        return v
    return v / norm

def home(robot):
    robot.movej((-1.57, -1.57, -1.57, -1.57, 1.57, 0), acc = 0.7, vel = 0.7)

def getMarker(robot, tool_changer, unlock, lock):
    home(robot)
    robot.set_tcp((0,0,0,0,0,0))
    tool_changer.write(unlock)
    # coordinates for tool change 
    tool_changer.write(lock)
    # coordinates for tool change 
    home(robot)
    
def returnMarker(robot, tool_changer, unlock):
    home(robot)
    # coordinates for tool change 
    tool_changer.write(unlock)
    # coordinates for tool change 
    robot.set_tcp((0,0,0,0,0,0))
    home(robot)

def vector_to_euler_angles(target_normal):
    """Convert a target normal vector into Euler angles (roll, pitch, yaw) for a UR16e robot arm."""
    # Initial direction vector (end-effector pointing up along the z-axis)
    initial_vector = np.array([0, 0, 1])
    target_normal = normalize(target_normal)
    
    # Rotation axis (cross product of initial and target vectors)
    rotation_axis = np.cross(initial_vector, target_normal)
    rotation_axis_normalized = normalize(rotation_axis)
    
    # Angle of rotation (using the dot product and arccosine)
    cos_angle = np.dot(initial_vector, target_normal)
    angle = np.arccos(cos_angle)
    
    # Convert axis-angle to quaternion
    qx = rotation_axis_normalized[0] * np.sin(angle / 2)
    qy = rotation_axis_normalized[1] * np.sin(angle / 2)
    qz = rotation_axis_normalized[2] * np.sin(angle / 2)
    qw = np.cos(angle / 2)
    
    # Convert quaternion to Euler angles (roll, pitch, yaw)
    # Assuming a 'zyx' rotation order
    roll = np.arctan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx**2 + qy**2))
    pitch = np.arcsin(2 * (qw * qy - qz * qx))
    yaw = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))
    
    return roll, pitch, yaw

def checkSurface(robot, tool_changer, tool, waypoints, normal_vector):
    robot.set_tcp((0,0,0,0,0,0))
    home(robot)
    lock = 0
    unlock = 1
    tool_off = 0 
    tool_on = 1
    counter = 0
    tool.write(tool_off)
    euler_angles = (vector_to_euler_angles(normal_vector))
    eax = euler_angles[0]
    eay = euler_angles[1]
    eaz = euler_angles[2]
    o = robot.get_orientation()
    o.rotate_xb(eax)
    o.rotate_yb(eay)
    o.rotate_zb(eaz)
    robot.set_orientation(o)
    orientation = robot.getl()
    rx = orientation[3]
    ry = orientation[4]
    rz = orientation[5]
    getMarker(robot, tool_changer, unlock, lock)
    for x in waypoints:
        robot.movel(x, acc = 0.3, vel = 0.3)
        time.sleep(0.2)
        counter = counter + 1
        print("Moving to point", counter)
    returnMarker(robot, tool_changer, unlock)
    
def main():
    robot = urx.Robot("172.16.3.114")
    board = Arduino('/dev/ttyACM0')
    waypoints = [()]      #insert published points here santi
    normal_vector = []    #insert normal here santi
    tool_relay_pin_number = 7
    tool = board.get_pin(f'd:{tool_relay_pin_number}:o')
    tool_changer_relay_pin_number = 8
    tool_changer = board.get_pin(f'd:{tool_changer_relay_pin_number}:o')
    checkSurface(robot, tool_changer, tool, waypoints, normal_vector)
    home(robot)
    robot.close()

if __name__ == "__main__":
    main()
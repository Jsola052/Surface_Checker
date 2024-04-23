import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from pyfirmata import Arduino
import urx
import numpy as np
import time

# Convert degrees to radians
def deg2rad(deg):
    return deg * np.pi / 180

# Normalize a vector
def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0: 
        return v
    return v / norm

# Move the robot to its home position
def home(robot, acc, vel):
    home_position = (deg2rad(-90), deg2rad(-90), deg2rad(-90), deg2rad(-90), deg2rad(90), deg2rad(0))
    robot.movej(home_position, acc, vel)

# Calculate the orientation for surface check based on the points
def vector_to_euler_angles(target_normal):
   initial_vector = np.array([0, 0, 1])
   target_normal = normalize(target_normal)
   rotation_axis = np.cross(initial_vector, target_normal)
   rotation_axis_normalized = normalize(rotation_axis)
   cos_angle = np.dot(initial_vector, target_normal)
   angle = np.arccos(cos_angle)
   qx = rotation_axis_normalized[0] * np.sin(angle / 2)
   qy = rotation_axis_normalized[1] * np.sin(angle / 2)
   qz = rotation_axis_normalized[2] * np.sin(angle / 2)
   qw = np.cos(angle / 2)
   # Using a 'zyx' rotation order
   roll = np.arctan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx**2 + qy**2))
   pitch = np.arcsin(2 * (qw * qy - qz * qx))
   yaw = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))
   return roll, pitch, yaw

def getMarker(robot, tool_changer, unlock, lock, marker_payload, marker_tcp):
    home(robot, 0.5, 0.5)
    robot.set_tcp((0,0,0,0,0,0))
    tool_changer.write(unlock)
    robot.movel((0.26055, -0.03720, 0.48702, 2.204, 2.247, -0.067), 0.3, 0.3)
    robot.movel((0.26055, -0.03720, 0.28893, 2.204, 2.247, -0.067), 0.3, 0.3)
    robot.movel((0.26055, -0.03720, 0.23795, 2.204, 2.247, -0.067), 0.05, 0.05)
    time.sleep(0.2)  
    tool_changer.write(lock)
    time.sleep(0.2)
    robot.set_payload(marker_payload)
    time.sleep(0.2)
    robot.movel((0.26055, -0.03720, 0.28893, 2.204, 2.247, -0.067), 0.1, 0.1)
    robot.movel((0.26055, -0.03720, 0.48702, 2.204, 2.247, -0.067), 0.3, 0.3)
    home(robot, 0.5, 0.5)
    robot.set_tcp(marker_tcp)
    time.sleep(0.2)
    
def returnMarker(robot, tool_changer, unlock, normal_payload, normal_tcp):
    home(robot, 0.5, 0.5)
    robot.set_tcp(normal_tcp)
    robot.movel((0.26055, -0.03720, 0.48702, 2.204, 2.247, -0.067), 0.3, 0.3)
    robot.movel((0.26055, -0.03720, 0.28893, 2.204, 2.247, -0.067), 0.3, 0.3)
    robot.movel((0.26055, -0.03720, 0.23795, 2.204, 2.247, -0.067), 0.05, 0.05) 
    time.sleep(0.2)
    tool_changer.write(unlock)
    time.sleep(0.2)
    robot.set_payload(normal_payload)
    time.sleep(0.2)
    robot.movel((0.26055, -0.03720, 0.28893, 2.204, 2.247, -0.067), 0.1, 0.1)
    robot.movel((0.26055, -0.03720, 0.48702, 2.204, 2.247, -0.067), 0.3, 0.3)
    home(robot, 0.5, 0.5)

def offset(corner, offset, normal):
   corner_new = corner - offset*normal
   return corner_new

def generatePath(points, normal, rx, ry, rz):
    path = []
    last_point = offset(points[0], 0.01, normal)
    for x in points:
        path.append((x[0], x[1], x[2], rx, ry, rz))
    path.append(points[0], rx, ry, rz)
    path.append((last_point, rx, ry, rz))
    return path

# Perform the surface check task
def checkSurface(ur_control, acc, vel, normal_vector, points, tool_changer):
    home(ur_control.robot, 0.5, 0.5)
    lock = 0
    unlock = 1
    marker_payload = 1.200
    normal_payload = 1.100
    normal_tcp = (0, 0, 0, 0, 0, 0)
    marker_tcp = (0, 0, 0.22563, 0.0017, 3.1368, -0.0013)
    getMarker(ur_control.robot, tool_changer, unlock, lock, marker_payload, marker_tcp)
    ur_control.robot.set_payload(marker_payload)
    ur_control.robot.set_tcp(marker_tcp)
    orientation = vector_to_euler_angles(normal_vector)
    eax = orientation[0]
    eay = orientation[1]
    eaz = orientation[2]
    o = ur_control.robot.get_orientation()
    o.rotate_xb(eax)
    o.rotate_yb(eay)
    o.rotate_zb(eaz)
    ur_control.robot.set_orientation(o)
    linearPosition = ur_control.robot.getl() 
    rx = linearPosition[3]
    ry = linearPosition[4]
    rz = linearPosition[5]
    path = generatePath(points, normal_vector, rx, ry, rz)
    for x in path:
        ur_control.robot.movel(x, acc, vel)
    home(ur_control.robot, 0.5, 0.5)
    returnMarker(ur_control.robot, tool_changer, unlock, normal_payload, normal_tcp)
    ur_control.robot.set_payload(normal_payload)
    # Clean
    ur_control.clear_path()

# ROS2 Node for controlling the UR robot
class URControlNode(Node):
    def __init__(self):
        super().__init__('ur_control_node')
        self.subscription = self.create_subscription(PoseArray, 'repair_area', self.pose_array_callback, 10)
        self.marker_publisher = self.create_publisher(Marker,'repair_path', 10)
        self.points_list = []
        self.robot_ip = "172.16.3.114"  # Replace with your robot's IP address
        self.robot = None

    def pose_array_callback(self, msg):
        # Saving corners
        self.clear_path()
        self.points_list.clear()
        for pose in msg.poses:
            processed_point = np.array([-round(pose.position.x , 2 ), 
                                        -round(pose.position.y , 2 ), 
                                         round(pose.position.z , 2 )])
            self.points_list.append(processed_point)
        # Make sure there are four corners
        if len(self.points_list) >= 4:
            self.points_list = self.points_list[-4:]
            self.control_ur_robot()

    def control_ur_robot(self):
        connected = False
        tries = 0
        maxTries= 5
        while not connected and tries < maxTries:
            try:
                time.sleep(0.3)
                robot = urx.Robot("172.16.3.114")
                time.sleep(0.3)
                connected = True
            except:
                tries += 1
                print(f"Connection attempt {tries} failed.")
                time.sleep(1)  # Wait for a second before next attempt
        if connected:
            points = np.array(self.points_list)
            self.robot = urx.Robot(self.robot_ip)
            board = Arduino('/dev/ttyACM0')
            tool_changer_relay_pin_number = 8
            tool_changer = board.get_pin(f'd:{tool_changer_relay_pin_number}:o')
            normal_vector = []
            home(self.robot, 0.8, 0.8)
            checkSurface(self, 0.3, 0.3, normal_vector, points, tool_changer)
            home(self.robot,0.8,0.8)
            self.robot.close()
            self.robot = None
        else:
            print("mani has the solution up his ass")

    def show_path(self, waypoints):
        marker = Marker() 
        marker.id = 1 
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "base_link"
        marker.type = Marker.LINE_STRIP
        marker.scale.x = 0.001
        # Color    
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.5
        # Geometry
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        # Waypoints
        marker.points = []
        for point in waypoints:
            p = Point() 
            p.x = -point[0]
            p.y = -point[1]
            p.z = point[2]
            marker.points.append(p)
        # Publish
        self.marker_publisher.publish(marker)

    def clear_path(self):
        marker = Marker() 
        marker.id = 1 
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "base_link"
        marker.action = Marker.DELETE
        self.marker_publisher.publish(marker)

def main():
    rclpy.init()
    ur_control_node = URControlNode()
    rclpy.spin(ur_control_node)
    ur_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

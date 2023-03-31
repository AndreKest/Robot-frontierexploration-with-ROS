#!/usr/bin/env python3
# Calculate the map
# With the help of the Bresenham-Algorithm the cell states of the grid map are calculated
# and publshed to a topic with the current robot position
#	map:		/grid_map
#	current pose:	/pose_robot
# FREE (0), UNKNOWN (-1), OCCUPIED (100)
import numpy as np
import cv2
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

import message_filters

import rclpy
from rclpy.node import Node

# ------------------------------------------------------------------------------------------------------------
# Global Constants
TRESHOLD_P_FREE = 0.3
TRESHOLD_P_OCC = 0.6

P_PIOR = -1 	# Prior occupancy probability
P_OCC = 100     # Probability that cell is occupied with total confidence
P_FREE = 0	    # Probability that cell is free with total confidence 

RESOLUTION = 0.05 # Grid resolution in [m]

MAP_NAME  = 'house' # map name without extension
if MAP_NAME == 'world':
    X_LIM = [-4, 4]
    Y_LIM = [-4, 4]
elif MAP_NAME == 'house':
    X_LIM = [-10, 10]
    Y_LIM = [-10, 10]
else:
    X_LIM = [-15, 15]
    Y_LIM = [-15, 15]  

class GridMap:
	"""
	Grid map
	# -1:	UNKNOWN
	# 0:	FREE
	# 100: 	OCCUPIED

	"""
	def __init__(self, X_lim, Y_lim, resolution, p):
		self.X_lim = X_lim
		self.Y_lim = Y_lim
		self.resolution = resolution

		x = np.arange(start = X_lim[0], stop = X_lim[1] + resolution, step = resolution)
		y = np.arange(start = Y_lim[0], stop = Y_lim[1] + resolution, step = resolution)
		
		# probability matrix 
		self.data = np.full(shape = (len(x), len(y)), fill_value = p, dtype=np.int8)


	def get_shape(self):
		"""
		Get dimensions
		"""
		return np.shape(self.data)

	def to_numpy_uint8(self):
		"""
		Convert the map to a numpy array of dtype=uint8
		-1 	-> 100: 	Unknown
		0: 	-> 0: 		Free
		100 -> 255		Occupied
		"""
		np_map_uint8 = np.zeros(shape=self.data.shape, dtype=np.uint8)
		np_map_uint8[self.data == 0] = 255
		np_map_uint8[self.data == 100] = 0
		np_map_uint8[self.data == -1] = 100
		return np_map_uint8
				

	def discretize(self, x_cont, y_cont):
		"""
		Discretize continious x and y 
		"""
		x = int((x_cont - self.X_lim[0]) / self.resolution)
		y = int((y_cont - self.Y_lim[0]) / self.resolution)
		return (x,y)

	
	def update(self, x, y, p):
		"""
		Update x and y coordinates in discretized grid map
		"""
		# if it's already marked as occupied then never mark it as free
		# otheriwse I had problems with mapping during driving
		if self.data[x,y] != 100:
			self.data[x,y] = p

	def getCost(self, mx, my):
		return self.data[mx, my]
	
	def getSize(self):
		""" Return the shape of the map """
		return self.data.shape

	def getSizeX(self):
		""" Return shape of the map in X direction  """
		return self.data.shape[0]

	def getSizeY(self):
		""" Return shape of the map in Y direction """
		return self.data.shape[1]

	def mapToWorld(self, mx, my):
		""" Transform map coordinates to world coordinates """
		wx = mx * self.resolution + self.X_lim[0]
		wy = my * self.resolution + self.Y_lim[0]
		return (wx,wy)

	def worldToMap(self, wx, wy):
		""" Transform world coordinates to map coordinates """
		mx = int((wx - self.X_lim[0]) / self.resolution)
		my = int((wy - self.Y_lim[0]) / self.resolution)
		return (mx,my)

def lidar_scan(msgScan):
    """ 
    Convert msgScan to Arrays 
    distances: distance of the ith lidar ray 
    angles: angle of the ith lidar ray
    information: confident of the ith measurement 

    return: distances [m]
            angles [rad]
            information [0-1]
    """
    ranges = msgScan.ranges
    distances = np.array([])
    angles = np.array([])
    information = np.array([])

    for i in range(len(ranges)):
        
        if ranges[i] != float('inf'):
            # Calculate angles
            ang = i * msgScan.angle_increment

            # Calculate distances
            if(ranges[i] > msgScan.range_max):
                dist = msgScan.range_max
            elif(ranges[i] < msgScan.range_min):
                dist = msgScan.range_min
            else:
                dist = ranges[i]

            # Calculate information gain
            # Smaller distance, bigger the information (measurement is more confident)
            inf = ((msgScan.range_max - dist) / msgScan.range_max) **2

            distances = np.append(distances, dist)
            angles = np.append(angles, ang)
            information = np.append(information, inf) 

    return distances, angles, information
        

def convert_lidar_to_xy(distances, angles, x_odom, y_odom, theta_odom):
    """ 
    Convert the lidar measurements to xy-coodinates in the XY-plane

    return  distances_x: x-coordiantes of the distances
            distances_y_ y-coordinates of the distances
    """    
    distances_x = np.array([])
    distances_y = np.array([])

    for dist, ang in zip(distances, angles):
        dist_x = x_odom + dist * np.cos(ang + theta_odom)
        dist_y = y_odom + dist * np.sin(ang + theta_odom)
        distances_x = np.append(distances_x, dist_x) 
        distances_y = np.append(distances_y, dist_y) 

    return distances_x, distances_y


def euler_from_quaternion(quaternion):
    """
    Converts quaternion to euler - roll, pitch, yaw

    Source: https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = np.arctan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = np.arcsin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = np.arctan2(t3, t4)

    return roll_x, pitch_y, yaw_z


def get_odom_data(msg_odom):
    """ Get the x, y coodinates and angle theta of the current robot_position """
    x = msg_odom.pose.pose.position.x
    y = msg_odom.pose.pose.position.y
    _, _, theta = euler_from_quaternion(msg_odom.pose.pose.orientation)

    return x, y, theta


def bresenham(map, x1, y1, x2, y2):
    """
    Bresenham's line drawing algorithm - working for all 4 quadrants 
    https://github.com/lukovicaleksa/grid-mapping-in-ROS/edit/main/scripts/bresenham.py
    """
	# Output pixels
    X_bres , Y_bres = [], []
    x, y = x1, y1
	
    delta_x, delta_y = np.abs(x2 - x1),  np.abs(y2 - y1)
	
    s_x, s_y = np.sign(x2 - x1), np.sign(y2 - y1)

    if delta_y > delta_x:
        delta_x, delta_y = delta_y, delta_x
        interchange = True
    else:
        interchange = False
    
    A = 2 * delta_y
    B = 2 * (delta_y - delta_x)
    E = 2 * delta_y - delta_x

    # mark output pixels
    X_bres.append(x)
    Y_bres.append(y)

    # point (x2,y2) must not be included
    for i in range(1, delta_x):
        if E < 0:
            if interchange:
                y += s_y
            else:
                x += s_x
            E = E + A
        else:
            y += s_y
            x += s_x
            E = E + B
        
        # mark output pixels
        X_bres.append(x)
        Y_bres.append(y)

    return zip(X_bres, Y_bres)


class Minimal_PubSub(Node):
    def __init__(self):
        super().__init__('minimal_pubsub')
        # Publisher
        qos_policy_pub = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.RELIABLE, history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)
        qos_policy_pub_map = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.RELIABLE, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL, history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=5)
        self.Publisher_map = self.create_publisher(msg_type=OccupancyGrid, topic='/grid_map', qos_profile=qos_policy_pub_map)
        self.Publisher_pose = self.create_publisher(msg_type=PoseStamped, topic='/pose_robot', qos_profile=qos_policy_pub)

        # Subscribe to messages with message_filters due to synchronizatio of the topics.
        qos_policy_sub = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)
        Subscription_odom = message_filters.Subscriber(self, Odometry, '/odom', qos_profile=qos_policy_sub)
        Subscription_scan = message_filters.Subscriber(self, LaserScan, '/scan', qos_profile=qos_policy_sub)
        ts = message_filters.ApproximateTimeSynchronizer([Subscription_odom, Subscription_scan], 10, 0.01)
        ts.registerCallback(self.callback)

        self.map = GridMap(X_lim=X_LIM, Y_lim=Y_LIM, resolution=RESOLUTION, p=P_PIOR)
        self.msg_odom = None
        self.msg_scan = None


    def callback(self, msg_odom, msg_scan):
        """ Callback function for Subscriber """
        try:
            # Read sensor values
            self.read_odom(msg_odom)
            self.read_scan(msg_scan)

            # Mapping
            self.fct_mapping()

            # Write data (map, current_pose)
            self.fct_write_data()
        except:
            print("Error - Waiting that everything is started..")

    def read_scan(self, msg_scan):
        """ Read scan messages from /scan topic """
        self.msg_scan = msg_scan

    def read_odom(self, msg_odom):
        """ Read sensor message from /odom topic """
        self.msg_odom = msg_odom


    def fct_mapping(self):
        # Current robot position x, y, theta
        x_odom, y_odom, theta_odom = get_odom_data(self.msg_odom)

        # Lidar data
        distances, angles, _ = lidar_scan(self.msg_scan)

        # Lidar measurement in XY plane
        distances_x, distances_y = convert_lidar_to_xy(distances, angles, x_odom, y_odom, theta_odom)
        
        # Convert robot pose to map coordinates for bresenham
        x1, y1 = self.map.worldToMap(x_odom, y_odom)

        # Update map with lidar measurements
        for dist_x, dist_y, dist in zip(distances_x, distances_y, distances):
            x2, y2 = self.map.worldToMap(dist_x, dist_y)

            # Draw discrete line of free pixels with bresenham
            for x_bres, y_bres in bresenham(self.map, x1, y1, x2, y2):
                self.map.update(x=x_bres, y=y_bres, p=0)
            
            # Mark laser hit spot as occupied (if exists)
            if dist < self.msg_scan.range_max:
                self.map.update(x=x2, y=y2, p=100)


    def fct_write_data(self):
        """
        Publish sensor data 
        Write:  - map: current occupancy grid map   topic: /grid_map
                - pose: current pose of the robot   topic: /pose_robot
        """
        map_x = ((self.map.get_shape()[0]/2) * RESOLUTION)
        map_y = ((self.map.get_shape()[1]/2) * RESOLUTION)

        # Build OccupancyGrid message
        pose = Pose(position=Point(x=-map_x, y=-map_y, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
        header = Header(stamp=Node.get_clock(self).now().to_msg(), frame_id='map')
        info = MapMetaData(width=self.map.get_shape()[0], height=self.map.get_shape()[1], resolution=RESOLUTION, map_load_time=Node.get_clock(self).now().to_msg(), origin=pose)
        msg_gridmap = OccupancyGrid(header=header, info=info)
        msg_gridmap.data = cv2.rotate(cv2.flip(self.map.data, 1), cv2.ROTATE_90_COUNTERCLOCKWISE).flatten().tolist()

        msg_pose = PoseStamped(header=header, pose=self.msg_odom.pose.pose)
        
        # Publish data
        self.Publisher_map.publish(msg_gridmap)
        self.Publisher_pose.publish(msg_pose) 

        # Visualization with cv2 for debugging
        # np_map = self.map.to_numpy_uint8()
        # cv2.imshow('Gridmap', cv2.rotate(np_map, cv2.ROTATE_180))
        # cv2.waitKey(1)


def main(args=None):
    print("Start Node mapping..")
    rclpy.init(args=args)
    minimal_PubSub = Minimal_PubSub()

    rclpy.spin(minimal_PubSub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_PubSub.destroy_node()
    rclpy.shutdown()
    print("Stop Node exploring..")

if __name__ == '__main__':
    main()

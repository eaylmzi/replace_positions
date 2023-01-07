#! /usr/bin/env python

#Make a python node executable
#chmod u+x ~/catkin_ws/src/beginner_tutorials/src/movetogoal2.py

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import sys
from math import pow, atan2, sqrt
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time
from std_srvs.srv import Empty


#nodeid=str(sys.argv[1])
#nodename="turtle"+nodeid
#nodename="tb3_"+nodeid

goalx=-2 #float(sys.argv[2])
goaly=2 #float(sys.argv[3])
direction = 1
class Robot:
	
	def __init__(self,nodeid):
		rospy.init_node('turtletogoal', anonymous=True)
		self.nodename = 'robot_' + nodeid
		#For ROS Stage robot_0 is added before the topic names, remove it for ros Gazebo
		self.vel_publisher = rospy.Publisher(self.nodename + '/cmd_vel', Twist, queue_size=10) 
		self.pose_subscriber = rospy.Subscriber(self.nodename +'/odom', Odometry, self.update_pose)
		#self.pose = Pose()
		self.odom = Odometry()
		self.pose = self.odom.pose.pose
		self.rate = rospy.Rate(10)
		self.roll = self.pitch = self.yaw = 0.0

	def update_pose(self, data):
		self.pose = data.pose.pose
	        orientation_q = self.pose.orientation
	        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	        (self.roll, self.pitch, self.yaw) = euler_from_quaternion (orientation_list)
		#print self.yaw
		#rospy.loginfo(str(self.pose.position.x), str(self.pose.position.y))
	def get_position_x(self):
		return self.pose.position.x
	def get_position_y(self):
		return self.pose.position.y

	def euclidean_distance(self, goal_pose):
		return sqrt(pow((goal_pose.position.x-self.pose.position.x),2)+pow((goal_pose.position.y-self.pose.position.y),2))

	def linear_vel(self, goal_pose, constant=0.1):
		return constant * self.euclidean_distance(goal_pose)

	def steering_angle(self, goal_pose):
		return atan2(goal_pose.position.y-self.pose.position.y, goal_pose.position.x-self.pose.position.x)

	def angular_vel(self, goal_pose, constant=0.5):
		return constant * (self.steering_angle(goal_pose)-self.yaw)

	def movetask(self):
					
		vel_msg =Twist()
		speed=0.05
		isforward=1
		distance=0.0000000001
		if isforward == 1:
			vel_msg.linear.x=abs(speed)
		else:
			vel_msg.linear.x=-abs(speed)

	
		vel_msg.angular.z=0.0
	
		rate= rospy.Rate(10)
		t0=rospy.Time.now().to_sec()
		current_dist=0
		while current_dist < distance:
			self.vel_publisher.publish(vel_msg)
			t1=rospy.Time.now().to_sec()
			current_dist=speed*(t1-t0)
			rate.sleep()	
	def rotatetask(self,angle, clockwise, lspeed=0.0):
		
		if(clockwise == -1):			
			print("The robot turns clockwise with %s degree" % (angle))
		else:
			print("The robot turns counter clockwise with %s degree" % (angle))
			
		
		speed = 7			
		vel_msg =Twist()

		vel_msg.linear.x=0
		vel_msg.angular.z=0

		angularspeed = speed * (math.pi)/180
		vel_msg.angular.z=clockwise*abs(angularspeed)
		rate= rospy.Rate(10)
		t0=rospy.Time.now().to_sec()
		current_angle=0
		relativeangle = angle *(math.pi)/180
		while current_angle < relativeangle:
			self.vel_publisher.publish(vel_msg)
			t1=rospy.Time.now().to_sec()
			current_angle=angularspeed*(t1-t0)
			rate.sleep()
		vel_msg.linear.x=0
		vel_msg.angular.z=0
		self.vel_publisher.publish(vel_msg)	

	def move2goal(self,x,y,tolerance):
		newodom = Odometry()
		goal_pose = newodom.pose.pose
		goal_pose.position.x = x
		goal_pose.position.y = y
		dist_tolerance = tolerance #input("tolerance: ")
		
		print("The robot moves x : %s y : %s with tolerance : %s" % (x,y,tolerance))
		vel_msg = Twist()
		
		while self.euclidean_distance(goal_pose) >= dist_tolerance :
						
			vel_msg.linear.x = self.linear_vel(goal_pose)
			vel_msg.linear.y = 0
			vel_msg.linear.z = 0
			vel_msg.angular.x = 0
			vel_msg.angular.y = 0
			vel_msg.angular.z = self.angular_vel(goal_pose)

			#print(str(self.pose.position.x), str(self.pose.position.y))
			self.vel_publisher.publish(vel_msg)			
			self.rate.sleep()		
		print("The robot stops %s meters away from the final point"% (self.euclidean_distance(goal_pose)))
		vel_msg.linear.x = 0
		vel_msg.angular.z = 0

	def get_angle(self,x,y):
		y1 = (y - self.pose.position.y)
		x1 = (x - self.pose.position.x)
		arct = atan2(y1,x1)
		degree = (arct * 180) / math.pi
		return degree
	def find_way(self,x,y):
		global direction
		y1 = (y - self.pose.position.y)
		x1 = (x - self.pose.position.x)
		if(direction == 1):
			if(x1 < 0 and y1 < 0):
				direction = 3
				return -1
			elif(x1 > 0 and y1 < 0):
				direction = 4
				return -1
			elif(x1 > 0 and y1 > 0):
				direction = 1
				return 1
			elif(x1 < 0 and y1 > 0):
				direction = 2
				return 1				
		elif(direction == 2):
			if(x1 < 0 and y1 < 0):
				direction = 3
				return 1
			elif(x1 > 0 and y1 < 0):
				direction = 4
				return -1
			elif(x1 > 0 and y1 > 0):
				direction = 1
				return -1
			elif(x1 < 0 and y1 > 0):
				direction = 2
				return 1
		elif(direction ==3):
			if(x1 < 0 and y1 < 0):
				direction = 3
				return 1
			elif(x1 > 0 and y1 < 0):
				direction = 4
				return 1
			elif(x1 > 0 and y1 > 0):
				direction = 1
				return -1
			elif(x1 < 0 and y1 > 0):
				direction = 2
				return -1
		elif(direction == 4):
			if(x1 < 0 and y1 < 0):
				direction = 3
				return -1
			elif(x1 > 0 and y1 < 0):
				direction = 4
				return 1
			elif(x1 > 0 and y1 > 0):
				direction = 1
				return 1
			elif(x1 < 0 and y1 > 0):
				direction = 2
				return -1

	def find_angle(self,angle,x1,y1):
		if(direction == 1):
			if(x1 < 0 and y1 < 0):
				return angle + 90 
			elif(x1 > 0 and y1 < 0):
				return angle
			elif(x1 > 0 and y1 > 0):
				return angle 
			elif(x1 < 0 and y1 > 0):
				return angle + 90				
		elif(direction == 2):
			if(x1 < 0 and y1 < 0):
				return angle +90
			elif(x1 > 0 and y1 < 0):
				return angle +90
			elif(x1 > 0 and y1 > 0):
				return angle  
			elif(x1 < 0 and y1 > 0):
				return angle 
		elif(direction ==3):
			if(x1 < 0 and y1 < 0):
				return angle
			elif(x1 > 0 and y1 < 0):
				return angle + 90
			elif(x1 > 0 and y1 > 0):
				return angle + 90
			elif(x1 < 0 and y1 > 0):
				return angle 
		elif(direction == 4):
			if(x1 < 0 and y1 < 0):
				return angle 
			elif(x1 > 0 and y1 < 0):
				return angle  
			elif(x1 > 0 and y1 > 0):
				return angle + 90
			elif(x1 < 0 and y1 > 0):
				return angle + 90



	

if __name__ == "__main__":
	try:
		global direction
		robot_0 = Robot("0")
		robot_1 = Robot("1")
		robot_2 = Robot("2")
		
		

		robot_0.movetask()
		robot_1.movetask()
		robot_2.movetask()		
		time.sleep(1)
		
		first_robot_coordinate_x = robot_0.get_position_x()
		first_robot_coordinate_y = robot_0.get_position_y()
		
		second_robot_coordinate_x = robot_1.get_position_x()
		second_robot_coordinate_y = robot_1.get_position_y()

		third_robot_coordinate_x = robot_2.get_position_x()
		third_robot_coordinate_y = robot_2.get_position_y()

		print("First robot x-axis : %s" % (first_robot_coordinate_x))
		print("First robot y-axis : %s" % (first_robot_coordinate_y))

		angle = robot_0.get_angle(robot_1.get_position_x(),robot_1.get_position_y()) 
		direction_robot = robot_0.find_way(robot_1.get_position_x(),robot_1.get_position_y())
		if(angle < 0):
			angle =  angle * -1
		angle_robot = robot_0.find_angle(angle,robot_1.get_position_x(),robot_1.get_position_y())
		

		robot_0.rotatetask(angle_robot,direction_robot,0)
		robot_0.move2goal(robot_1.get_position_x(),robot_1.get_position_y(),1.5)
		direction = 1
		time.sleep(1)

		print("Second robot x-axis : %s" % (second_robot_coordinate_x))
		print("Second robot y-axis : %s" % (second_robot_coordinate_y))

		angle = robot_0.get_angle(robot_2.get_position_x(),robot_2.get_position_y()) 
		direction_robot = robot_0.find_way(robot_2.get_position_x(),robot_2.get_position_y())
		if(angle < 0):
			angle =  angle * -1
		angle_robot = robot_1.find_angle(angle,robot_2.get_position_x(),robot_2.get_position_y())
		

		robot_1.rotatetask(angle_robot,direction_robot,0)
		robot_1.move2goal(robot_2.get_position_x(),robot_2.get_position_y(),1.5)
		direction = 1
		time.sleep(1)

		print("Third robot x-axis : %s" % (third_robot_coordinate_x))
		print("Third robot y-axis : %s" % (third_robot_coordinate_y))
		angle = robot_2.get_angle(first_robot_coordinate_x,first_robot_coordinate_y) 
		direction_robot = robot_2.find_way(first_robot_coordinate_x,first_robot_coordinate_y)

		if(angle < 0):
			angle =  angle * -1
		angle_robot = robot_2.find_angle(angle,first_robot_coordinate_x,first_robot_coordinate_y)

		

		robot_2.rotatetask(angle_robot,direction_robot,0)
		robot_2.move2goal(first_robot_coordinate_x,first_robot_coordinate_y,0.05)
		time.sleep(1)
			
		print("First robot")
		robot_0.move2goal(second_robot_coordinate_x,second_robot_coordinate_y,0.05)
		time.sleep(1)
		print("Second robot")
		robot_1.move2goal(third_robot_coordinate_x,third_robot_coordinate_y,0.05)
		time.sleep(1)


		reset_positions_client = rospy.ServiceProxy('/reset_positions', Empty)


		reset_positions_client()
		

	except rospy.ROSInterruptException:
		pass


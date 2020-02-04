#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_msgs.msg import TFMessage
import matplotlib
import matplotlib.pyplot as plt
RATE= 10
#Robot's velociy
LIN_VEL = 0.2
#State estimate parameters initialization
F=1
Q=1
R=1

class StateEstimate:
    def __init__(self):
        #initializing the publishers
        self.tf_pub = rospy.Publisher("/tf", TFMessage, queue_size=1)
        self.cmd_pub=rospy.Publisher("/cmd_vel",Twist, queue_size=0)
        self.bot_pose=rospy.Publisher("/StateEstimatePose", PoseWithCovarianceStamped, queue_size=1)
        self.initpose_pub=rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
        #initializing the subscribers
        rospy.Subscriber("/cmd_vel", Twist, self.vel_callback, queue_size=1)
        rospy.Subscriber("/pose", PoseStamped, self.pose_callback, queue_size=1)
        rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.init_pose_callback, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size=1)
        #Initializing the class' variables
        self.rate=rospy.Rate(RATE)
        self.vel=LIN_VEL
        #Robot's initial pose
        self.initpose=0
        #Robot's LIDAR reading to the front
        self.front_dist=0
        #Robot state's uncertainty
        self.p_old=1000
        self.p_old_vel=1000
        #Robot's pose before the propagation and update calculated from the /pose topic
        self.x_before_estimate=0
        #Robot's pose before the propagation and update calculated from (vel*delta_t)
        self.x_before_estimate_vel=0
        #Robot's pose
        self.x_from_pose=0
        self.flag=0
        #Robot's firs pose from the bag file
        self.first_pose=0
        #list of delta time between robot's initial move and current time
        self.time_set=[]
        #list of pose from subscribing to /pose topic
        self.pose_set=[]
        #list the estimated state from subscribing to cmd_vel and scan
        self.scan_vel=[]
        #list the estimated state from subscribing to pose and scan
        self.scan_pose=[]
        
    def pose_callback(self, msg):
        if(self.flag==0):
            #save the first reading
            self.first_pose= msg.pose.position.x
            self.flag=1
        #Continiously read the robot's pose and subtract it from the first reading to initialize the pose to start from zero
        self.x_from_pose= msg.pose.position.x-self.first_pose

    def vel_callback(self, vel_msg):
        #subscribing to the robot's vel
        self.vel=vel_msg.linear.x

    def scan_callback(self, scan_msg):	
        #The LIDAR reading in front of the robot
        self.front_dist=scan_msg.ranges[0]

    def init_pose_callback(self, initpose_msg):
        #Reading the init pose of the robot
        print("Robot's initial pose")	
        print(initpose_msg.pose.pose.position.x)

    def spin(self):
        #the following lines are to initialize the robot's pose to position 0 by publishing to the /initialpose topic
        rospy.sleep(0.1)
        pub_initpose_msg= PoseWithCovarianceStamped()
        pub_initpose_msg.header.stamp=rospy.Time.now()
        pub_initpose_msg.header.frame_id="odom2"
        pub_initpose_msg.pose.pose.position.x= self.initpose
        self.initpose_pub.publish(pub_initpose_msg)

        #set up a twist message
        vel_msg= Twist()
        vel_msg.linear.x=LIN_VEL
        #publishing the velocity message
        self.cmd_pub.publish(vel_msg)

        #set up a pose message to publish Robot's estimated state to /StateEstimatePose topic
        pose_msg= PoseWithCovarianceStamped()
        pose_msg.header.stamp=rospy.Time.now()
        pose_msg.header.frame_id="odom2"

        #start time of the robot
        t0 = rospy.Time.now().to_sec()
        u=LIN_VEL	
        B=0

        while not rospy.is_shutdown():
            #Start estimating robot's state only when the robot start moving
            if(self.x_from_pose<.01):
                t0 = rospy.Time.now().to_sec()
                continue

            #Stop the state esimation process when the robot stop moving
            if(self.x_from_pose>0.95):
                break

            #Skip the inf reading from the LIDAR
            if(self.front_dist==float('inf')):
                continue

            #Start the state estimation process
            t1= rospy.Time.now().to_sec()
            #B is the delta t
            B= t1-t0
            self.time_set.append(B)
            self.pose_set.append(self.x_from_pose)
            #update Robot's propagation state
            x_prop_estimate=F*self.x_before_estimate+self.x_from_pose
            x_prop_estimate_vel=F*self.x_before_estimate_vel+B*u
            #print(x_prop_estimate)
            #update states' uncertainty
            p_prop=self.p_old+Q
            p_prop_vel=self.p_old_vel+Q
            #print(p_prop)
            #robot's observation from the LIDAR reading
            z_robot= self.front_dist
            #print(z_robot)
            #Robot's state observation
            z_real=2-z_robot
            #print(z_real)
            #Robot's estimated observation
            z_estimate=x_prop_estimate
            z_estimate_vel=x_prop_estimate_vel
            #print(z_estimate)
            #calculate the difference between the actual observation and the estimated observation
            r=z_real-z_estimate
            r_vel=z_real-z_estimate_vel
            #print(r)
            #calculate stats' covariance
            S=p_prop+R
            S_vel=p_prop_vel+R
            #print(S)
            #Computer uncertainty gain
            k=float(p_prop)/S
            k_vel=float(p_prop_vel)/S_vel
            #print(k)
            #Robot's state update
            x_update_estimate=x_prop_estimate+float(k)*r
            x_update_estimate_vel=x_prop_estimate_vel+float(k_vel)*r_vel
            print("estimated state from /pose and /scan")
            print(x_update_estimate)
            print("estimated state from /cmd_vel and /scan")
            print(x_update_estimate_vel)
            #update state's uncertainty
            p_update=p_prop-float(p_prop*p_prop)/S
            p_update_vel=p_prop_vel-float(p_prop_vel*p_prop_vel)/S_vel
            #print(p_update)
            #set the old state estimate value and the old uncertainty value to the current calculated values before updating again
            self.x_before_estimate=x_update_estimate
            self.x_before_estimate_vel=x_update_estimate_vel
            #print(self.x_before_estimate)
            self.p_old=p_update
            self.p_old_vel=p_update_vel
                    
            #publishing the estimated state after every update to /StateEstimatePose topic
            self.rate.sleep()
            pose_msg.pose.pose.position.x= x_update_estimate
            pose_msg.pose.covariance[0]= S
            self.bot_pose.publish(pose_msg)
            self.scan_pose.append(x_update_estimate)
            self.scan_vel.append(x_update_estimate_vel)

            #publishing a transformation between odom_kf frame and base_footpring frame
            t = TransformStamped()
            t.header.frame_id = "odom_kf"
            t.header.stamp = rospy.Time.now()
            t.child_frame_id = "base_footprint"
            t.transform.translation.x = x_update_estimate
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            trans = TFMessage([t])
            self.tf_pub.publish(trans)
       
        #plotting commands
        plt.plot(self.time_set, self.pose_set, label="a")
        plt.plot(self.time_set, self.scan_pose, label="b")
        plt.plot(self.time_set, self.scan_vel, label="c")
        plt.plot([self.time_set[0], self.time_set[len(self.time_set)-1]],[0, 0.96],'ko',label="f")
        plt.plot([self.time_set[0], self.time_set[len(self.time_set)-1]],[self.pose_set[0],self.pose_set[len(self.pose_set)-1]-0.96],'bx',label="error in a")
        plt.plot([self.time_set[0], self.time_set[len(self.time_set)-1]],[self.scan_pose[0],self.scan_pose[len(self.scan_pose)-1]-0.96],'r^',label="error in b")
        plt.plot([self.time_set[0], self.time_set[len(self.time_set)-1]],[self.scan_vel[0],self.scan_vel[len(self.scan_vel)-1]-0.96],'g+',label="error in c")
        plt.title("State Estimate plot")
        plt.legend(loc='upper left')
        plt.xlabel('Time')
        plt.ylabel('Distance')
        plt.savefig("pose.png")
        plt.show()


if __name__ == "__main__":
    #initialize a node to move forward
    rospy.init_node("State_Estimate")
    #Create an object from the MoveForward class
    r=StateEstimate()
    #Sleep for 2 sec after the initialization step and before the excution
    rospy.sleep(0.01)
    r.spin()

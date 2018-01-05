#!/usr/bin/env python

import cv2
import rospy
from ardrone_autonomy.msg import Navdata
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PointStamped, Twist
from visualization_msgs.msg import Marker
from std_msgs.msg import Empty

RED = (0,0,255)
BLUE = (255,0,0)
GREEN = (0,255,0)


class Node:
    #init origin of the camera, will be adjusted after camera_info first publishes
    x_origin = -1
    y_origin = -1
    origin = (x_origin,y_origin)
    #save camera sizes for future ref
    x_size = -1
    y_size = -1
    #merker location in frame
    marker_x = -1
    marker_y = -1
    marker_z = 0.8
    #previous marker location in frame, to reset movement
    prev_marker_x = -1
    prev_marker_y = -1
    prev_marker_z = 0.8
    #bounds for the deadzone where the drone won't move
    deadzone_min_bounds = (-1,-1,0.8)
    deadzone_max_bounds = (-1,-1,1.2)
    #hold the drone status
    status = -1

    def __init__(self):
        #openCV brdge for image functions
        self.bridge = CvBridge()

        #navdata = what the drone currenty THINKs it's position is
        self.navdata_sub = rospy.Subscriber("/ardrone/navdata",Navdata,self.RawNavdataCallback)
        #drone's camera feed
        self.image_sub = rospy.Subscriber("/ardrone/image_raw",Image,self.RawImageCallback)
        self.caminfo_sub = rospy.Subscriber("/ardrone/camera_info",CameraInfo,self.CameraConfigCallback)
        #subscribe to Aruco tag info
        self.aruco_sub = rospy.Subscriber("/aruco_single/pixel",PointStamped,self.MarkerPixelCallback)
        self.aruco_sub = rospy.Subscriber("/aruco_single/pose",PoseStamped,self.MarkerPoseCallback)
        #debug camera feed
        #self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.RawImageCallback)
        #self.caminfo_sub = rospy.Subscriber("/usb_cam/camera_info",CameraInfo,self.CameraConfigCallback)
        
        # Land the drone if we are shutting down
        rospy.on_shutdown(self.Shutdown)
        # Allow the controller to publish to the /ardrone/takeoff, land and reset topics
        self.pubLand = rospy.Publisher('/ardrone/land',Empty,queue_size=0)
        self.pubTakeoff = rospy.Publisher('/ardrone/takeoff',Empty,queue_size=0)
        self.pubReset   = rospy.Publisher('/ardrone/reset',Empty,queue_size=0)
		# Allow the controller to publish to the /cmd_vel topic and thus control the drone
        self.pubCommand = rospy.Publisher('/cmd_vel',Twist,queue_size=0)



    def CameraConfigCallback(self,data):
        #read camera dimensions
        self.x_size = data.width
        self.y_size = data.height
        # set origin to center of image
        self.x_origin = self.x_size / 2
        self.y_origin = self.y_size / 2
        self.origin = (self.x_origin,self.y_origin)
        print("Origin = [ "+str(self.x_origin)+" , "+str(self.y_origin)+" ]")
        print("Size = [ "+str(self.x_size)+" , "+str(self.y_size)+" ]")
        
        # set bounds to 1/3 off from origin in top-left and bottom-right
        self.deadzone_min_bounds = ((self.x_origin/3)*2, (self.y_origin/3)*2, self.deadzone_min_bounds[2])
        self.deadzone_max_bounds = ((self.x_origin+(self.x_origin/3)),(self.y_origin+(self.y_origin/3)),self.deadzone_max_bounds[2])
        print("Deadzone = [ "+str(self.deadzone_min_bounds)+" - "+str(self.deadzone_max_bounds)+" ]")
        #stop subscribing to cam_info now that we know the dimensions
        self.caminfo_sub.unregister()



    def RawImageCallback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        #draw overlays
        radius = 10
        #draw rectangle for deadzone
        # image, top left, bottom right, color, thickness
        if self.deadzone_max_bounds[0] is -1:
            pass
        else:
            twodmin = (self.deadzone_min_bounds[0], self.deadzone_min_bounds[1])
            twodmax = (self.deadzone_max_bounds[0], self.deadzone_max_bounds[1])
            cv2.rectangle(cv_image,twodmin,twodmax,RED,2)

        #cv2.circle(img, center, radius, color, thickness=1, lineType=8, shift=0)
        #draw a circle at the Center of tag
        if self.marker_x is -1:
            #print("nope")
            pass
        else:
            cv2.circle(cv_image,(self.marker_x,self.marker_y),radius,GREEN,2)
        #display window
        cv2.namedWindow('DroneFollow', cv2.WINDOW_NORMAL)
        cv2.imshow("DroneFollow", cv_image)
        #grab key for takeoff/shutdown        
        key = cv2.waitKey(1) & 255
        if key == 32: # t to takeof
            self.Shutdown()
        elif key == 116: # space to land
            self.Takeoff()
        #reset marker position in case it disappears
        # if self.prev_marker_x == self.marker_x and self.prev_marker_y == self.marker_y and self.prev_marker_z == self.marker_z:
        #     print("lost Marker!")
        #     self.marker_x = self.x_origin
        #     self.marker_y = self.y_origin
        #     self.marker_z = 0.8


    def RawNavdataCallback(self,data):
        if data.state != self.status:
            self.status = data.state
            print("Changed State: " + str(self.status))

    def MarkerPoseCallback(self,data):
        self.marker_z = data.pose.position.z

    def MarkerPixelCallback(self,data):
        #set previous position
        self.prev_marker_x = self.marker_x
        self.prev_marker_y = self.marker_y
        self.prev_marker_z = self.marker_z
        self.marker_x = int(data.point.x)
        self.marker_y = int(data.point.y)
        self.AdjustPosition()

    def AdjustPosition(self):
        # -linear.x: move backward
        # +linear.x: move forward
        # -linear.y: move right
        # +linear.y: move left
        # -linear.z: move down
        # +linear.z: move up
        # -angular.z: turn right
        # +angular.z: turn left
        command = Twist()
        if self.marker_x < self.deadzone_min_bounds[0]:
            #print("need to move left in x marker = " + str(self.marker_x))
            command.linear.y = 0.1
            pass
        elif self.marker_x > self.deadzone_max_bounds[0]:
            #print("need to move right in x marker = " + str(self.marker_x))
            command.linear.y = -0.1
            pass
        else:
            command.linear.y = 0
            pass

        if self.marker_y < self.deadzone_min_bounds[1]:
            #print("need to move up in y marker = " + str(self.marker_y))
            command.linear.z = 0.2
            pass
        elif self.marker_y > self.deadzone_max_bounds[1]:
            #print("need to move down in y marker = " + str(self.marker_y))
            command.linear.z = -0.2
            pass
        else:
            command.linear.z = 0
            pass
        
        if self.marker_z < self.deadzone_min_bounds[2]:
            #print("need to move backward in z marker = " + str(self.marker_z))
            command.linear.x = -0.1
            pass
        elif self.marker_z > self.deadzone_max_bounds[2]:
            #print("need to move forward in z marker = " + str(self.marker_z))
            command.linear.x = 0.1
            pass
        else:
            command.linear.x = 0
            pass
        #print(command)
        self.pubCommand.publish(command)

    def Takeoff(self):
        self.pubTakeoff.publish(Empty())
        print("FLY!!!!")

    def Shutdown(self):
        self.pubLand.publish(Empty())
        print("landing!")

if __name__ == "__main__":
    rospy.init_node("lab2_example")
    n = Node()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.signal_shutdown("Shutting Down")

    #kill everything once we're done
    cv2.destroyAllWindows()

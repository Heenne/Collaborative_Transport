#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
#from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty,EmptyRequest
from ps4_controller import PlayStationHandler

class PlayStationMultiplexer():

    def __init__(self):
        rospy.init_node('controller_ros_node', anonymous=False)
        self.__sub_joy = rospy.Subscriber('joy', Joy, self.joy_callback)

        self.__buttons_old=[0]*4

        
        self.is_twist_stamped=True
        
        if self.is_twist_stamped:
            self.cmd_vel_pub_miranda = rospy.Publisher('/mur/mir/cmd_vel', TwistStamped, queue_size=1) 
            self.cmd_vel_pub_mur = rospy.Publisher('/miranda/mir/cmd_vel', TwistStamped, queue_size=1)
            self.twist_msg = TwistStamped()
        else:
            self.cmd_vel_pub_miranda = rospy.Publisher('/mur/mir/cmd_vel', Twist, queue_size=1) 
            self.cmd_vel_pub_mur = rospy.Publisher('/miranda/mir/cmd_vel', Twist, queue_size=1)
            self.twist_msg = Twist()
       

        # Default values
        self.speed_vx = 0.1
        self.speed_w = 0.2
        self.state = 1
        self.maxStates = 2 


    def run(self):
        print("run")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.state == 1:
                self.cmd_vel_pub_miranda.publish(self.twist_msg)
            elif self.state == 2:
                self.cmd_vel_pub_mur.publish(self.twist_msg)
            elif self.state == 2:
                self.cmd_vel_pub_miranda.publish(self.twist_msg)
                self.cmd_vel_pub_mur.publish(self.twist_msg)
            rate.sleep()

    def joy_callback(self,data):
        self.x = abs(data.axes[5] - 1) - abs(data.axes[2] - 1) #data.axes[1] + data.axes[4]
        self.w = data.axes[0] + data.axes[3]
        self.changeMode = data.buttons[10]
        
        
        if (data.buttons[3] == 1 and self.__buttons_old[3] == 0):
            self.speed_vx *= 1.1

        if (data.buttons[0] == 1 and self.__buttons_old[0] == 0):
            self.speed_vx *= 0.9

        if (data.buttons[1] == 1 and self.__buttons_old[1] == 0):
            self.speed_w *= 1.1

        if (data.buttons[2] == 1 and self.__buttons_old[2] == 0):
            self.speed_w *= 0.9

        if (self.changeMode == 1 and self.changeMode_old == 0):
            self.state += 1
            if self.state > self.maxStates:
                self.state = 1

        if self.is_twist_stamped:
            self.twist_msg.twist.linear.x = self.x * self.speed_vx
            self.twist_msg.twist.angular.z = self.w * self.speed_w
        else:
            self.twist_msg.linear.x = self.x * self.speed_vx
            self.twist_msg.angular.z = self.w * self.speed_w





        print(self.x, self.w,self.speed_vx, self.speed_w)
        # save old state
        self.changeMode_old = data.buttons[10]
        self.__buttons_old = data.buttons




class PlayStationStateMachineHandler(PlayStationHandler):
    def __init__(self,message_type,button_clients):
        PlayStationHandler.__init__(self)
        self.__rate=rospy.Rate(10)

        self.__clientlist=[rospy.ServiceProxy(client,Empty) for client in button_clients]
        
        self.__speed_translation = rospy.get_param("~translation",0.1)
        self.__speed_rotation =  rospy.get_param("~rotation",0.2)
        self.__trans_incr=rospy.get_param("~trans_incr",0.1)
        self.__rot_incr=rospy.get_param("~rot_incr",0.1)
        self.__enable_cmd_vel=rospy.get_param("~allow_cmd_vel",True)

        self._private_services={"change_rot","change_trans","switch_incr_decr"}
        self.__increase=False
       
        self.__translation = float()
        self.__rotation = float()
        
        self.__publishFunction=None        
            
        if message_type==Twist:
            print("Publishing as Twist")
            self.__publisher=rospy.Publisher("cmd_vel",message_type,queue_size= 10)
            self.__publishFunction=self.__publishTwist__
        elif  message_type==TwistStamped:
            print("Publishing as TwistStamped")
            self.__publisher=rospy.Publisher("cmd_vel",message_type,queue_size=10)
            self.__publishFunction=self.__publishTwistStamped__


    def __publishTwist__(self):
        msg=Twist()
        msg.linear.x=self.__translation
        msg.angular.z=self.__rotation
        
        self.__publisher.publish(msg)

    def __publishTwistStamped__(self):
        msg=TwistStamped()
        msg.twist.linear.x=self.__translation
        msg.twist.angular.z=self.__rotation
        
        self.__publisher.publish(msg)

    def run(self):
        while not rospy.is_shutdown(): 
            for i,edge in enumerate(self._edges):
                if edge:
                    try:
                        self.__clientlist[i].call(EmptyRequest())
                    except Exception as ex:
                        print(ex)
                        pass

            self.__translation = (abs(self._axes[5] - 1) - abs(self._axes[2] - 1)) *self.__speed_translation #data.axes[1] + data.axes[4]
            self.__rotation = (self._axes[0] + self._axes[3])*self.__speed_translation

            if self.__enable_cmd_vel:
                self.__publishFunction()
            self.__rate.sleep()            
            
        
   
        
        

if __name__ == '__main__':
    try:
      #rospy.loginfo("Starting Controller Node")
      PS4_ros = PlayStationMultiplexer()
      PS4_ros.run()
      rospy.spin()

    except rospy.ROSInterruptException:
        print("das hat nicht geklaptt")
      #rospy.loginfo( "ps4_ros node terminated.")


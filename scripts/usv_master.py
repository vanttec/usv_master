#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose2D
import subprocess

#TO DO 
#Cambiar tipo de dato STATUS
#Dejas



class USVMaster:
    def __init__(self):

        self.xbee_message= ""
        self.mission_status = 0
        self.start_mission = False
        self.stop_mission = False

        #Data Input Status
        rospy.Subscriber("/usv_comms/boat_transceiver/course_config", String, self.mission_callback) 
        rospy.Subscriber("/mission_status", Int32, self.mission_status_callback)
        rospy.Subscriber("/usv_comms/boat_transceiver/stop_mission", Bool, self.stop_mission_callback)
        rospy.Subscriber("/usv_comms/boat_transceiver/start_mission", Bool , self.start_mission_callback)

        ##Para mandar el status de las misiones

        self.dataframe_pub = rospy.Publisher('/vanttec_usv/usv_master/usv_data', String, queue_size=10)
        self.boat_current_mission_pub = rospy.Publisher('/usv_master/usv_master/general_status', String, queue_size=10) #<<--CHECK-->>#
    
    def mission_callback(self,xbee_message):
        self.xbee_message = xbee_message.data
        rospy.loginfo(self.xbee_message)
    
    def mission_status_callback(self,mission_status):
        self.mission_status = mission_status.data
    
    def start_mission_callback(self,start_mission):
        self.start_mission=start_mission.data

    def stop_mission_callback(self,stop_mission):
        self.stop_mission = stop_mission.data

def main():
    rospy.init_node('usv_master', anonymous=False)
    rate = rospy.Rate(100)
    usv_master = USVMaster()

    #CUrrentMission Node 
    current_node=None 
    
    while not rospy.is_shutdown():    
            #COURSE MESSAGES
        #rospy.loginfo("mensaje leido = " + usv_master.xbee_message)
      
        if (usv_master.xbee_message)== "CA" or (usv_master.xbee_message)== "ca" :
            usv_master.boat_current_mission_pub.publish("Course Alpha")

        elif (usv_master.xbee_message)== "CB" or (usv_master.xbee_message)== "cb":
            usv_master.boat_current_mission_pub.publish("Course Beta")
            
        elif (usv_master.xbee_message)== "CC" or  (usv_master.xbee_message)== "cc":
            usv_master.boat_current_mission_pub.publish("Course Charlie")
            
            #MISSION MESSAGES

        elif (usv_master.xbee_message)== "AN" or  (usv_master.xbee_message)== "an":
            current_node = subprocess.Popen("rosrun rb_missions auto_nav_position.py", shell = True)
            usv_master.xbee_message= ""
            print(usv_master.xbee_message)
            while usv_master.mission_status != 1 and  usv_master.stop_mission!=True and not rospy.is_shutdown():
                 usv_master.boat_current_mission_pub.publish("Atonomous Navigation")
                 rate.sleep()
            current_node.kill()
                #wait             
        elif (usv_master.xbee_message)== "SP" or (usv_master.xbee_message)== "sp":
            current_node = subprocess.Popen("rosrun rb_missions speed_ch.py", shell = True)
            usv_master.xbee_message= ""
            while usv_master.mission_status != 1 and usv_master.stop_mission!=True and not rospy.is_shutdown():
                #wait
                 usv_master.boat_current_mission_pub.publish("Speed Challenge")
                 rate.sleep()
            current_node.kill()
        usv_master.xbee_message= ""
        rate.sleep()

    if current_node is not None:
        pass
        #current_node.kill()# subprocess.Popen("rosnode kill usv_master", shell = True)
    rospy.spin()
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:     
        pass

#AUTONAV Y SPEED CHALLENGE
#3 CURSOS Y 6 RETOS 
#FUTURO SUSCRIBIRSE A START Y STOP MISSION

#Primero nos suscribimos



#PARA MATAR SUBPROCESS 
#TO 
##self.course_pub = rospy.Publisher("/usv_comms/boat_transceiver/course_config", String, queue_size=10) SUSCRIBIRNOS
#CALL BACK
#
#   DECODIFICAR SWICH CASE 
#MAIN
#  


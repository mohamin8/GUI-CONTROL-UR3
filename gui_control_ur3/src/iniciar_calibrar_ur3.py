#!/usr/bin/env python3
import os

# os.system('cd ../../..')
# #os.system('source devel/setup.bash')
# ip='192.168.2.129'
# comando='roslaunch ur_robot_driver ur3_bringup.launch robot_ip:='+ip+' kinematics_config:="${HOME}/my_robot_calibration.yaml"'
# print(comando)
# os.system(comando)
#os.system('roslaunch ur_robot_driver ur3_bringup.launch robot_ip:=192.168.2.129 kinematics_config:="${HOME}/my_robot_calibration.yaml"')
#!/usr/bin/env python3
import sys
from turtle import position

from sklearn.cluster import MeanShift
# from pruebas_codigo_base import *
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool,MultiArrayLayout
from visualization_msgs.msg import InteractiveMarkerUpdate
from geometry_msgs.msg import Point,Pose
def callback_calibrar(data):
        if data.data!='false':
            pub.publish(False)
            os.system('ls')
            ip=data.data.split(' ')[0]
            archivo_calibracion=data.data.split(' ')[1]
            comando='roslaunch ur_calibration calibration_correction.launch robot_ip:='+ip+' target_filename:="'+archivo_calibracion+'/my_robot_calibration.yaml"'

            os.system(comando)
            pub.publish(True)
            print(comando)

def callback_iniciar_driver(data):
    print(data.data)
    if data.data!='false':
        os.system('ls')
        ip=data.data.split(' ')[0]
        archivo_calibracion=data.data.split(' ')[1]
        comando='roslaunch ur_robot_driver ur3_bringup.launch robot_ip:='+ip+' kinematics_config:="'+archivo_calibracion+'"'

        print(comando)
        os.system(comando)
   
# def info_sim(data):
#     # import pdb; pdb.set_trace()
#     print(type(data.poses))
#     print(data.poses)
#     mi_pose=list()
#     if len(data.poses)>0:
#         print(type(data.poses[0].pose))
#         print(type(data.poses[0].pose.position))
#         print(type(data.poses[0].pose.position.x))

#         print(data.poses[0].pose.position.x)
#         mi_x=data.poses[0].pose.position.x
#         mi_y=data.poses[0].pose.position.y      
#         mi_z=data.poses[0].pose.position.z 
#         while mi_x<3:
#             inpu=input()
#             if inpu=='a':
#                 mi_x=0.0
#             elif inpu=='d':
#                 mi_x-=0.1
#             print(mi_x)
#             punto=Point(x=mi_x,y=mi_y, z=mi_z)
#             pose=Pose(position=punto)
#             mi_pose.append(pose)
#             pub.publish(mi_pose)



if __name__ == "__main__":
    os.system('cd ../../..')

    rospy.init_node('iniciar_calibrar_ur3', anonymous=True)
    # rospy.Subscriber('TEXTO_ESCRITO', String, callback)
    rospy.Subscriber('iniciar_driver', String, callback_iniciar_driver)
    rospy.Subscriber('calibrar', String, callback_calibrar)
    # rospy.Subscriber('/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update', InteractiveMarkerUpdate, info_sim)
    # pub=rospy.Publisher('/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update/poses/pose', MultiArrayLayout, queue_size=10)

    pub = rospy.Publisher('fin_calibrado', Bool, queue_size=10)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
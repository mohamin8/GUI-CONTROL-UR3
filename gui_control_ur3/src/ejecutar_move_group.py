#!/usr/bin/env python3
import os
import subprocess
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
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool,MultiArrayLayout
from visualization_msgs.msg import InteractiveMarkerUpdate
from geometry_msgs.msg import Point,Pose


def callback_ejecutar_move_group(data):

    if data.data:
        # result = subprocess.run(['roslaunch gui_control.ur3 move_group.launch'])
        #decode resultado porque esta en bytes

        # texto=result.stdout.decode('utf-8')
        # print(texto)
        os.system('ls')
        comando='roslaunch gui_control_ur3 demo_sim.launch'
    else:
        comando='roslaunch gui_control_ur3 demo.launch'
    
    pub.publish(True)
    print(comando)
    os.system(comando)
    print('------------------------------------------------------------------------------------')
    # os.system('rosrun gui_control_ur3 cont_imp_pos_codigo_ordenado.py')


if __name__ == "__main__":
    os.system('cd ../../..')
    # comando='roslaunch gui_control_ur3 demo_sim.launch'

    # print(comando)
    # os.system('source devel/setup.bash')

    # os.system('roslaunch gui_control_ur3 demo.launch')
    rospy.init_node('ejecutar_move_group', anonymous=True)
    # rospy.Subscriber('TEXTO_ESCRITO', String, callback)
    pub = rospy.Publisher('iniciar_control', Bool, queue_size=10)

    rospy.Subscriber('move_group_fake_execution', Bool, callback_ejecutar_move_group)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
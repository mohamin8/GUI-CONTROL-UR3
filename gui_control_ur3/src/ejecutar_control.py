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
import time
import rospy
from std_msgs.msg import Bool


def callback_ejecutar_control(data):

    # if data.data:
    #     # result = subprocess.run(['roslaunch gui_control.ur3 move_group.launch'])
    #     #decode resultado porque esta en bytes

    #     # texto=result.stdout.decode('utf-8')
    #     # print(texto)
    #     os.system('ls')
    #     comando='roslaunch gui_control_ur3 demo_sim.launch'
    # else:
    #     comando='roslaunch gui_control_ur3 demo.launch'
    

    # print(comando)
    # os.system(comando)
    print('------------------------------------------------------------------------------------')

    time.sleep(6)
    print('------------------------------------------------------------------------------------')
    os.system('rosrun gui_control_ur3 cont_imp_pos_codigo_ordenado.py')


if __name__ == "__main__":
    os.system('cd ../../..')

    # print(comando)
    # os.system('source devel/setup.bash')

    # os.system('roslaunch gui_control_ur3 demo.launch')
    rospy.init_node('ejecutar_control', anonymous=True)
    # rospy.Subscriber('TEXTO_ESCRITO', String, callback)
    rospy.Subscriber('iniciar_control', Bool, callback_ejecutar_control)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
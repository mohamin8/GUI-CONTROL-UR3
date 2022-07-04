#!/usr/bin/env python3
# importamos las librerias a utilizar
from pickle import NONE
import sys
from xmlrpc.client import Boolean

from matplotlib.colors import is_color_like
import rospy
import moveit_commander
import geometry_msgs.msg
import math
import actionlib
import operator
import numpy as np
import scipy.integrate as integrate
import matplotlib.pyplot as plt
from control_msgs.msg import *
from trajectory_msgs.msg import *
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import Bool
from std_msgs.msg import String, Float32
from geometry_msgs.msg import WrenchStamped
from pykdl_utils.kdl_kinematics import KDLKinematics
from urdf_parser_py.urdf import URDF
#from scipy.fftpack import fft, ifft
#from scipy import signal
from pynput import keyboard as kb
from controller_manager_msgs.srv import SwitchController

import pdb
import sys
from tokenize import String
import rospy
import moveit_commander
import geometry_msgs.msg
import math
import actionlib
import numpy as np
from control_msgs.msg import *
from trajectory_msgs.msg import *
from std_msgs.msg import Int32MultiArray,Bool,String, Float32MultiArray,Float64MultiArray,MultiArrayDimension
from geometry_msgs.msg import WrenchStamped
from controller_manager_msgs.srv import SwitchController
from xml.dom import minidom
import ast
import os
import pdb

# obtener directorio actual para cargar archivos
file_path = os.path.realpath(__file__)
file_path=file_path.split('/')
file_path.remove(file_path[0])
file_path.remove(file_path[-1])
my_path=''
for i in range(len(file_path)):
        print(file_path[i])
        my_path+='/'+file_path[i]
print(my_path)
# Cargamos el archivo urdf

file_name = my_path+"/urdf/ur3.urdf"

# Nombre de las articulaciones
JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
client = None


# Obtenemos el grupo del manipulador entero
arm_group =arm_group = moveit_commander.MoveGroupCommander("manipulator")


# FUNCIONES --------------------------------------------------------------------------------------------------------------------------------

# Funcion de para obtener los parametros del ur3
def urdf():

    # Abrimos el archivo urdf
    f = open(file_name,'r')

    # Creamos el robot a partir del urdf
    robot = URDF.from_xml_string(f.read())

    # Guardamos los links de la base y el extremo
    base_link = robot.get_root()
    end_link = list(robot.link_map.keys())[len(robot.link_map)-1]

    return robot, base_link, end_link

# Funcion para obtener la posicion del robot
def posicion():

    # Definimos la posicion
    pos = arm_group.get_current_pose()

    return pos

# Funcion para obtener el valor de las articulaciones del robot
def articulaciones():

    # Definimos la posicion
    art = arm_group.get_current_joint_values()
    return art

# Funcion para mover en el espacio articular
def mover_articular(art1, art2, art3, art4, art5, art6):

    # Creamos la posicion articular
    pos_art = [art1, art2, art3, art4, art5, art6]

    #Duracion 
    duracion = 4.0

    # Realizamos la trayectoria con la posicion final
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points = [JointTrajectoryPoint(positions = pos_art, velocities = [0]*6, time_from_start = rospy.Duration(duracion))]
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

# Funcion que calcula la posicion deseada dependendiendo de la velocidad calculada
def calculo_pos_d(velocidades_cart, t_int, pos_inicial):

    # Definimos las posiciones iniciales 
    pos_ini_x = (pos_inicial.pose.position.x, 0)
    pos_ini_y = (pos_inicial.pose.position.y, 0)
    pos_ini_z = (pos_inicial.pose.position.z, 0)

    # Calculamos las componentes deseadas x y z
    pos_d_x = tuple(map(operator.add, integrate.quad(lambda x:velocidades_cart[0], 0, t_int), pos_ini_x))
    pos_d_y = tuple(map(operator.add, integrate.quad(lambda x:velocidades_cart[1], 0, t_int), pos_ini_y))
    pos_d_z = tuple(map(operator.add, integrate.quad(lambda x:velocidades_cart[2], 0, t_int), pos_ini_z))

    # Definimos el vector de posicion
    pos_d = []
    pos_d.append(pos_d_x[0])
    pos_d.append(pos_d_y[0])
    pos_d.append(pos_d_z[0])

    return pos_d

# Funcion que realiza el control en velocidad cartesiano
def control_vel_cart(velocidades_cart, t_int, pos_inicial, pos_referencia, kdl_kin):

    # Obtenemos la posicion deseada en funcion de la velocidad
    pos_deseada = calculo_pos_d(velocidades_cart, t_int, pos_referencia)

    # Obtenemos el error entre la posicion deseada y la de referencia
    error_pos =  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    error_pos[0] = pos_deseada[0] - pos_referencia.pose.position.x
    error_pos[1] = pos_deseada[1] - pos_referencia.pose.position.y
    error_pos[2] = pos_deseada[2] - pos_referencia.pose.position.z
 
    # Definimos la velocidad cartesiana de control
    v = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # Obtenemos los valores absolutos de la velocidad cartesiana
    valores = []
    for i in range(0,3):
        valores.append(abs(velocidades_cart[i]))

    # Obtenemos la media y el valor maximo
    media = (valores[0] + valores[1] + valores[2])/3
    maximo = np.amax(valores)

    # Valoramos la importancia de cada componente 
    for i in range(0,3):
        if valores[i] <= media/2:
            v[i] = velocidades_cart[i] + 0.1 * error_pos[i]
        elif valores[i] <= media: 
            v[i] = velocidades_cart[i] + 0.3 * error_pos[i]
        elif valores[i] <= media + (maximo - media)/2:
            v[i] = velocidades_cart[i] + 0.6 * error_pos[i]
        else:
            v[i] = velocidades_cart[i] + 1 * error_pos[i]
    
    #print('----------------------------------------')
    #print('controoool:',v)

    # Devolvemos la posicion
    q_ini = articulaciones()
    
    # Jacobiana de la base
    J_base = kdl_kin.jacobian(q_ini,None)

    # Determinante de la jacobiana
    DJ = np.linalg.det(J_base)

    # Inversa
    J_base_inv = np.linalg.inv(J_base)

    # Calculo velocidad articular
    vel_art = np.matmul(J_base_inv,v)
    vel_art = vel_art.tolist()
    
    # Pasamos de matriz a vector
    vel_articulaciones = []
    for i in range(0,6):
        vel_articulaciones.append(vel_art[0][i])

    return vel_articulaciones, DJ

# Funcion que calcula las velocidades a partir de la fuerzas
def control_admitancia(fuerza_primera, determinante):

    # Definicion de variables
    global fuerza
    global c
    global lista_fuerzasx
    global lista_fuerzasy
    global lista_fuerzasz
    global umbral_fx
    global umbral_fy
    global umbral_fz
    global lim_vel
    print('umbrales', umbral_fx,umbral_fy,umbral_fz)
    print('constante', c)

    velocidades = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    inc = []
    # c = 0.3
    m =11
    cc=c
    # Calculamos la amortiguazion en funcion al determinante
    if determinante > 0:
        
        if (determinante > 0.005):
            cc = cc + 1 / (determinante)  #VALOR NUMERADOR
        else:
            cc = cc + 10 /determinante
        
        # print("funcion de transferencia es:", c)
        # print("determinante en este punto es:", determinante)

    # Definimos la fuuncion de transferencia
    funcion_trans = (1/cc) / (m/cc + 1)

    # Calculamos la velocidad en cada eje
    for i in range(0,3):

        # Incremento de la fuerza
        # incremento_f = fuerza[i]
        incremento_f = (fuerza[i] - fuerza_primera[i])

        inc.append(incremento_f)
        if i==0:
            lista_fuerzasx.append(incremento_f)
            # print('incremento de fuerzaaaaaaa........................                                    ',incremento_f)
            n_fuerzas=len(lista_fuerzasx)
            # umbral_fx=10
            if n_fuerzas>15:
                # Si están dentro de los limites tendra un valor las 5 últimas fuerzas detectadas en este eje
                if (lista_fuerzasx[n_fuerzas-1] >= umbral_fx and lista_fuerzasx[n_fuerzas-1] <= 200.0 and lista_fuerzasx[n_fuerzas-2] >= umbral_fx and lista_fuerzasx[n_fuerzas-2] <= 200.0 and lista_fuerzasx[n_fuerzas-3] >= umbral_fx and lista_fuerzasx[n_fuerzas-3] <= 200.0 and lista_fuerzasx[n_fuerzas-4] >= umbral_fx and lista_fuerzasx[n_fuerzas-5] >= umbral_fx):
                    velocidades[i] = funcion_trans * (lista_fuerzasx[n_fuerzas-1]+lista_fuerzasx[n_fuerzas-2]+lista_fuerzasx[n_fuerzas-3])/3
                    print('..............................se ha superado la fuerza limite.........................................................................................................')
                    print(lista_fuerzasx[n_fuerzas-1],' ',lista_fuerzasx[n_fuerzas-2],' ',lista_fuerzasx[n_fuerzas-3],' ',lista_fuerzasx[n_fuerzas-4],' ',lista_fuerzasx[n_fuerzas-5],' ',lista_fuerzasx[n_fuerzas-6],' ',lista_fuerzasx[n_fuerzas-7],' ',lista_fuerzasx[n_fuerzas-8],' ',lista_fuerzasx[n_fuerzas-9],' ',lista_fuerzasx[n_fuerzas-10])               
                    print('fuerza_inicial.........',fuerza_primera[i])
                    # print(incremento_f1,' ',incremento_f2,' ',incremento_f3,' ',incremento_f4,' ',incremento_f5)
                elif (lista_fuerzasx[n_fuerzas-1] >= -200.0 and lista_fuerzasx[n_fuerzas-1] <= -umbral_fx and lista_fuerzasx[n_fuerzas-2] >= -200.0 and lista_fuerzasx[n_fuerzas-2] <= -umbral_fx and lista_fuerzasx[n_fuerzas-3] >= -200.0 and lista_fuerzasx[n_fuerzas-3] <= -umbral_fx and lista_fuerzasx[n_fuerzas-4] <= -umbral_fx and lista_fuerzasx[n_fuerzas-5] <= -umbral_fx):
                    velocidades[i] = funcion_trans * (lista_fuerzasx[n_fuerzas-1]+lista_fuerzasx[n_fuerzas-2]+lista_fuerzasx[n_fuerzas-3])/3
                    print('..............................se ha superado la fuerza limite.........................................................................................................')
                    print(lista_fuerzasx[n_fuerzas-1],' ',lista_fuerzasx[n_fuerzas-2],' ',lista_fuerzasx[n_fuerzas-3],' ',lista_fuerzasx[n_fuerzas-4],' ',lista_fuerzasx[n_fuerzas-5],' ',lista_fuerzasx[n_fuerzas-6],' ',lista_fuerzasx[n_fuerzas-7],' ',lista_fuerzasx[n_fuerzas-8],' ',lista_fuerzasx[n_fuerzas-9],' ',lista_fuerzasx[n_fuerzas-10])
                    print('fuerza_inicial.........',fuerza_primera[i])

                    # print(incremento_f1,' ',incremento_f2,' ',incremento_f3,' ',incremento_f4,' ',incremento_f5)

                else:
                    velocidades[i] = 0.0
            else:
                print(n_fuerzas)
            # print('velocidades----------------------------',velocidades)
        elif i==1:
            lista_fuerzasy.append(incremento_f)
            n_fuerzas=len(lista_fuerzasy)
            # umbral_fy=1000000
            if n_fuerzas>15:
                # Si están dentro de los limites tendra un valor las 5 últimas fuerzas detectadas en este eje
                if (lista_fuerzasy[n_fuerzas-1] >= umbral_fy and lista_fuerzasy[n_fuerzas-1] <= 200.0 and lista_fuerzasy[n_fuerzas-2] >= umbral_fy and lista_fuerzasy[n_fuerzas-2] <= 200.0 and lista_fuerzasy[n_fuerzas-3] >= umbral_fy and lista_fuerzasy[n_fuerzas-3] <= 200.0 and lista_fuerzasy[n_fuerzas-4] >= umbral_fy and lista_fuerzasy[n_fuerzas-5] >= umbral_fy):
                    velocidades[i] = funcion_trans * (lista_fuerzasy[n_fuerzas-1]+lista_fuerzasy[n_fuerzas-2]+lista_fuerzasy[n_fuerzas-3])/3
                    print('..............................se ha superado la fuerza limite.........................................................................................................')
                    print(lista_fuerzasy[n_fuerzas-1],' ',lista_fuerzasy[n_fuerzas-2],' ',lista_fuerzasy[n_fuerzas-3],' ',lista_fuerzasy[n_fuerzas-4],' ',lista_fuerzasy[n_fuerzas-5],' ',lista_fuerzasy[n_fuerzas-6],' ',lista_fuerzasy[n_fuerzas-7],' ',lista_fuerzasy[n_fuerzas-8],' ',lista_fuerzasy[n_fuerzas-9],' ',lista_fuerzasy[n_fuerzas-10])               
                    print('fuerza_inicial.........',fuerza_primera[i])
                    # print(incremento_f1,' ',incremento_f2,' ',incremento_f3,' ',incremento_f4,' ',incremento_f5)
                elif (lista_fuerzasy[n_fuerzas-1] >= -200.0 and lista_fuerzasy[n_fuerzas-1] <= -umbral_fy and lista_fuerzasy[n_fuerzas-2] >= -200.0 and lista_fuerzasy[n_fuerzas-2] <= -umbral_fy and lista_fuerzasy[n_fuerzas-3] >= -200.0 and lista_fuerzasy[n_fuerzas-3] <= -umbral_fy and lista_fuerzasy[n_fuerzas-4] <= -umbral_fy and lista_fuerzasy[n_fuerzas-5] <= -umbral_fy):
                    velocidades[i] = funcion_trans * (lista_fuerzasy[n_fuerzas-1]+lista_fuerzasy[n_fuerzas-2]+lista_fuerzasy[n_fuerzas-3])/3
                    print('..............................se ha superado la fuerza limite.........................................................................................................')
                    print(lista_fuerzasy[n_fuerzas-1],' ',lista_fuerzasy[n_fuerzas-2],' ',lista_fuerzasy[n_fuerzas-3],' ',lista_fuerzasy[n_fuerzas-4],' ',lista_fuerzasy[n_fuerzas-5],' ',lista_fuerzasy[n_fuerzas-6],' ',lista_fuerzasy[n_fuerzas-7],' ',lista_fuerzasy[n_fuerzas-8],' ',lista_fuerzasy[n_fuerzas-9],' ',lista_fuerzasy[n_fuerzas-10])
                    print('fuerza_inicial.........',fuerza_primera[i])

                    # print(incremento_f1,' ',incremento_f2,' ',incremento_f3,' ',incremento_f4,' ',incremento_f5)

                else:
                    velocidades[i] = 0.0
            else:
                print(n_fuerzas)
            # print('velocidades----------------------------',velocidades)
        else:
            lista_fuerzasz.append(incremento_f)
            n_fuerzas=len(lista_fuerzasz)
            # umbral_fz=20000000
            if n_fuerzas>15:
                # Si están dentro de los limites tendra un valor las 5 últimas fuerzas detectadas en este eje
                if (lista_fuerzasz[n_fuerzas-1] >= umbral_fz and lista_fuerzasz[n_fuerzas-1] <= 200.0 and lista_fuerzasz[n_fuerzas-2] >= umbral_fz and lista_fuerzasz[n_fuerzas-2] <= 200.0 and lista_fuerzasz[n_fuerzas-3] >= umbral_fz and lista_fuerzasz[n_fuerzas-3] <= 200.0 and lista_fuerzasz[n_fuerzas-4] >= umbral_fz and lista_fuerzasz[n_fuerzas-5] >= umbral_fz):
                    velocidades[i] = funcion_trans * (lista_fuerzasz[n_fuerzas-1]+lista_fuerzasz[n_fuerzas-2]+lista_fuerzasz[n_fuerzas-3])/3
                    print('..............................se ha superado la fuerza limite.........................................................................................................')
                    print(lista_fuerzasz[n_fuerzas-1],' ',lista_fuerzasz[n_fuerzas-2],' ',lista_fuerzasz[n_fuerzas-3],' ',lista_fuerzasz[n_fuerzas-4],' ',lista_fuerzasz[n_fuerzas-5],' ',lista_fuerzasz[n_fuerzas-6],' ',lista_fuerzasz[n_fuerzas-7],' ',lista_fuerzasz[n_fuerzas-8],' ',lista_fuerzasz[n_fuerzas-9],' ',lista_fuerzasz[n_fuerzas-10])               
                    print('fuerza_inicial.........',fuerza_primera[i])
                    # print(incremento_f1,' ',incremento_f2,' ',incremento_f3,' ',incremento_f4,' ',incremento_f5)
                elif (lista_fuerzasz[n_fuerzas-1] >= -200.0 and lista_fuerzasz[n_fuerzas-1] <= -umbral_fz and lista_fuerzasz[n_fuerzas-2] >= -200.0 and lista_fuerzasz[n_fuerzas-2] <= -umbral_fz and lista_fuerzasz[n_fuerzas-3] >= -200.0 and lista_fuerzasz[n_fuerzas-3] <= -umbral_fz and lista_fuerzasz[n_fuerzas-4] <= -umbral_fz and lista_fuerzasz[n_fuerzas-5] <= -umbral_fz):
                    velocidades[i] = funcion_trans * (lista_fuerzasz[n_fuerzas-1]+lista_fuerzasz[n_fuerzas-2]+lista_fuerzasz[n_fuerzas-3])/3
                    print('..............................se ha superado la fuerza limite.........................................................................................................')
                    print(lista_fuerzasz[n_fuerzas-1],' ',lista_fuerzasz[n_fuerzas-2],' ',lista_fuerzasz[n_fuerzas-3],' ',lista_fuerzasz[n_fuerzas-4],' ',lista_fuerzasz[n_fuerzas-5],' ',lista_fuerzasz[n_fuerzas-6],' ',lista_fuerzasz[n_fuerzas-7],' ',lista_fuerzasz[n_fuerzas-8],' ',lista_fuerzasz[n_fuerzas-9],' ',lista_fuerzasz[n_fuerzas-10])
                    print('fuerza_inicial.........',fuerza_primera[i])

                    # print(incremento_f1,' ',incremento_f2,' ',incremento_f3,' ',incremento_f4,' ',incremento_f5)

                else:
                    velocidades[i] = 0.0
            else:
                print(n_fuerzas)
            # print('velocidades----------------------------',velocidades)
        # Si esta dentro de los limites tendra un valor
        if velocidades[i]>=lim_vel:
            velocidades[i]=lim_vel
        elif velocidades[i]<=-lim_vel:
            velocidades[i]=-lim_vel
    # print('---------------------------------------------')
    # print("determinante en este punto es:", determinante)
    #print ('incrementos:',inc)
    #print('velocidades', velocidades)

    return velocidades,inc,c

# RESPUESTA AL MENSAJE ----------------------------------------------------------------------------------------------------

#callback detección de fuerza
def callback(data):

    # Definimos una varaible global que guarde los componentes fuerza
      
    global fuerza 
    fuerza = [0.0, 0.0, 0.0]

    # # Guardamos las componentes
    fuerza[0] = data.wrench.force.x
    fuerza[1] = data.wrench.force.z
    fuerza[2] = -data.wrench.force.y

# Estado del robot
def callback_robot_ready(data):
    # Definimos una varaible global que indique que el programa en el robot está en marcha
    global robot_ready
    robot_ready=data.data

# que control hay seleccionado: posición o fuerza 
def callback_tipo_control(data):
    global tipo_control
    #variable que indica que está ejecuandose el control de fuerza
    global is_control_on

    tipo_control=data.data
    if tipo_control=='posicion':
        is_control_on=False
#iniciar control de fuerza
def callback_iniciar_control_admitancia(data):
    global is_control_on
    global primera_iteracion
    global segunda_iteracion
    global tercera_iteracion

    is_control_on=data.data
    if not is_control_on:
        primera_iteracion=True
        segunda_iteracion=False
        tercera_iteracion=False


# FUNCIONES POSICION-------------------------------------------------------------------------------------------------------------------------------------

# Funcion para pasar de grados de euler a cuaternio
def eu_to_q(roll, pitch, yaw):

    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qw, qx, qy, qz]

# Funcion para ir a la posicion home
def home():
    

    global controlador
    if controlador=='vel':
        switch_controller(['scaled_pos_joint_traj_controller'],['joint_group_vel_controller'],2, True, 0)
        controlador='pos_art'
    # valores_art=float(data.data)*math.pi/180
    print('homeeeeeeeeeeeee')
    # mover_articular(-math.pi/2, 0.0, -math.pi/2, 0.0, 0.0, 0.0)
    #Situamos el robot en la posicion guardado
    arm_group.set_named_target("up")
    plan = arm_group.go()
# Funcion para mover en el espacio articular
def mover_articular(art1, art2, art3, art4, art5, art6): 

    # Guardar la posicion objetivo
    articulaciones = arm_group.get_current_joint_values()
    print(articulaciones)
    articulaciones[0] = art1
    articulaciones[1] = art2
    articulaciones[2] = art3
    articulaciones[3] = art4
    articulaciones[4] = art5
    articulaciones[5] = art6

    # Mandar el robot a la posicion
    arm_group.set_joint_value_target(articulaciones)
    plan = arm_group.go()

# Funcion para mover en el espacio cartesiano
def mover_cartesiano(x,y,z,roll,pitch,yaw):

    # Obtenemos los cuaternios
    rotacion = eu_to_q(roll, pitch, yaw)

    # Posicion objetivo
    pos_obj = geometry_msgs.msg.Pose()
    pos_obj.orientation.w = rotacion[0]
    pos_obj.orientation.x = rotacion[1]
    pos_obj.orientation.y = rotacion[2]
    pos_obj.orientation.z = rotacion[3]
    pos_obj.position.x = x
    pos_obj.position.y = y
    pos_obj.position.z = z

    # Ejecutamos el movimiento
    arm_group.set_pose_target(pos_obj)
    plan = arm_group.go()

# Funcion para mover el robot un trayectoria controlada
def mover_controlado(x, y, z, roll, pitch, yaw, duracion):
    # Obtenemos los cuaternios
    rotacion = eu_to_q(roll, pitch, yaw)

    # Guarda posicion
    auxiliar = geometry_msgs.msg.Pose()
    auxiliar.orientation.w = rotacion[0]
    auxiliar.orientation.x = rotacion[1]
    auxiliar.orientation.y = rotacion[2]
    auxiliar.orientation.z = rotacion[3]
    auxiliar.position.x = x
    auxiliar.position.y = y
    auxiliar.position.z = z
    
    # Obtenemos el espacio articular
    arm_group.set_joint_value_target(auxiliar, "tool0", True)
    pos_art = arm_group.get_joint_value_target()

    # Realizamos la trayectoria con la posicion final
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points = [JointTrajectoryPoint(positions = pos_art, velocities = [0]*6, time_from_start = rospy.Duration(duracion))]
    
    # client.send_goal(g)
    # try:
    #     client.wait_for_result()
    # except KeyboardInterrupt:
    #     client.cancel_goal()
    #     raise

# Funcion encargarda de la realizacion de un movimiento lineal
def movimiento_lineal(pos_ini, pos_final, duracion):

    # Obtenemos los cuaternios de cada posicion
    rot1 = eu_to_q(pos_ini[3], pos_ini[4], pos_ini[5])
    rot2 = eu_to_q(pos_final[3], pos_final[4], pos_final[5])
    
    # Creamos un objetivo para la action
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES

    # Guardamos la posicion actual del robot
    pos_actual = arm_group.get_current_joint_values()

    # Fragmentamos el movimiento para realizar un bucle precision
    iteraciones = 10
    tiempo = duracion/iteraciones
    for i in range(1,iteraciones+1):
        auxiliar = geometry_msgs.msg.Pose()
        auxiliar.orientation.w = rot1[0] + (rot2[0] - rot1[0])/iteraciones * i
        auxiliar.orientation.x = rot1[1] + (rot2[1] - rot1[1])/iteraciones * i
        auxiliar.orientation.y = rot1[2] + (rot2[2] - rot1[2])/iteraciones * i
        auxiliar.orientation.z = rot1[3] + (rot2[3] - rot1[3])/iteraciones * i
        auxiliar.position.x = pos_ini[0] + (pos_final[0] - pos_ini[0])/iteraciones * i
        auxiliar.position.y = pos_ini[1] + (pos_final[1] - pos_ini[1])/iteraciones * i
        auxiliar.position.z = pos_ini[2] + (pos_final[2] - pos_ini[2])/iteraciones * i

        # Obtenemos la posicion articular actual y objetivo
        arm_group.set_joint_value_target(auxiliar, "tool0", True)
        pos_obj = arm_group.get_joint_value_target()

        # Obtenemos el vector velocidades para los puntos de paso
        vel = []
        for j in range(0,6):
            aux = ((pos_obj[j] - pos_actual[j]) / (duracion/iteraciones))
            vel.append(aux)

        # Anadimos el punto a la trayectoria
        if i == iteraciones+1:
            g.trajectory.points.append(JointTrajectoryPoint(positions = pos_obj, velocities = [0]*6, time_from_start = rospy.Duration(tiempo)))
        else:
            g.trajectory.points.append(JointTrajectoryPoint(positions = pos_obj, velocities = [0]*6, time_from_start = rospy.Duration(tiempo)))

        # Aumentamos el tiempo de ejecucion
        tiempo = tiempo + duracion/iteraciones

        # Actualizamos la posicion actual
        pos_actual = pos_obj

    # Mandamos el objetivo y esperamos a que se realice la accion
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

# Funcion para el movimiento circular
def movimiento_circular(pos_ini, radio, angulo, duracion):

    # Creamos la posicion final
    pos_final = []
    pos_final.append(pos_ini[0] + math.sin(angulo)*radio)
    pos_final.append(pos_ini[1])

    # Dependiendo del angulo de entrada debemos cambiar la definion
    if (angulo > 0):
        pos_final.append(pos_ini[2] + radio * (1 - math.cos(angulo)))
    else:
        pos_final.append(pos_ini[2] - radio * (1 - math.cos(angulo)))

    pos_final.append(pos_ini[3])
    pos_final.append(pos_ini[4])
    pos_final.append(pos_ini[5])

    # Obtenemos los cuaternios de la rotacion
    rot = eu_to_q(pos_ini[3], pos_ini[4], pos_ini[5])

    # Creamos un objetivo para la action
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES

    # Guardamos la posicion actual del robot
    pos_actual = arm_group.get_current_joint_values()

    # Fragmentamos el movimiento para realizar un bucle precision
    iteraciones = 10
    tiempo = duracion/iteraciones
    for i in range(1,iteraciones+1):
        
        # Creamos el punto auxiliar
        auxiliar = geometry_msgs.msg.Pose()
        auxiliar.orientation.w = rot[0]
        auxiliar.orientation.x = rot[1]
        auxiliar.orientation.y = rot[2]
        auxiliar.orientation.z = rot[3]

        # La coordenada y siempre sera la misma    
        auxiliar.position.y = pos_ini[1] + (pos_final[1] - pos_ini[1])/iteraciones * i 

        # Dependiendo del angulo cambiamos las definiciones
        if (angulo > 0):
            auxiliar.position.x = pos_ini[0] + radio * math.sin(angulo/iteraciones* i)
            auxiliar.position.z = pos_ini[2] + radio * (1 - math.cos(angulo/iteraciones * i))
        else:
            auxiliar.position.x = pos_ini[0] + radio * (math.sin(-angulo + angulo/iteraciones*i) - math.sin(-angulo))
            auxiliar.position.z = pos_ini[2] + radio * ((1 - math.cos(-angulo + angulo/iteraciones * i)) - (1 - math.cos(-angulo)))
        
        # Obtenemos la posicion articular actual y objetivo
        arm_group.set_joint_value_target(auxiliar, "tool0", True)
        pos_obj = arm_group.get_joint_value_target()

        # Obtenemos el vector velocidades para los puntos de paso
        vel = []
        for j in range(0,6):
            aux = ((pos_obj[j] - pos_actual[j]) / (duracion/iteraciones))
            vel.append(aux)

        # Anadimos el punto a la trayectoria
        if i == iteraciones+1:
            g.trajectory.points.append(JointTrajectoryPoint(positions = pos_obj, velocities = [0]*6, time_from_start = rospy.Duration(tiempo)))
        else:
            g.trajectory.points.append(JointTrajectoryPoint(positions = pos_obj, velocities = vel, time_from_start = rospy.Duration(tiempo)))

        # Aumentamos el tiempo de ejecucion
        tiempo = tiempo + duracion/iteraciones

        # Actualizamos la posicion actual
        pos_actual = pos_obj

    # Mandamos el objetivo y esperamos a que se realice la accion
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

    return pos_final

# Ejericio 1 define el movimiento de subida y bajada del brazo frontal
# Ejericio 1 define el movimiento de subida y bajada del brazo frontal
def ejercicio_lineal(posiciones,duracion,num_rep):
    print(posiciones)
    print(duracion)
    print(num_rep)
    #Definimos las posiciones

    # Eje X
    #pos_1 = [0.03, 0.45, 0.4, 0, 0, math.pi/2]
    #pos_2 = [-0.23, 0.45, 0.4, 0, 0, math.pi/2]

    # Eje Y
    #pos_1 = [-0.11, 0.2, 0.3, 0, 0, math.pi/2]
    #pos_2 = [-0.11, 0.5, 0.3, 0, 0, math.pi/2]

    # Eje Z
    # pos_1 = [-0.11, 0.45, 0.4, 0, 0, math.pi/2]
    # pos_2 = [-0.11, 0.45, 0.05, 0, 0, math.pi/2]

    # Eje X-Z
    #pos_1 = [0.1, 0.45, 0.4, 0, 0, math.pi/2]
    #pos_2 = [-0.2, 0.45, 0.1, 0, 0, math.pi/2]

    # Eje X-Y
    #pos_1 = [0.2, 0.4, 0.4, 0, 0, math.pi/2]
    #pos_2 = [-0.2, 0.25, 0.4, 0, 0, math.pi/2]
    # Eje Y-Z
    #pos_1 = [-0.11, 0.2, 0.4, 0, 0, math.pi/2]
    #pos_2 = [-0.11, 0.4, 0.2, 0, 0, math.pi/2]

    # Movemos al comienzo de movimiento
    mover_cartesiano(posiciones[0][0],posiciones[0][1],posiciones[0][2],math.radians(posiciones[0][3]),math.radians(posiciones[0][4]),math.radians(posiciones[0][5]))

    # Pedimos el numero de repetaciones y la duracion del movimiento

    # Bucle de ejecucion
    for i in range(0,int(num_rep)):
        for j in range(0,len(posiciones)-1):
            posiciones[i][3]=math.radians(posiciones[i][3])
            posiciones[i][4]=math.radians(posiciones[i][4])
            posiciones[i][5]=math.radians(posiciones[i][5])
            movimiento_lineal(posiciones[i], posiciones[i+1], duracion)
            rospy.sleep(0.5)
        movimiento_lineal(posiciones[len(posiciones)-1], posiciones[0], duracion)
        rospy.sleep(0.5)


def ejercicio_1():

    #Definimos las posiciones

    # Eje X
    #pos_1 = [0.03, 0.45, 0.4, 0, 0, math.pi/2]
    #pos_2 = [-0.23, 0.45, 0.4, 0, 0, math.pi/2]

    # Eje Y
    #pos_1 = [-0.11, 0.2, 0.3, 0, 0, math.pi/2]
    #pos_2 = [-0.11, 0.5, 0.3, 0, 0, math.pi/2]

    # Eje Z
    pos_1 = [-0.11, 0.45, 0.4, 0, 0, math.pi/2]
    pos_2 = [-0.11, 0.45, 0.05, 0, 0, math.pi/2]

    # Eje X-Z
    #pos_1 = [0.1, 0.45, 0.4, 0, 0, math.pi/2]
    #pos_2 = [-0.2, 0.45, 0.1, 0, 0, math.pi/2]

    # Eje X-Y
    #pos_1 = [0.2, 0.4, 0.4, 0, 0, math.pi/2]
    #pos_2 = [-0.2, 0.25, 0.4, 0, 0, math.pi/2]

    # Eje Y-Z
    #pos_1 = [-0.11, 0.2, 0.4, 0, 0, math.pi/2]
    #pos_2 = [-0.11, 0.4, 0.2, 0, 0, math.pi/2]

    # Movemos al comienzo de movimiento
    mover_cartesiano(pos_2[0],pos_2[1],pos_2[2],pos_2[3],pos_2[4],pos_2[5])

    # Pedimos el numero de repetaciones y la duracion del movimiento
    num_rep = int(input("Numero de repeticiones: "))
    duracion = float(input("Tiempo del recorrido: "))
    rospy.sleep(5)
    # Bucle de ejecucion
    for i in range(0,int(num_rep)):
        movimiento_lineal(pos_2, pos_1, duracion)
        rospy.sleep(0.5)
        movimiento_lineal(pos_1, pos_2, duracion)
        rospy.sleep(0.5)

# Ejercicio 2 define el movimiento de subida y bajada del brazo lateral
def ejercicio_circular(pos,radio,angulo,duracion,num_rep):

    # Tiempo de espera
    rospy.sleep(3)


    # Movemos al comienzo de movimiento
    mover_cartesiano(pos[0],pos[1],pos[2],math.radians(pos[3]),math.radians(pos[4]),math.radians(pos[5]))

    #
    rospy.sleep(5)
    # Pasamos el angulo a radianes
    angulo = math.radians(angulo)
    num_rep=int(num_rep)
    # Bucle de ejecucion
    for i in range(0,num_rep):
        pos[3]=math.radians(pos[3])
        pos[4]=math.radians(pos[4])
        pos[5]=math.radians(pos[5])
        pos_1 = movimiento_circular(pos, radio, angulo, duracion)
        rospy.sleep(0.5)
        movimiento_circular(pos_1, radio, -angulo, duracion)
        rospy.sleep(0.5)
def ejercicio_2():

    # Tiempo de espera
    rospy.sleep(3)

    # Definimos la posicion inicial
    pos_2 = [-0.11, 0.45, 0.05, 0, 0, math.pi/2]

    # Movemos al comienzo de movimiento
    mover_cartesiano(pos_2[0],pos_2[1],pos_2[2],pos_2[3],pos_2[4],pos_2[5])

    # Pedimos el numero de repetaciones y la duracion del movimiento
    num_rep = int(input("Numero de repeticiones: "))
    duracion = float(input("Tiempo del recorrido: "))
    radio = float(input("Radio del recorrido: "))
    angulo = float(input("Angulo del recorrido: "))
    rospy.sleep(5)
    # Pasamos el angulo a radianes
    angulo = math.radians(angulo)
    
    # Bucle de ejecucion
    for i in range(0,num_rep):
        pos_1 = movimiento_circular(pos_2, radio, angulo, duracion)
        rospy.sleep(0.5)
        movimiento_circular(pos_1, radio, -angulo, duracion)
        rospy.sleep(0.5)


#callbacks..............................................................
#recibe las posiciones articulares dela interfaz y los ejecuta
def callback_movimiento(data):
    global controlador
    if controlador=='vel':
        switch_controller(['scaled_pos_joint_traj_controller'],['joint_group_vel_controller'],2, True, 0)
        controlador='pos_art'
    # valores_art=float(data.data)*math.pi/180
    valores_articulares=np.asarray(data.data)

    print(type(valores_articulares))
    # valores_articulares=np.vectorize(valores_articulares) 
    #pasamos a radianes y dimidimos entre 10 porque los valores de los sliders van de +-3600
    valores_art=np.multiply(valores_articulares,math.pi/1800)
    print('valores articulares deseados',valores_art)
    mover_articular(valores_art[0],valores_art[1],valores_art[2],valores_art[3],valores_art[4],valores_art[5])

# Funcion para mover cada articulación en un sentido deseado
def callback_mover_articular_vel(data):
    global controlador
    if controlador=='pos_art' and data.data!='home':
    ########prueba enviando velocidades articulares
        switch_controller(['joint_group_vel_controller'], ['scaled_pos_joint_traj_controller'], 2, True, 0)
        print('client cambiado                 ',client.get_state())
        controlador='vel'

    global pub
    global kdl_kin
    t=1/60
    pos_ini = posicion()
    pos_ref=posicion()
    vel_cart_deseada=[0,0,0,0,0,0]

    if data.data=='x+':
        vel_cart_deseada=[0.1,0,0,0,0,0]
    elif data.data=='y+':
        vel_cart_deseada=[0,0.1,0,0,0,0]
    elif data.data=='z+':
        vel_cart_deseada=[0,0,0.1,0,0,0]
    elif data.data=='x-':
        vel_cart_deseada=[-0.1,0,0,0,0,0]
    elif data.data=='y-':
        vel_cart_deseada=[0,-0.1,0,0,0,0]
    elif data.data=='z-':
        vel_cart_deseada=[0,0,-0.1,0,0,0]
    elif data.data=='q1+':
        vel.data=[0.1,0,0,0,0,0]
    elif data.data=='q2+':
        vel.data=[0,0.1,0,0,0,0]
    elif data.data=='q3+':
        vel.data=[0,0,0.1,0,0,0]
    elif data.data=='q4+':
        vel.data=[0,0,0,0.1,0,0]
    elif data.data=='q5+':
        vel.data=[0,0,0,0,0.1,0]
    elif data.data=='q6+':
        vel.data=[0,0,0,0,0,0.1]
    elif data.data=='q1-':
        vel.data=[-0.1,0,0,0,0,0]
    elif data.data=='q2-':
        vel.data=[0,-0.1,0,0,0,0]
    elif data.data=='q3-':
        vel.data=[0,0,-0.1,0,0,0]
    elif data.data=='q4-':
        vel.data=[0,0,0,-0.1,0,0]
    elif data.data=='q5-':
        vel.data=[0,0,0,0,-0.1,0]
    elif data.data=='q6-':
        vel.data=[0,0,0,0,0,-0.1]

    elif data.data=='home':
        print('home')
        home()

    # obtener velocidades articulares a partir de las cartesianas
    if vel_cart_deseada!=[0,0,0,0,0,0] and data.data!='home':
    # # Llamo al control en velocidad
    #     vel_art, determinante = control_vel_cart(vel_cart_deseada, t, pos_ini, pos_ref,kdl_kin)
     # Devolvemos la posicion
        q_ini = articulaciones()
        
        # Jacobiana de la base
        J_base = kdl_kin.jacobian(q_ini,None)

        # Determinante de la jacobiana
        determinante = np.linalg.det(J_base)

        # Inversa
        J_base_inv = np.linalg.inv(J_base)

        # Calculo velocidad articular
        vel_art = np.matmul(J_base_inv,vel_cart_deseada)
        vel_art = vel_art.tolist()
        
        # Pasamos de matriz a vector
        vel.data = []
        for i in range(0,6):
            vel.data.append(vel_art[0][i])

        print(vel.data)
        print(determinante)
        # Cambio velocidades dependiendo del determinante 
        if determinante <= 0.001:
            vel.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        print(vel.data)
    print(vel.data)
    pub.publish(vel)
    



#parar giro enviado por la función  callback_mover_art_vel
def callback_parar_giro(data):
    global pub
    print('cancelado')
    # client.cancel_goal()
    # arm_group.stop()
    vel.data=[0,0,0,0,0,0]

    pub.publish(vel)
def conf_admitancia_inicial():
    global c
    global umbral_fx
    global umbral_fy
    global umbral_fz
    global lim_vel

    c=0.3
    umbral_fx=10
    umbral_fy=10
    umbral_fz=50
    lim_vel=0.5
def callback_configuracion(data):
    global c
    global umbral_fx
    global umbral_fy
    global umbral_fz
    global lim_vel
    c=data.data[0]
    print('he recibido esta constante', c)
    umbral_fx=data.data[1]
    umbral_fy=data.data[2]
    umbral_fz=data.data[3]
    lim_vel=data.data[4]
def cargar_ejercicios():
    global lista_ejercicios
    global ejercicio_actual
    dict={}

    dict['nombre']='Nuevo ejercicio'
    ejercicio_actual=dict
    lista_ejercicios=list()
    # obtener directorio actual para cargar archivos
    file_path = os.path.realpath(__file__)
    file_path=file_path.split('/')
    file_path.remove(file_path[0])
    file_path.remove(file_path[-1])
    mi_path=''
    for i in range(len(file_path)):
            print(file_path[i])
            mi_path+='/'+file_path[i]
    doc = minidom.parse(mi_path+"/ejercicios/ejercicios.xml")
    ejercicios = doc.getElementsByTagName("ejercicio")
    # primer_ejercicio=True

    for ejercicio in ejercicios:
        dict={}

        nombre = ejercicio.getElementsByTagName("nombre")[0]
        dict['nombre']=nombre.firstChild.data


        tipo = ejercicio.getElementsByTagName("tipo")[0]
        dict['tipo']=tipo.firstChild.data

        if tipo.firstChild.data=='lineal':
            #añadir item a desplegable porque por defecto está seleccionado ejerccio lineal
            puntos = ejercicio.getElementsByTagName("punto")
            lista_puntos=list()
            for punto in puntos:
                lista_puntos.append(ast.literal_eval(punto.firstChild.data))
                print(punto.firstChild.data)
            dict['puntos']=lista_puntos
        else:
            punto_inicial = ejercicio.getElementsByTagName("punto_inicial")[0]
            dict['punto_inicial']=ast.literal_eval(punto_inicial.firstChild.data)
            radio = ejercicio.getElementsByTagName("radio")[0]
            dict['radio']=float(radio.firstChild.data)
            angulo = ejercicio.getElementsByTagName("angulo")[0]
            dict['angulo']=float(angulo.firstChild.data)
            print(type(dict['angulo']),dict['angulo'])
        print(dict,'...................')
        lista_ejercicios.append(dict)
        #ponemos como ejercicio seleccionado el primer ejercicio del archivo xml
        # if primer_ejercicio:
        #     ejercicio_actual=dict
        #     primer_ejercicio=False
    print(lista_ejercicios)
def callback_ejercicio_seleccionado(data):
    global lista_ejercicios
    global ejercicio_actual
    if data.data=='Nuevo ejercicio':
        d={}
        d['nombre']='Nuevo ejercicio'
        ejercicio_actual=d

    else:
        for d in lista_ejercicios:
            if d['nombre'] == data.data:
                print(d)
                ejercicio_actual=d
    print(ejercicio_actual)
def callback_ejecutar_ejercicio(data):
    print(data.data)
    global ejercicio_actual
    print(ejercicio_actual)
    print(data.data)
    if ejercicio_actual['nombre']=='Nuevo ejercicio':
        ej=[];nuevo_punto=[];s=0
        for i in range(len(data.data)):
            s+=1
            print(data.data[i])
            nuevo_punto.append(data.data[i])
            print(nuevo_punto)
            if s==6:
                s=0
                nuevo_punto_copia=nuevo_punto.copy()
                ej.append(nuevo_punto_copia)
                print('el ejercicio es', ej)

                nuevo_punto.clear()
        print('el ejercicio es', ej)
        # ejercicio_1()
        #el último elemento representa el tipo de ejercicio
        if ej[-1][-1]==0:
            print('nuevo ejercicio')

            puntos=[]

            for i in range(len(ej)-1):
                puntos.append(ej[i])

            ejercicio_lineal(puntos,ej[-1][0],ej[-1][1])

        else:
            ejercicio_circular(ej[0],data[1][2],data[1][3],ej[1][0],ej[1][1])



    else:
        if ejercicio_actual['tipo']=='lineal':
               ejercicio_lineal(ejercicio_actual['puntos'],data.data[0],data.data[1])


        else:
              ejercicio_circular(ejercicio_actual['punto_inicial'],ejercicio_actual['radio'],ejercicio_actual['angulo'],data.data[0],data.data[1])

        # print(type(ejercicio_actual['punto_inicial']))
        # print(type(ejercicio_actual['radio']))
        # print(type(ejercicio_actual['angulo']))


def callback_parar_ejercicio(data):
    print('parar_ejercicio')
    client.cancel_goal()
    arm_group.stop()

# def callback_mover_cart_vel(data):

    
#     # Llamo al control en velocidad
#     vel_art, determinante = control_vel_cart(vel_cart_deseada, t, pos_ini, pos_ref,kdl_kin)

#     # Cambio velocidades dependiendo del determinante 
#     if determinante >= 0.001:
#         vel.data = vel_art
#     else:
#         vel.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#     pub.publish(vel)
# MAIN ---------------------------------------------------------------------------------------------------------------------
def main():
# Varianbles globales
    cargar_ejercicios()
    conf_admitancia_inicial()
    global client
    global lista_fuerzasx
    global lista_fuerzasy
    global lista_fuerzasz
    global simulacion
    simulación=True
    lista_fuerzasx=[]
    lista_fuerzasy=[]
    lista_fuerzasz=[]
    global fuerza
    global switch_controller
    global parada
    global tipo_control
    tipo_control='posicion'
    global is_control_on
    is_control_on=False

    global pub
    global vel
    global controlador
    controlador='pos_art'
    global kdl_kin
    # Definicion del vector de velocidades del robot
    vel = Float64MultiArray()
    vel.layout.dim.append(MultiArrayDimension())  # speed
    vel.layout.dim[0].label = "speed"
    vel.layout.dim[0].size = 1
    vel.layout.data_offset = 1
    # solucionar con robor
    rospy.init_node('control_impedancia', anonymous=True)
    rospy.Subscriber('/ur_hardware_interface/robot_program_running',Bool, callback_robot_ready)
    rospy.Subscriber('tipo_control',String, callback_tipo_control)
    rospy.Subscriber('control_admitancia',Bool, callback_iniciar_control_admitancia)
    #topics control posicion
    rospy.Subscriber('valores_articulares',Int32MultiArray,callback_movimiento)
    rospy.Subscriber('configuracion',Float32MultiArray,callback_configuracion)
    # mover con botones
    rospy.Subscriber('girar',String,callback_mover_articular_vel)
    # rospy.Subscriber('girar_cartesiano',String,callback_mover_cart_vel)
    rospy.Subscriber('nombre_ejercicio',String,callback_ejercicio_seleccionado)
    rospy.Subscriber('ejecutar_ejercicio',Float32MultiArray,callback_ejecutar_ejercicio)
    rospy.Subscriber('parar_ejercicio',Bool,callback_parar_ejercicio)

    rospy.Subscriber('parar_giro',Bool,callback_parar_giro)
    rospy.Subscriber('tipo_control',String, callback_tipo_control)
    #saber si esta activa solo la simumlación
    # rospy.Subscriber('iniciar_driver',String,callback_iniciar_driver)

    # while not robot_ready:
    #     rospy.spin()
    #     print(robot_ready,"--------------------------noooooooooooooooooooooo------------------------------")
    # print("--------------------------ya se ha conectado el driver del robot---------------------------------")
    # # Esperamos al servicio que para variar entre controladores

    # rospy.wait_for_service('/controller_manager/switch_controller')


    # Comenzamos la ejecucion
    try:
        print('he entrado')
        # Variable de parada negativa
        parada = False
        # Iniciamos el variador de controladores
        switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)

        # Nos relacionamos como publicadores al nodo de velocidades
        pub = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=10)
        pub2 = rospy.Publisher('diferencia_fuerzas',Float32MultiArray, queue_size=10)
        # Realizamos la subscripcion al nodo de fuerzas del extremo del robot
        rospy.Subscriber("/wrench", WrenchStamped, callback)
        
        # Lanzamos el nodo controlador
        # rospy.init_node('control_impedancia', anonymous=True)

        # Cliente para el movimiento articular 

        client = actionlib.SimpleActionClient('/scaled_pos_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        print("Waiting for server...")
        # client.wait_for_server()
        print("Connected to server")
        #real

        # client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)                   
        #Simulado

        # ejercicio_1()
        # ejercicio_2()
        # pdb.set_trace()
        # Ratio de control de 10hz
        frec = 60.0
        rate = rospy.Rate(frec)


        global robot_ready
        robot_ready=False
        global primera_iteracion
        primera_iteracion=True
        global segunda_iteracion
        segunda_iteracion=False
        global tercera_iteracion
        tercera_iteracion=False
        global robot_quieto
        robot_quieto=True
        iniciado_codigo= False
        # Definicion del vector de velocidades del robot
        velocidades = Float64MultiArray()
        velocidades.layout.dim.append(MultiArrayDimension())  # speed
        velocidades.layout.dim[0].label = "speed"
        velocidades.layout.dim[0].size = 1
        velocidades.layout.data_offset = 1

                # Obtenemos los componentes del urdf
        robot, base_link, end_link = urdf()


        # Creamos un objeto KDL
        kdl_kin = KDLKinematics(robot, base_link, end_link)
        # Bucle de control 
        while not rospy.is_shutdown():
            if robot_ready and not iniciado_codigo:
                print('yendo a home')
                home()
                iniciado_codigo=True
            # print(robot_ready,'bucleee robooooooot readyyyyyy')
            # print(is_control_on,tipo_control,robot_ready)
            if is_control_on and tipo_control=='fuerza':
            # if True:
                # print('primer if')
                # print(robot_ready,'robooooooot readyyyyyy')

                if primera_iteracion:
                    print('segundo if')
                    print(robot_ready,'robooooooot readyyyyyy')

                    primera_iteracion=False  
                    segunda_iteracion=True                 
                    # Movemos el robot a la posicion inicial
                    if controlador=='vel':
                        switch_controller(['scaled_pos_joint_traj_controller'],['joint_group_vel_controller'],2, True, 0)
                        controlador='pos_art'
                    #  la buena
                    mover_articular(math.pi/2, -math.pi*56/180, math.pi*115/180, -math.pi*150/180, -math.pi/2, 0)
                    
                    
                    
                    #mover_articular(math.pi/2, -math.pi*3/4, math.pi*3/4, -math.pi, -math.pi/2, 0)
                    # mover_articular(math.pi/2, -math.pi/2, math.pi/2, math.pi/2, -math.pi/2, 0)
                    # mover_articular(0, -math.pi/2, math.pi/2, -math.pi,0, 0)

                    ########prueba enviando velocidades articulares
                    switch_controller(['joint_group_vel_controller'], ['scaled_pos_joint_traj_controller'], 2, True, 0)
                    print('client cambiado                 ',client.get_state())
                    controlador='vel'
                    # Obtenemos la posicion inicial
                    pos_ini = posicion()

                    # Tiempo del bucle
                    t = 1 / frec
                    tt = t
                    
                    # Definicion velocidades cartesianas
                    velocidades_cart = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

                    # # Definicion del vector de velocidades del robot
                    # velocidades = Float64MultiArray()
                    # velocidades.layout.dim.append(MultiArrayDimension())  # speed
                    # velocidades.layout.dim[0].label = "speed"
                    # velocidades.layout.dim[0].size = 1
                    # velocidades.layout.data_offset = 1

                    # # print ('Pulsa cualquier tecla para empezar!!!')
                    # # print ('Para parar pulse la tecla q en cualquier momento')
                    
                    # # # Peticion inicial de paro
                    # # tecla = input()
                    # # if tecla == 'q':
                    # #     rospy.signal_shutdown("KeyboardInterrupt")

                    # Iniciamos el escuchador del teclado
                    # escuchador = kb.Listener(pulsa, suelta)
                    # escuchador.start()
                elif segunda_iteracion:
                    segunda_iteracion=False
                    tercera_iteracion=True
                    fuerzas = []
                    vel_mal = []
                    vel_bien = []
                    tiempos = []

                    # Variables iniciales
                    fuerza_primera = fuerza.copy()
                    print('obtenida_primera fuerza',fuerza)
                    vel_anterior = velocidades_cart
                    determinante = -1

                elif tercera_iteracion:

                    # print('velocidad anterior.............', vel_anterior)
                    #rospy.loginfo(velocidades)
                    tiempo1 = rospy.get_time()

                    # Velocidades cartesianas del extremo
                    velocidades_cart, inc, c = control_admitancia(fuerza_primera, determinante)
                    
                    # Obtengo la referencia de posicion actual de referencia
                    pos_ref = posicion()
                    
                    # Velocidad final que pasarle al robot
                    vel_final = []

                    # Se da cierta importancia al valor anterior
                    for i in range (0,6):
                        error_vel = velocidades_cart[i] - vel_anterior[i]
                        vel_final.append(vel_anterior[i] + error_vel/32.5) # VALOR DIVISION 

                    # Llamo al control en velocidad
                    vel_art, determinante = control_vel_cart(vel_final, t, pos_ini, pos_ref,kdl_kin)
                    vel_anterior = vel_final

                    # Cambio velocidades dependiendo del determinante 
                    if determinante >= 0.001:
                        velocidades.data = vel_art
                    else:
                        velocidades.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

                    # Si se procede a la parada
                    if parada:
                        print('.................................................estoy en parada')
                        velocidades.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

                    #print(velocidades.data)

                    # Publicar velocidades
                    #print("determinante:",determinante)
                    #para que solo actualice la fuerza una vez en la posición estática

                    vel_absoluto =  [abs(ele) for ele in velocidades.data]
    #...................... no mejor jugar con velocidades cartesianas que si darán 00000?
                    if vel_absoluto<=[0.0001,0.0001,0.0001,0.0001,0.0001,0.0001]:
                        if not robot_quieto:
                            print('velocidades                                                  ',vel_absoluto)

                            fuerza_primera=fuerza.copy()
                            print('nueva fuerza primera', fuerza_primera)
                            robot_quieto=True
                    else:
                        robot_quieto=False
                    print('esta es la velocidad que imprimpo coñooo:           ',velocidades.data)
                    pub.publish(velocidades)
                    xyz=0
                    fuerzas.append(inc[xyz])  
                    vel_mal.append(velocidades_cart[xyz])
                    vel_bien.append(vel_final[xyz])
                    tiempos.append(tt - 1/frec)

                    #publicar velocides y diferencias de fueza para plotear en la interfaz, junto al tiempo
                    array=[inc,vel_final[0:3],[tiempos[-1],tiempos[-1],tiempos[-1]],[fuerza_primera[0],fuerza_primera[1],fuerza_primera[2]]]
                    # print(array)
                    # print(type(array))
                    array=np.array(array)
                    array_1d=array.flatten()
                    # print(type(array_1d))
                    array_1d=array_1d.tolist()
                    # print(array_1d)
                    array_float=[float(i) for i in array_1d]
                    # print(array_float)
                    # array=np.array(array)
                    fuerzas_vel = Float32MultiArray(data=array_float)
                    pub2.publish(fuerzas_vel)
                    # Aumento el tiempo total del intervalo
                    tt = tt + 1/frec
                    if velocidades.data == [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] and parada:
                        #plt.plot(tiempos, posiciones)
                        #plt.xlim([0, 5])
                        #plt.ylim([0, 1])
                        #plt.title('Velocidad Eje Y: 0.05')
                        #plt.xlabel('tiempo (s)')
                        #plt.ylabel('posicion (m)')
                        #plt.show()
                        #print (tiempos)

                        fig, axs = plt.subplots(3)
                        fig.suptitle('Graficas fuerza-velocidades Eje Y')
                        axs[0].plot(tiempos,fuerzas, 'tab:blue')
                        axs[0].set_title('incrementos fuerza')
                        axs[0].set(xlabel = 'tiempo(s)', ylabel = 'fuerza (N)')
                        #axs[0].set_ylim([-0.4, 0.0])
                        axs[1].plot(tiempos, vel_mal, 'tab:orange')
                        axs[1].set_title('velocidad control ad.')
                        axs[1].set(xlabel = 'tiempo(s)', ylabel = 'velocidad (m/s)')
                        #axs[1].set_ylim([0, 0.4])
                        axs[2].plot(tiempos, vel_bien, 'tab:green')
                        axs[2].set_title('velocidad final')
                        axs[2].set(xlabel = 'tiempo(s)', ylabel = 'velocidad (m/s)')
                        #axs[2].set_ylim([0.2, 0.6])
                        fig.tight_layout(pad=1.3)
                        #for ax in axs.flat:
                        #   ax.set(xlabel='tiempo (s)', ylabel='posicion (m)')
                        plt.show()
            
            elif tipo_control=='fuerza' and not is_control_on:
                velocidades.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                pub.publish(velocidades)
                print('lo he paraado', is_control_on, robot_ready)
            # print(velocidades)

            # Ajusto el intervalo a la frecuencia
            rate.sleep()
            tiempo2 = rospy.get_time()
            #print("tiempo int:", tiempo2 - tiempo1)



    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()
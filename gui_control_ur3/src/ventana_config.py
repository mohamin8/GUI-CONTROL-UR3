#!/usr/bin/env python3
from operator import imod
from pickle import NONE, TRUE
from socket import MsgFlag
import sys
from ventana_config_ui import *
from ventana_config2_ui import *
from conf_admitancia_ui import *
import time
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool, Int8, Float32
from ur_dashboard_msgs.msg import RobotMode
from PyQt5.QtCore import QTimer,QThread,pyqtSignal,QObject
from PyQt5.QtGui import QMovie
from PyQt5.QtWidgets import QFileDialog,QDesktopWidget, QLabel
import subprocess
import os
import pdb
import numpy as np
#import control
from operator import imod
from pickle import TRUE
from socket import MsgFlag
import sys
from control_ui import *
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool, Int8
from ur_dashboard_msgs.msg import RobotMode
from PyQt5.QtCore import QTimer,QThread,pyqtSignal,QObject
from PyQt5.QtGui import QMovie
from PyQt5.QtWidgets import QFileDialog,QDesktopWidget, QLabel
# import subprocess
# import os
# import pdb
from std_msgs.msg import Int8MultiArray,Int32MultiArray,Float32MultiArray
from functools import partial
from PyQt5.QtGui import QIntValidator,QDoubleValidator,QFont
from PyQt5 import QtGui, QtCore
from qwt import (
    QwtPlot,
    QwtPlotMarker,
    QwtSymbol,
    QwtLegend,
    QwtPlotCurve,
    QwtAbstractScaleDraw,
)
from geometry_msgs.msg import WrenchStamped

import moveit_commander

import math

#leer xml y escribir xml
from xml.dom import minidom
import xml.etree.ElementTree as ET
from ventana_nombre_ejercicio_ui import *
#convertir lista string en lista numérica
import ast
# clase para trabajar con ros en paralelo
class Worker(QObject):
    finished = pyqtSignal()
    progress = pyqtSignal(str)
    mensaje=''
    progress_fuerza = pyqtSignal(WrenchStamped)
    progress_fuerza_vel = pyqtSignal(tuple)

    # def __init__(self, formulario):
    #     super(Worker, self).__init__()
    #     # Store constructor arguments (re-used for processing)
    #     self.formulario=formulario

    # def probar_funcion_plot(self):
    #     # self.formulario.window3.isVisible()
    #     while True:
    #         self.formulario.t.append(self.formulario.t[-1]+1)
    #         self.formulario.fx.append(self.formulario.t[-1]+1)

    #         self.formulario.curvefx.setData(self.formulario.t,self.formulario.fx)

    #         self.formulario.ui3.plot_fx.replot()
#crear otro progress para enviar mensajes en diferentes canales
    def run(self):
        """Long-running task."""
        rospy.Subscriber('/ur_hardware_interface/robot_program_running',Bool, self.callback_robot_state)
        rospy.Subscriber('/ur_hardware_interface/robot_mode',RobotMode, self.callback_robot_mode)
        rospy.Subscriber('fin_calibrado',Bool, self.callback_fin_calibrado)
        #control
        # rospy.Subscriber("/wrench", WrenchStamped, self.callback_fuerza)
        rospy.Subscriber("diferencia_fuerzas", Float32MultiArray, self.callback_fuerza_vel)

        rospy.spin()
    def callback_robot_state(self,data):
        print('robot_state',str(data.data))

        if data.data:
            self.progress.emit('conectado')
        else:
            self.progress.emit('no_conectado')
    def callback_robot_mode(self,data):
        print('robot_mode',str(data.mode))
        self.progress.emit(str(data.mode))

    def callback_fin_calibrado(self,data):
        print('calibrado',str(data.data))
        if data.data:
            self.progress.emit('fin_calibrado')
        else:
            self.progress.emit('no_fin_calibrado')
    #control
    # def callback_fuerza(self,data):
    #     self.formulario.t.append(self.formulario.t[-1]+1)
    #     self.formulario.fx.append(data.wrench.force.z)

    #     # self.formulario.curvefx.setData(self.formulario.t,self.formulario.fx)

    #     # self.formulario.ui3.plot_fx.replot()
    #     # self.progress_fuerza.emit(data)
    def callback_fuerza_vel(self,data):

        self.progress_fuerza_vel.emit(data.data)

class Miformulario(QtWidgets.QMainWindow):
    def __init__(self,parent=None):
        QtWidgets.QWidget.__init__(self,parent)
        self.ui=Ui_MainWindow()
        self.ui.setupUi(self)
        self.setWindowIcon(QtGui.QIcon('/home/amine/Master/TFM/catkin_ws/src/gui_control_ur3/src/images/Universal_robots_logo.png'))
        # self.ui.iniciar_driver.setIcon(QtGui.QIcon('/home/amine/Master/TFM/catkin_ws/src/gui_control_ur3/src/images/477200.png'))

        # ventana 2
        self.window2=QtWidgets.QMainWindow()
        self.ui2=Ui_MainWindow2()
        self.ui2.setupUi(self.window2)
        #ventana de control
        self.window3=QtWidgets.QMainWindow()
        self.ui3=Ui_MainWindow_control()
        self.ui3.setupUi(self.window3)
        self.ui3.actualizar_x.setIcon(QtGui.QIcon('/home/amine/Master/TFM/catkin_ws/src/gui_control_ur3/src/images/477200.png'))
        self.ui3.actualizar_y.setIcon(QtGui.QIcon('/home/amine/Master/TFM/catkin_ws/src/gui_control_ur3/src/images/477200.png'))
        self.ui3.actualizar_z.setIcon(QtGui.QIcon('/home/amine/Master/TFM/catkin_ws/src/gui_control_ur3/src/images/477200.png'))
        self.ui3.plotx_ampliar.setIcon(QtGui.QIcon('/home/amine/Master/TFM/catkin_ws/src/gui_control_ur3/src/images/ampliar.png'))
        self.ui3.ploty_ampliar.setIcon(QtGui.QIcon('/home/amine/Master/TFM/catkin_ws/src/gui_control_ur3/src/images/ampliar.png'))
        self.ui3.plotz_ampliar.setIcon(QtGui.QIcon('/home/amine/Master/TFM/catkin_ws/src/gui_control_ur3/src/images/ampliar.png'))
        self.ui3.plotx_reducir.setIcon(QtGui.QIcon('/home/amine/Master/TFM/catkin_ws/src/gui_control_ur3/src/images/reducir.png'))
        self.ui3.ploty_reducir.setIcon(QtGui.QIcon('/home/amine/Master/TFM/catkin_ws/src/gui_control_ur3/src/images/reducir.png'))
        self.ui3.plotz_reducir.setIcon(QtGui.QIcon('/home/amine/Master/TFM/catkin_ws/src/gui_control_ur3/src/images/reducir.png'))

        #ventana de  para guardar nombre de ejercicio
        self.window4=QtWidgets.QMainWindow()
        self.ui4=Ui_ventana_nombre_ejercicio()
        self.ui4.setupUi(self.window4)
        #ventana de configuración del control de admitancia
        self.window5=QtWidgets.QMainWindow()
        self.ui5=Ui_Conf_admitancia()
        self.ui5.setupUi(self.window5)
        #poner ventana 1 en el centro
        self.centerPoint = QDesktopWidget().availableGeometry().center()
        qtRectangle = self.frameGeometry()
        centerPoint = QDesktopWidget().availableGeometry().center()
        qtRectangle.moveCenter(centerPoint)
        self.move(qtRectangle.topLeft())
        #poner ventana 2 en el centro
        self.window2.centerPoint = QDesktopWidget().availableGeometry().center()
        qtRectangle = self.window2.frameGeometry()
        centerPoint = QDesktopWidget().availableGeometry().center()
        qtRectangle.moveCenter(centerPoint)
        self.window2.move(qtRectangle.topLeft())        
        #poner ventana 5 en el centro
        self.window5.centerPoint = QDesktopWidget().availableGeometry().center()
        qtRectangle = self.window5.frameGeometry()
        centerPoint = QDesktopWidget().availableGeometry().center()
        qtRectangle.moveCenter(centerPoint)
        self.window5.move(qtRectangle.topLeft())   
        # poner ventana de control en la izquierda
        topLeftPoint = QtWidgets.QApplication.desktop().availableGeometry().topLeft()
        self.window3.move(topLeftPoint)     
                
        #conexiones..................................................................
        self.ui.seleccionar_archivo.clicked.connect(self.buscar_archivo)
        self.ui.Calibrar.clicked.connect(self.calibrar_robot)
        self.ui.iniciar_driver.clicked.connect(self.iniciar_driver)
        self.ui.iniciar_driver.clicked.connect(self.abrir_dialogo)
        self.ui2.ok_boton.clicked.connect(self.cerrar_dialogo)

        self.ui.archivo_calibracion.setText("${HOME}/my_robot_calibration.yaml")
        # leer ip constantemente
        self.timer = QTimer()
        self.timer.timeout.connect(self.ips_recibidas)
        self.timer.start(1000)
        #abrir ventana de carga durante 5 seg
        self.timer_carga = QTimer()
        self.timer_carga.timeout.connect(self.cerrar_dialogo)


        # obtener directorio actual para cargar archivos
        self.file_path = os.path.realpath(__file__)
        self.file_path=self.file_path.split('/')
        self.file_path.remove(self.file_path[0])
        self.file_path.remove(self.file_path[-1])
        self.mi_path=''
        for i in range(len(self.file_path)):
                print(self.file_path[i])
                self.mi_path+='/'+self.file_path[i]
        print(self.mi_path)

# ventana de control........................................................................................................
        self.ui3.conectar_robot.setDisabled(True)
        self.ui3.visualizar_ejercicio.setDisabled(True)

        #valores iniciales del control de admitancia
        self.ui5.constante_c.setValue(0.3)
        self.ui5.umbral_fx.setValue(10)
        self.ui5.umbral_fy.setValue(10)
        self.ui5.umbral_fz.setValue(50)
        self.ui5.lim_velocidad.setValue(0.5)


       #conexiones control admitancia
        self.ui3.conectar_robot.clicked.connect(self.abrir_dialogo_config)
        self.ui3.configuracion.clicked.connect(self.abrir_configuracion)
        self.ui5.aceptar.accepted.connect(self.enviar_configuracion)
        self.ui5.aceptar.rejected.connect(self.cerrar_dialogo)
        self.ui3.play_stop.clicked.connect(self.iniciar_parar_control_admitancia)
        #plotear datos
        self.curvefx = QwtPlotCurve("Force applied in axis x")
        self.curvefx.attach(self.ui3.plot_fx)
        self.curvevx = QwtPlotCurve("Velocity in axis x")
        self.curvevx.attach(self.ui3.plot_vx)
        self.curvefy = QwtPlotCurve("Force applied in axis y")
        self.curvefy.attach(self.ui3.plot_fy)
        self.curvevy = QwtPlotCurve("Velocity in axis y")
        self.curvevy.attach(self.ui3.plot_vy)
        self.curvefz = QwtPlotCurve("Force applied in axis z")
        self.curvefz.attach(self.ui3.plot_fz)
        self.curvevz = QwtPlotCurve("Velocity in axis z")
        self.curvevz.attach(self.ui3.plot_vz)
        self.t=list();self.t.append(0)
        self.fx=list();self.fx.append(0)
        self.fy=list();self.fy.append(0)
        self.fz=list();self.fz.append(0)
        self.vx=list();self.vx.append(0)
        self.vy=list();self.vy.append(0)
        self.vz=list();self.vz.append(0)
        self.tx=list();self.tx.append(0)
        self.ty=list();self.ty.append(0)
        self.tz=list();self.tz.append(0)
        #que ultimos elementos mostrar en plot
        self.desplazamiento_en_plotx=1
        self.desplazamiento_en_ploty=1
        self.desplazamiento_en_plotz=1
        self.num_elementos_plotx=100
        self.num_elementos_ploty=100
        self.num_elementos_plotz=100

        #desplazar por el plot
        # # desplazamiento continuo
        # self.timer_desplazamiento = QTimer()
        # self.timer_desplazamiento.timeout.connect(partial(self.desplazar_en_plot,'Px+'))
        # self.timer_desplazamiento.start(100)
        self.ui3.plot_x_mas.pressed.connect(partial(self.desplazar_en_plot,'Px+'))
        self.ui3.plot_x_menos.pressed.connect(partial(self.desplazar_en_plot,'Px-'))
        self.ui3.plot_y_mas.pressed.connect(partial(self.desplazar_en_plot,'Py+'))
        self.ui3.plot_y_menos.pressed.connect(partial(self.desplazar_en_plot,'Py-'))
        self.ui3.plot_z_mas.pressed.connect(partial(self.desplazar_en_plot,'Pz+'))
        self.ui3.plot_z_menos.pressed.connect(partial(self.desplazar_en_plot,'Pz-'))
        self.ui3.plot_x_mas.released.connect(partial(self.desplazar_en_plot,'Px+'))
        self.ui3.plot_x_menos.released.connect(partial(self.desplazar_en_plot,'Px-'))
        self.ui3.plot_y_mas.released.connect(partial(self.desplazar_en_plot,'Py+'))
        self.ui3.plot_y_menos.released.connect(partial(self.desplazar_en_plot,'Py-'))
        self.ui3.plot_z_mas.released.connect(partial(self.desplazar_en_plot,'Pz+'))
        self.ui3.plot_z_menos.released.connect(partial(self.desplazar_en_plot,'Pz-'))
        #ampliar por el plot

        self.eje_ampliando_reduciendo='Px+'
        self.timer_ampliar_reducir=QTimer()
        self.timer_ampliar_reducir.timeout.connect(self.ampliar_plot)
        self.ui3.plotx_ampliar.pressed.connect(partial(self.iniciar_ampliar_reducir_plot,'Px+'))
        self.ui3.plotx_reducir.pressed.connect(partial(self.iniciar_ampliar_reducir_plot,'Px-'))
        self.ui3.plotx_reducir.released.connect(self.parar_ampliar_reducir_plpot)
        self.ui3.plotx_ampliar.released.connect(self.parar_ampliar_reducir_plpot)
        self.ui3.ploty_ampliar.pressed.connect(partial(self.iniciar_ampliar_reducir_plot,'Py+'))
        self.ui3.ploty_reducir.pressed.connect(partial(self.iniciar_ampliar_reducir_plot,'Py-'))
        self.ui3.ploty_reducir.released.connect(self.parar_ampliar_reducir_plpot)
        self.ui3.ploty_ampliar.released.connect(self.parar_ampliar_reducir_plpot)
        self.ui3.plotz_ampliar.pressed.connect(partial(self.iniciar_ampliar_reducir_plot,'Pz+'))
        self.ui3.plotz_reducir.pressed.connect(partial(self.iniciar_ampliar_reducir_plot,'Pz-'))
        self.ui3.plotz_reducir.released.connect(self.parar_ampliar_reducir_plpot)
        self.ui3.plotz_ampliar.released.connect(self.parar_ampliar_reducir_plpot)
        # self.ui3.ploty_ampliar.pressed.connect(partial(self.ampliar_plot,'Py+'))
        # self.ui3.ploty_reducir.pressed.connect(partial(self.ampliar_plot,'Py-'))
        # self.ui3.plotz_ampliar.pressed.connect(partial(self.ampliar_plot,'Pz+'))
        # self.ui3.plotz_reducir.pressed.connect(partial(self.ampliar_plot,'Pz-'))
        #volver a mostar gráfica en tiempo real
        self.ui3.actualizar_x.clicked.connect(partial(self.plot_tiempo_real,'x'))
        self.ui3.actualizar_y.clicked.connect(partial(self.plot_tiempo_real,'y'))
        self.ui3.actualizar_z.clicked.connect(partial(self.plot_tiempo_real,'z'))

        #guardar mediciones
        self.ui3.guardar_mediciones.clicked.connect(self.guardar_mediciones)
        #cargar mediciones
        self.ui3.cargar_mediciones.clicked.connect(self.cargar_mediciones)
        #plotear valores en directo
        self.tiempo_realx=True
        self.tiempo_realy=True
        self.tiempo_realz=True

        # sección control de posición
        #crear variable cuando se haya cargado todo
        self.arm_group =NONE

        #conexiones control posición..................................................................
        self.ui3.home.clicked.connect(self.ir_a_home)
        #movimiento continuo derecha
        self.ui3.q1_derecha.pressed.connect(partial(self.boton_giro_presionado,'q1+'))
        self.ui3.q1_derecha.released.connect(self.boton_giro_soltado)
        self.ui3.q2_derecha.pressed.connect(partial(self.boton_giro_presionado,'q2+'))
        self.ui3.q2_derecha.released.connect(self.boton_giro_soltado)
        self.ui3.q3_derecha.pressed.connect(partial(self.boton_giro_presionado,'q3+'))
        self.ui3.q3_derecha.released.connect(self.boton_giro_soltado)
        self.ui3.q4_derecha.pressed.connect(partial(self.boton_giro_presionado,'q4+'))
        self.ui3.q4_derecha.released.connect(self.boton_giro_soltado)
        self.ui3.q5_derecha.pressed.connect(partial(self.boton_giro_presionado,'q5+'))
        self.ui3.q5_derecha.released.connect(self.boton_giro_soltado)
        self.ui3.q6_derecha.pressed.connect(partial(self.boton_giro_presionado,'q6+'))
        self.ui3.q6_derecha.released.connect(self.boton_giro_soltado)
        #movimiento continuo izquierda
        self.ui3.q1_izquierda.pressed.connect(partial(self.boton_giro_presionado,'q1-'))
        self.ui3.q1_izquierda.released.connect(self.boton_giro_soltado)
        self.ui3.q2_izquierda.pressed.connect(partial(self.boton_giro_presionado,'q2-'))
        self.ui3.q2_izquierda.released.connect(self.boton_giro_soltado)
        self.ui3.q3_izquierda.pressed.connect(partial(self.boton_giro_presionado,'q3-'))
        self.ui3.q3_izquierda.released.connect(self.boton_giro_soltado)
        self.ui3.q4_izquierda.pressed.connect(partial(self.boton_giro_presionado,'q4-'))
        self.ui3.q4_izquierda.released.connect(self.boton_giro_soltado)
        self.ui3.q5_izquierda.pressed.connect(partial(self.boton_giro_presionado,'q5-'))
        self.ui3.q5_izquierda.released.connect(self.boton_giro_soltado)
        self.ui3.q6_izquierda.pressed.connect(partial(self.boton_giro_presionado,'q6-'))
        self.ui3.q6_izquierda.released.connect(self.boton_giro_soltado)
        #actualizar valores articulares en pantalla
        self.ui3.value_q1.setValue(self.ui3.slider_q1.value()/10)
        self.ui3.value_q2.setValue(self.ui3.slider_q2.value()/10)
        self.ui3.value_q3.setValue(self.ui3.slider_q3.value()/10)
        self.ui3.value_q4.setValue(self.ui3.slider_q4.value()/10)
        self.ui3.value_q5.setValue(self.ui3.slider_q5.value()/10)
        self.ui3.value_q6.setValue(self.ui3.slider_q6.value()/10)
        self.ui3.slider_q1.valueChanged.connect(self.joints_slider_changed)
        self.ui3.slider_q2.valueChanged.connect(self.joints_slider_changed)
        self.ui3.slider_q3.valueChanged.connect(self.joints_slider_changed)
        self.ui3.slider_q4.valueChanged.connect(self.joints_slider_changed)
        self.ui3.slider_q5.valueChanged.connect(self.joints_slider_changed)
        self.ui3.slider_q6.valueChanged.connect(self.joints_slider_changed)
        self.ui3.value_q1.valueChanged.connect(self.joints_value_changed)
        self.ui3.value_q2.valueChanged.connect(self.joints_value_changed)
        self.ui3.value_q3.valueChanged.connect(self.joints_value_changed)
        self.ui3.value_q4.valueChanged.connect(self.joints_value_changed)
        self.ui3.value_q5.valueChanged.connect(self.joints_value_changed)
        self.ui3.value_q6.valueChanged.connect(self.joints_value_changed)
        self.q1= self.ui3.value_q1.value()
        self.q2= self.ui3.value_q2.value()
        self.q3= self.ui3.value_q3.value()
        self.q4= self.ui3.value_q4.value()
        self.q5= self.ui3.value_q5.value()
        self.q6= self.ui3.value_q6.value()

        #estado de slider pulsado para solo
        #enviar valores articulares y saber si se esta moviendo el slider o no
        self.ui3.slider_q1.sliderPressed.connect(self.slider_pressed)
        self.ui3.slider_q2.sliderPressed.connect(self.slider_pressed)
        self.ui3.slider_q3.sliderPressed.connect(self.slider_pressed)
        self.ui3.slider_q4.sliderPressed.connect(self.slider_pressed)
        self.ui3.slider_q5.sliderPressed.connect(self.slider_pressed)
        self.ui3.slider_q6.sliderPressed.connect(self.slider_pressed)
        self.ui3.slider_q1.sliderReleased.connect(self.send_joints)
        self.ui3.slider_q2.sliderReleased.connect(self.send_joints)
        self.ui3.slider_q3.sliderReleased.connect(self.send_joints)
        self.ui3.slider_q4.sliderReleased.connect(self.send_joints)
        self.ui3.slider_q5.sliderReleased.connect(self.send_joints)
        self.ui3.slider_q6.sliderReleased.connect(self.send_joints)      
        self.i=0


        #movimiento continuo cartesiano
        self.ui3.x_neg.pressed.connect(partial(self.boton_giro_presionado,'x-'))
        self.ui3.x_neg.released.connect(partial(self.boton_giro_soltado))
        self.ui3.x_pos.pressed.connect(partial(self.boton_giro_presionado,'x+'))
        self.ui3.x_pos.released.connect(partial(self.boton_giro_soltado))
        self.ui3.y_neg.pressed.connect(partial(self.boton_giro_presionado,'y-'))
        self.ui3.y_neg.released.connect(partial(self.boton_giro_soltado))
        self.ui3.y_pos.pressed.connect(partial(self.boton_giro_presionado,'y+'))
        self.ui3.y_pos.released.connect(partial(self.boton_giro_soltado))
        self.ui3.z_neg.pressed.connect(partial(self.boton_giro_presionado,'z-'))
        self.ui3.z_neg.released.connect(partial(self.boton_giro_soltado))
        self.ui3.z_pos.pressed.connect(partial(self.boton_giro_presionado,'z+'))
        self.ui3.z_pos.released.connect(partial(self.boton_giro_soltado))
        #variable para ver si los sliders están pulsados para solo enviar ordenes cuando se modifique el indicador o se suelte el slider
        self.slider_pressed_bool=False
        self.mov_direccion_bool=False
        self.lista_ejercicios=list()

        self.ui3.nuevo_punto.clicked.connect(self.nueva_posicion)
        self.ui3.reiniciar_seleccion.clicked.connect(self.reiniciar_posiciones)
        self.ui3.guardar_ejercicio.clicked.connect(self.abrir_ventana_guardar_ejercicio)
        # cargar los ejercicios del xml
        self.cargar_ejercicios_xml()
        # mostrar los ejercicios en función del tipo seleccionado
        self.ui3.seleccion_ejercicio.currentTextChanged.connect(self.item_cambiado)
        self.ui3.boton_ej_circular.clicked.connect(partial(self.tipo_ej_seleccionado,False))
        self.ui3.boton_ej_lineal.clicked.connect(partial(self.tipo_ej_seleccionado,True))
        self.ui3.ejecutar_ejercicio.clicked.connect(self.ejecutar_ejercicio)
        self.ui3.parar_ejercicio.clicked.connect(self.parar_ejercicio)
        #saber que tipo de ejercicio tenemos seleccionado
        self.ejercicio_lineal=True
        self.ui3.boton_ej_lineal.setChecked(True)
        # self.ui3.tipo_ejercicio_guardar.addItems(['Lineal','Circular'])
        self.lista_puntos_nuevo_ejercicio=list()
        
        # cerrar ventana de guardado de ejercicio
        self.ui4.OK.clicked.connect(self.cerrar_ventana_guardar_ejercicio)
        # QtCore.Qt.Key_Enter
        # hilo de ros
        #que control tenemos
        self.tipo_control='posicion'
        self.ui3.tabWidget.currentChanged.connect(self.tipo_de_control)

        #leer valores articulares constantemente mientras se tenga pulsado los botones de giro
        self.timer_valores_art = QTimer()
        self.timer_valores_art.timeout.connect(self.obtener_valores_articulares)

        #añadir tooltips(mensajes que se muestran al poner el raton encima de botón)
        self.ui3.nuevo_punto.setToolTip("Añadir nueva posición al ejercicio")
        self.ui3.reiniciar_seleccion.setToolTip("Reiniciar selección de posiciones")
        self.ui3.fuerza_primera_x.setToolTip("Fuerza de comparación en X")
        self.ui3.fuerza_primera_y.setToolTip("Fuerza de comparación en Y")
        self.ui3.fuerza_primera_z.setToolTip("Fuerza de comparación en Z")
        self.ui3.configuracion.setToolTip('Configurar control de fuerza')
        self.ros_listener()




        # iniciar  nuevo hilo para estar suscrito a un topic en paralelo
    def ros_listener(self):
        # Step 2: Create a QThread object
        self.thread = QThread()
        # Step 3: Create a worker object
        self.worker = Worker()
        # Step 4: Move worker to the thread
        self.worker.moveToThread(self.thread)
        # Step 5: Connect signals and slots
        self.thread.started.connect(self.worker.run)
        # self.thread.started.connect(self.worker.probar_funcion_plot)
        self.worker.finished.connect(self.thread.quit)
        self.worker.finished.connect(self.worker.deleteLater)
        self.thread.finished.connect(self.thread.deleteLater)
        self.worker.progress.connect(self.show_ros_info)
        
        #fuerza del extremo
        # self.worker.progress_fuerza.connect(self.plot)
        self.worker.progress_fuerza_vel.connect(self.plot)

        # Step 6: Start the thread
        self.thread.start()
        # mostrar mensaje de topic
    def show_ros_info(self,str):
        print(str)

        if str=='1' or str=='2' or str=='3' or str=='4':
            self.ui2.Estado.setText('Enciende el Robot')
        elif str=='5':
            self.ui2.Estado.setText('Inicia el Robot')
        if str=='7':
            self.ui2.Estado.setText('Ejecuta programa en Robot')
        elif str=='conectado':
            self.ui2.Estado.setText('Robot conectado')
        # iniciar animaciones
            self.ui2.movie = QMovie(self.mi_path+"/images/check_green_lit.gif")
            self.ui2.gif_label.setMovie(self.ui2.movie)
            self.ui2.movie.frameChanged.connect(self.fin_animacion)
            self.ui2.movie.start()
            self.ui2.ok_boton.show()
        elif str=='no_conectado':
            self.ui2.Estado.setText('Cargando configuración')
        elif str=='fin_calibrado':
            self.ui2.Estado.setText('Fin de calibración')
            self.ui2.movie = QMovie(self.mi_path+"/images/check_green_lit.gif")
            self.ui2.gif_label.setMovie(self.ui2.movie)
            self.ui2.movie.frameChanged.connect(self.fin_animacion)
            self.ui2.movie.start()
            self.ui2.ok_boton.show()

        else:
            self.ui2.Estado.setText('Calibrando')

              



#     def callback_robot_state(self,data):
#         self.ui.archivo_calibracion.setText(str(data.data))

    def buscar_archivo(self):
        fname=QFileDialog.getOpenFileName(self,'Seleccionar archivo de calibración',self.mi_path,'*.yaml')
        
        # self.actiondees.triggered.connect(lambda: self.pulsado('le he dado a dees'))
        # texto=self.ui.textEdit.toPlainText()
        # pdb.set_trace()
        self.ui.archivo_calibracion.setText(fname[0])
  
        # print('buscar_archivo')

    def calibrar_robot(self):
        dir=QFileDialog.getExistingDirectory(self,'Seleccionar directorio',self.mi_path)
        # print(dir)
        ip=self.ui.robot_ip.toPlainText()
        comando='calibrar'
        # print(ip+' '+dir)
        pub_calibrar.publish(ip+' '+dir)
        self.abrir_dialogo()
        # comando='roslaunch ur_calibration calibration_correction.launch robot_ip:='+ip+' kinematics_config:="${HOME}/my_robot_calibration.yaml"'
    def ips_recibidas(self):
        # print('leyendo ip')
        # print('ip recibidooo')
        result = subprocess.run(['ifconfig'], stdout=subprocess.PIPE)
        #decode resultado porque esta en bytes

        texto=result.stdout.decode('utf-8')
        texto_div=texto.split('\n\n')

        #pdb.set_trace()
        #ip ethernet
        if texto_div[0].find('inet')==-1:
                ip_ethernet='No hay conexion ethernet'
                self.ui.my_etherner_ip.setStyleSheet("color: red;")

                #     print(ip_ethernet)
        else:
                ip_ethernet=texto_div[0].split('inet ')[1]
                ip_ethernet=ip_ethernet.split('  netmask')[0]
                self.ui.my_etherner_ip.setStyleSheet("color: green;")

                #     print('ip ethernet:',ip_ethernet)

        self.ui.my_etherner_ip.setText(ip_ethernet)

        #ip wlan
        if texto_div[2].find('inet')==-1:
                ip_wlan='no hay conexion wlan'
                self.ui.my_wlan_ip.setStyleSheet("color: red;")

                #     print('no hay conexion wlan')
        else:
                ip_wlan=texto_div[2].split('inet ')[1]
                ip_wlan=ip_wlan.split('  netmask')[0]
                self.ui.my_wlan_ip.setStyleSheet("color: green;")

                #     print('ip  wlan:',ip_wlan)

        self.ui.my_wlan_ip.setText(ip_wlan)
    def iniciar_driver(self):
        self.timer.stop()

        # ping_ip='ping '+ip
        
        # resultado_ping_ip = subprocess.run(['ping','192.168.2.129'], stdout=subprocess.PIPE)

        # #decode resultado porque esta en bytes
        # print(resultado_ping_ip)
        # texto_resultado=resultado_ping_ip.stdout.decode('utf-8')
        # texto_resultado=texto_resultado[::-1]
        # texto_resultado=texto_resultado.split(' ')[0]
        # texto_resultado=texto_resultado[::-1]

        # if texto_resultado=='Unreachable':
        #     print('ip no conectado')
        # else:
        #     print('ip conectado')
        #si esta seleccionado conectar robot iniciar driver 
        if self.ui.conectar_robot.isChecked():
            # self.arm_group = moveit_commander.MoveGroupCommander("manipulator")
            self.ui3.conectar_robot.setChecked(True)
            ip=self.ui.robot_ip.toPlainText()

            cal_archivo=self.ui.archivo_calibracion.toPlainText()
            archivo_sep=cal_archivo[::-1]
            archivo_sep=archivo_sep.split('/')[0]
            archivo_sep=archivo_sep[::-1]
            # print(archivo_sep)
            if archivo_sep!='my_robot_calibration.yaml':
                    cal_archivo=QFileDialog.getOpenFileName(self,'Seleccionar archivo de calibración','/home/amine','*.yaml')[0]
                    self.ui.archivo_calibracion.setText(cal_archivo)

            pub.publish(ip+' '+cal_archivo)
            # print(ip+' '+cal_archivo)
            #no ejecutar move_group con fake execution
            pub_iniciar_move_group.publish(False)
        else:
            pub.publish('false')
            # print('false')
            pub_iniciar_move_group.publish(True)

            self.abrir_dialogo()


    def abrir_dialogo(self):
        self.window2.show()
        self.ui2.ok_boton.hide()

        #mostrar gif
        self.ui2.movie = QMovie(self.mi_path+"/images/carga_lit.gif")
        self.ui2.gif_label.setMovie(self.ui2.movie)
        self.ui2.movie.start()
        if not self.ui.conectar_robot.isChecked():
            self.timer_carga.start(5000)

        # else:
        #     # self.window2.show()
        #     # self.ui2.ok_boton.hide()
        #     # time.sleep(5)
        #     self.window3.show()
        #     # self.window2.hide()
        #     self.ui2.ok_boton.hide()
        #      #mostrar gif
        #     self.ui2.movie = QMovie(self.mi_path+"/images/carga_lit.gif")
        #     self.ui2.gif_label.setMovie(self.ui2.movie)
        #     self.ui2.movie.start()
            # time.sleep(5)



    def cerrar_dialogo(self):
        self.window2.close()
        self.window3.show()
        self.hide()
        self.window5.close()
        self.timer_carga.stop()
        if self.arm_group==NONE:
            self.arm_group = moveit_commander.MoveGroupCommander("manipulator")
            self.obtener_valores_articulares()





    def fin_animacion(self):
        # print(self.ui2.movie.currentFrameNumber())
        if self.ui2.movie.currentFrameNumber()==30:
        # if self.ui2.movie.frameCount()==31:
            self.ui2.movie.stop()

#.........................control ........................................

    def abrir_dialogo_config(self):
        if self.ui3.conectar_robot.isChecked():
            self.show()
    def enviar_configuracion(self):
        c=self.ui5.constante_c.value()
        umbralx=self.ui5.umbral_fx.value()
        umbraly=self.ui5.umbral_fy.value()
        umbralz=self.ui5.umbral_fz.value()
        lim_vel=self.ui5.lim_velocidad.value()
        conf=[c,umbralx,umbraly,umbralz,lim_vel]
        conf_env=Float32MultiArray(data=conf)
        pub_configuracion.publish(conf_env)
        self.window5.close()

# .......................control admitancia....................................
    def abrir_configuracion(self):
        self.window5.show()

    def guardar_mediciones(self):
        dir=QFileDialog.getSaveFileName(self,'Introduce nombre del archivo',self.mi_path)
        dir=str(dir[0])
        dir= dir+'.txt'
        # print(dir)
        archivo=open(dir,'w')
        archivo.write('t'+'       '+'fx'+'       '+'vx'+'       '+'fy'+'       '+'vy'+'       '+'fz'+'       '+'vz'+'\n')
        # print(len(self.tx),' ',len(self.fx),' ',len(self.vx),' ',len(self.fy),' ',len(self.vy),' ',len(self.fz),' ',len(self.vz),' ',)
        for i in range(len(self.fx)):
            archivo.write(str(self.tx[i])+'       '+str(self.fx[i])+'       '+str(self.vx[i])+'       '+str(self.fy[i])+'       '+str(self.vy[i])+'       '+str(self.fz[i])+'       '+str(self.vz[i])+'\n')
            
        archivo.close()
    def cargar_mediciones(self):
        archivo=QFileDialog.getOpenFileName(self,'Seleccionar archivo de calibración',self.mi_path,'*.txt')
        # print(archivo[0])
        nombre=archivo[0].split('/')[-1]
        self.ui3.nombre_ejercicio.setText(nombre)
        tx,fx,vx,fy,vy,fz,vz=np.loadtxt(archivo[0],skiprows=1,usecols=[0,1,2,3,4,5,6],unpack=True)
        self.tx=tx.tolist()
        self.ty=tx.tolist()
        self.tz=tx.tolist()
        self.fx=fx.tolist()
        self.fy=fy.tolist()
        self.fz=fz.tolist()
        self.vx=vx.tolist()
        self.vy=vy.tolist()
        self.vz=vz.tolist()
        # n=10
        # self.fz=self.fy.copy()
        # print(type(self.fz))
        # for i in range(len(self.fx)):
        #     if i>n and i<len(self.fx)-n:
        #         sum=0
        #         for j in range(5):
        #             sum+=self.fy[int(i-j)]
        #         self.fy[i]=sum/n

        # mostrar toda la grafica en el plot
        self.num_elementos_plotx=len(self.fx)
        self.num_elementos_ploty=len(self.fy)
        self.num_elementos_plotz=len(self.fz)
        self.desplazamiento_en_plotx=1
        self.desplazamiento_en_ploty=1
        self.desplazamiento_en_plotz=1

        # self.curvefx.setData(self.tx[-self.num_elementos_plotx-self.desplazamiento_en_plotx:-self.desplazamiento_en_plotx],self.fx[-self.num_elementos_plotx-self.desplazamiento_en_plotx:-self.desplazamiento_en_plotx])
        # self.ui3.plot_fx.replot()
        # print('a')
        self._replotear('x')
        self._replotear('y')
        self._replotear('z')
        # print(self.tx,'................',self.fx)


    def iniciar_parar_control_admitancia(self):
        if self.ui3.play_stop.text()=='Iniciar':
            pub_admitancia_on.publish(True)
            self.ui3.play_stop.setText('Parar')
            self.tx.clear()
            self.ty.clear()
            self.tz.clear()
            self.fx.clear()
            self.fy.clear()
            self.fz.clear()
            self.vx.clear()
            self.vy.clear()
            self.vz.clear()

        else:
            pub_admitancia_on.publish(False)
            self.ui3.play_stop.setText('Iniciar')


    def desplazar_en_plot(self,value):
        if value=='Px+':
            if self.desplazamiento_en_plotx!=1:
                self.desplazamiento_en_plotx-=1
                self._replotear('x')
        if value=='Px-':
            self.tiempo_realx=False
            if self.desplazamiento_en_plotx!=len(self.fx):
                self.desplazamiento_en_plotx+=1
                self._replotear('x')
        
        if value=='Py+':
            if self.desplazamiento_en_ploty!=1:
                self.desplazamiento_en_ploty-=1
                self._replotear('y')

        if value=='Py-':
            self.tiempo_realy=False
            if self.desplazamiento_en_ploty!=len(self.fy):
                self.desplazamiento_en_ploty+=1  
                self._replotear('y')

        if value=='Pz+':
            if self.desplazamiento_en_plotz!=1:
                self.desplazamiento_en_plotz-=1
                self._replotear('z')
        
        if value=='Pz-':
            self.tiempo_realz=False
            if self.desplazamiento_en_plotz!=len(self.fz):
                self.desplazamiento_en_plotz+=1
                self._replotear('z')
    def iniciar_ampliar_reducir_plot(self,value):
        self.eje_ampliando_reduciendo=value
        # print(self.eje_ampliando_reduciendo)
        self.timer_ampliar_reducir.start(300)

    def parar_ampliar_reducir_plpot(self):
        self.timer_ampliar_reducir.stop()

    def ampliar_plot(self):
        value=self.eje_ampliando_reduciendo
        # print(value)

        if value=='Px+':
            if self.num_elementos_plotx>10:
                self.num_elementos_plotx-=10
                self._replotear('x')
        if value=='Px-':
            self.num_elementos_plotx+=10
            self._replotear('x')
        if value=='Py+':
            if self.num_elementos_ploty>10:
                self.num_elementos_ploty-=10
                self._replotear('y')
        if value=='Py-':
            self.num_elementos_ploty+=10
            self._replotear('y')       
        if value=='Pz+':
            if self.num_elementos_plotz>10:
                self.num_elementos_plotz-=10
                self._replotear('z')
        if value=='Pz-':
            self.num_elementos_plotz+=10
            self._replotear('z')
        # print(self.num_elementos_plotx,' ',self.num_elementos_ploty,' ',self.num_elementos_plotz)
    # def iniciar_desplazamiento(self):

    # def parar_desplazamiento(self):
    #     self.timer_desplazamiento.stop()
    def plot_tiempo_real(self,value):
        if value=='x':
            self.tiempo_realx=True
        if value=='y':
            self.tiempo_realy=True
        if value=='z':
            self.tiempo_realz=True

    def _replotear(self,eje):
        if eje=='x':
            if len(self.fx)>=self.num_elementos_plotx:
                self.curvefx.setData(self.tx[-self.num_elementos_plotx-self.desplazamiento_en_plotx:-self.desplazamiento_en_plotx],self.fx[-self.num_elementos_plotx-self.desplazamiento_en_plotx:-self.desplazamiento_en_plotx])

            self.ui3.plot_fx.replot()            
            if len(self.vx)>=self.num_elementos_plotx:
                self.curvevx.setData(self.tx[-self.num_elementos_plotx-self.desplazamiento_en_plotx:-self.desplazamiento_en_plotx],self.vx[-self.num_elementos_plotx-self.desplazamiento_en_plotx:-self.desplazamiento_en_plotx])
            self.ui3.plot_vx.replot()
        if eje=='y':
            if len(self.fy)>=self.num_elementos_ploty:
                self.curvefy.setData(self.ty[-self.num_elementos_ploty-self.desplazamiento_en_ploty:-self.desplazamiento_en_ploty],self.fy[-self.num_elementos_ploty-self.desplazamiento_en_ploty:-self.desplazamiento_en_ploty])

            self.ui3.plot_fy.replot()            
            if len(self.vy)>=self.num_elementos_ploty:
                self.curvevy.setData(self.ty[-self.num_elementos_ploty-self.desplazamiento_en_ploty:-self.desplazamiento_en_ploty],self.vy[-self.num_elementos_ploty-self.desplazamiento_en_ploty:-self.desplazamiento_en_ploty])
            self.ui3.plot_vy.replot()
        if eje=='z':
            if len(self.fz)>=self.num_elementos_plotz:
                self.curvefz.setData(self.tz[-self.num_elementos_plotz-self.desplazamiento_en_plotz:-self.desplazamiento_en_plotz],self.fz[-self.num_elementos_plotz-self.desplazamiento_en_plotz:-self.desplazamiento_en_plotz])

            self.ui3.plot_fz.replot()            
            if len(self.vz)>=self.num_elementos_plotz:
                self.curvevz.setData(self.tz[-self.num_elementos_plotz-self.desplazamiento_en_plotz:-self.desplazamiento_en_plotz],self.vz[-self.num_elementos_plotz-self.desplazamiento_en_plotz:-self.desplazamiento_en_plotz])
            self.ui3.plot_vz.replot()
    def plot(self,valores):

        # print(self.tipo_control)

        # mostrsar con menos frecuencia
        #hay que permitir introducir la frecuencia de muestreo
        # div=valores[-1]*1000/2
        # # print(div)
        # if int(div)==div:
        #     par=True
        # else: 
        #     par=False
        par=True
        # print(int(div)==div)

        self.ui3.fuerza_primera_x.setValue(valores[9])
        self.ui3.fuerza_primera_y.setValue(valores[10])
        self.ui3.fuerza_primera_z.setValue(valores[11])

        if self.tipo_control=='fuerza' and par and self.ui3.play_stop.text()=='Parar':
            self.fx.append(valores[0])
            self.vx.append(valores[3])
            self.tx.append(valores[6])
            self.fy.append(valores[1])
            self.vy.append(valores[4])
            self.ty.append(valores[7])
            self.fz.append(valores[2])
            self.vz.append(valores[5])
            self.tz.append(valores[8])
            # print(self.desplazamiento_en_plotx)
            # if len(self.fx)>=200:
            #     self.curvefx.setData(self.t[-200:],self.fx[-200:])
            # else:
            #     self.curvefx.setData(self.t,self.fx)
            # self.ui3.plot_fx.replot()
            # self.fx.append(f_value.wrench.force.x)

            # if len(self.fy)>=200:
            #     self.curvefy.setData(self.t[-200:],self.fy[-200:])
            # else:
            #     self.curvefy.setData(self.t,self.fy)
            # self.ui3.plot_fy.replot()
            # self.fy.append(f_value.wrench.force.y)

            # print('estoy ploteando')
            # print(valores)
            # print(type(valores))


                # plot fuerza y velocidad en x
            if self.ui3.eje_x.isChecked():
                # print(len(self.fx))
                # print(self.num_elementos_plotx)
                if self.tiempo_realx:
                    if len(self.fx)>=self.num_elementos_plotx:
                        self.curvefx.setData(self.tx[-self.num_elementos_plotx:],self.fx[-self.num_elementos_plotx:])
                        # print('self.curvefx es mayor')
                    else:
                        self.curvefx.setData(self.tx,self.fx)
                    self.ui3.plot_fx.replot()
                    
                    if len(self.vx)>=self.num_elementos_plotx:
                        self.curvevx.setData(self.tx[-self.num_elementos_plotx:],self.vx[-self.num_elementos_plotx:])
                    else:
                        self.curvevx.setData(self.tx,self.vx)
                    self.ui3.plot_vx.replot()
                else:
                    self.desplazamiento_en_plotx+=1


                # plot fuerza y velocidad en y
            if self.ui3.eje_y.isChecked():
                if self.tiempo_realy:

                    if len(self.fy)>=self.num_elementos_ploty:
                        self.curvefy.setData(self.ty[-self.num_elementos_ploty:],self.fy[-self.num_elementos_ploty:])
                    else:
                        self.curvefy.setData(self.ty,self.fy)
                    self.ui3.plot_fy.replot()
  
                    
                    if len(self.vy)>=self.num_elementos_ploty:
                        self.curvevy.setData(self.ty[-self.num_elementos_ploty:],self.vy[-self.num_elementos_ploty:])
                    else:
                        self.curvevy.setData(self.ty,self.vy)
                    self.ui3.plot_vy.replot()
                else:
                    self.desplazamiento_en_ploty+=1


                # plot fuerza y velocidad en z
            if self.ui3.eje_z.isChecked():
                if self.tiempo_realz:

                    if len(self.fz)>=self.num_elementos_plotz:
                        self.curvefz.setData(self.tz[-self.num_elementos_plotz:],self.fz[-self.num_elementos_plotz:])
                    else:
                        self.curvefz.setData(self.tz,self.fz)
                    self.ui3.plot_fz.replot()

                    if len(self.vz)>=self.num_elementos_plotz:
                        self.curvevz.setData(self.tz[-self.num_elementos_plotz:],self.vz[-self.num_elementos_plotz:])
                    else:
                        self.curvevz.setData(self.tz,self.vz)
                    self.ui3.plot_vz.replot()
                else:
                    self.desplazamiento_en_plotz+=1


            # self.t.append(self.t[-1]+1)

#.......................control posición...................................................................
    def ir_a_home(self):
        pub_girar.publish('home')

    def boton_giro_presionado(self,value):
        if self.ui3.conectar_robot.isChecked():
            self.mov_direccion_bool=True
            # print('pulsado',value)
            # print(value)
            pub_girar.publish(value)
            #iiciar actualización de valores articulares
            self.timer_valores_art.start(100)
            # print(1)
        else:
            self.obtener_valores_articulares()

    def boton_giro_soltado(self):
        if self.ui3.conectar_robot.isChecked():

            self.mov_direccion_bool=False

            pub_parar_giro.publish(True)

        # art = self.arm_group.get_current_joint_values()
        # print(art)
        # print(2)
        self.timer_valores_art.stop()

        pub_parar_giro.publish(True)

        #parar actualización de valores articulares
        self.timer_valores_art.stop

    #actualizar valores aritulares de sliders
    def joints_value_changed(self):

       #actualizar sliders multiplicando porque tienen un rango de +-3600
        self.ui3.slider_q1.setValue(int(self.ui3.value_q1.value()*10))
        self.ui3.slider_q2.setValue(int(self.ui3.value_q2.value()*10))
        self.ui3.slider_q3.setValue(int(self.ui3.value_q3.value()*10))
        self.ui3.slider_q4.setValue(int(self.ui3.value_q4.value()*10))
        self.ui3.slider_q5.setValue(int(self.ui3.value_q5.value()*10))
        self.ui3.slider_q6.setValue(int(self.ui3.value_q6.value()*10))
        #obtener valores articulares
        self.q1=self.ui3.value_q1.value()
        self.q2=self.ui3.value_q2.value()
        self.q3=self.ui3.value_q3.value()
        self.q4=self.ui3.value_q4.value()
        self.q5=self.ui3.value_q5.value()
        self.q6=self.ui3.value_q6.value()

        #enviar comando
        if not self.slider_pressed_bool and not self.mov_direccion_bool:
            # print('dentro',self.ui3.slider_q1.mousePressEvent)
            self.send_joints()
 
    #actualizar valores aritulares de indicadores numéricos
    def joints_slider_changed(self):

        #actualizar indicadores númericos por si hay cambios en  los sliders dividiendo entre 10 porque el rango de los sliders es +-3600
        self.ui3.value_q1.setValue(self.ui3.slider_q1.value()/10)
        self.ui3.value_q2.setValue(self.ui3.slider_q2.value()/10)
        self.ui3.value_q3.setValue(self.ui3.slider_q3.value()/10)
        self.ui3.value_q4.setValue(self.ui3.slider_q4.value()/10)
        self.ui3.value_q5.setValue(self.ui3.slider_q5.value()/10)
        self.ui3.value_q6.setValue(self.ui3.slider_q6.value()/10)

        #obtener valores articulares
        self.q1=self.ui3.value_q1.value()
        self.q2=self.ui3.value_q2.value()
        self.q3=self.ui3.value_q3.value()
        self.q4=self.ui3.value_q4.value()
        self.q5=self.ui3.value_q5.value()
        self.q6=self.ui3.value_q6.value()
    def slider_pressed(self):
        self.slider_pressed_bool=True

    def send_joints(self):
        self.slider_pressed_bool=False
        #enviar valores articulares
        valores_articulares=[self.ui3.slider_q1.value(),self.ui3.slider_q2.value(),self.ui3.slider_q3.value(),self.ui3.slider_q4.value(),self.ui3.slider_q5.value(),self.ui3.slider_q6.value()]
        valores_articulares_enviar = Int32MultiArray(data=valores_articulares)
        # print(type(valores_articulares_enviar))
        self.i+=1
        # print(self.i)

        pub_valores_art.publish(valores_articulares_enviar)     
    def cargar_ejercicios_xml(self):




        #funciona usando misma libreria que para escribir
        
        # mytree = ET.parse('src/gui_control_ur3/ejercicios/ejercicios.xml')
        # myroot = mytree.getroot()
        # for x in myroot.findall('ejercicio'):
        #     print(x)
        #     item =x.find('tipo').text
        #     price = x.find('nombre').text
        #     print(item, price)        
        # # tagname= dat.getElementsByTagName('ejercicio')
        # # print(tagname[0].data)
        
        #funciona pero importa otra libreria extra
        doc = minidom.parse(self.mi_path+"/ejercicios/ejercicios.xml")
        ejercicios = doc.getElementsByTagName("ejercicio")
        self.ui3.seleccion_ejercicio.addItem('Nuevo ejercicio') 
        d={}
        d['nombre']='Nuevo ejercicio'
        self.ejercicio_actual=d
        for ejercicio in ejercicios:
            dict={}

            nombre = ejercicio.getElementsByTagName("nombre")[0]
            dict['nombre']=nombre.firstChild.data


            tipo = ejercicio.getElementsByTagName("tipo")[0]
            dict['tipo']=tipo.firstChild.data

            if tipo.firstChild.data=='lineal':
                #añadir item a desplegable porque por defecto está seleccionado ejerccio lineal
                self.ui3.seleccion_ejercicio.addItem(nombre.firstChild.data)
                puntos = ejercicio.getElementsByTagName("punto")
                lista_puntos=list()
                for punto in puntos:
                    lista_puntos.append(ast.literal_eval(punto.firstChild.data))
                    # print(punto.firstChild.data)
                dict['puntos']=lista_puntos
            else:
                punto_inicial = ejercicio.getElementsByTagName("punto_inicial")[0]
                dict['punto_inicial']=punto_inicial.firstChild.data
                radio = ejercicio.getElementsByTagName("radio")[0]
                dict['radio']=radio.firstChild.data
                angulo = ejercicio.getElementsByTagName("angulo")[0]
                dict['angulo']=angulo.firstChild.data
            # print(dict,'...................')
            self.lista_ejercicios.append(dict)
            #ponemos como ejercicio seleccionado el primer ejercicio del archivo xml

        # print(self.lista_ejercicios)



    def item_cambiado(self):

        # print(self.ui3.seleccion_ejercicio.currentText())
        if self.ui3.seleccion_ejercicio.currentText()=='Nuevo ejercicio':
            d={}
            d['nombre']='Nuevo ejercicio'
            self.ejercicio_actual=d
        else:    
            for d in self.lista_ejercicios:
                if d['nombre'] == self.ui3.seleccion_ejercicio.currentText():
                    # print(d)
                    self.ejercicio_actual=d
            self.enviar_nombre_ejercicio()
            self.ui3.puntos_seleccionados.clear()
            if self.ejercicio_actual['tipo']=='lineal':
                puntos=self.ejercicio_actual['puntos']
                for i in range(len(puntos)):
                    self.ui3.puntos_seleccionados.append('Punto '+str(i+1)+':  '+str(puntos[i])+'\n')
            else:

                self.ui3.puntos_seleccionados.append('Punto inicial '+str(self.ejercicio_actual['punto_inicial'])+'\n')
                self.ui3.puntos_seleccionados.append('Radio: '+str(self.ejercicio_actual['radio'])+'\n')
                self.ui3.puntos_seleccionados.append('Ángulo: '+str(self.ejercicio_actual['angulo'])+'\n')

    def tipo_ej_seleccionado(self,value):
        #si value es true significa que el se ha seleccionado ejercicio linea
        # desactivar el otro boto
        if value:
            self.ui3.boton_ej_circular.setChecked(False)
            # print(self.ui3.boton_ej_circular.isChecked())
            # print(self.ui3.boton_ej_lineal.isChecked())
            self.ejercicio_lineal=True

        else:
            self.ui3.boton_ej_lineal.setChecked(False)
            # print(self.ui3.boton_ej_circular.isChecked())
            # print(self.ui3.boton_ej_lineal.isChecked())
            self.ejercicio_lineal=False
        self.ui3.seleccion_ejercicio.clear()
        
        self.ui3.seleccion_ejercicio.addItem('Nuevo ejercicio') 
        for d in self.lista_ejercicios:
            if d['tipo'] =='lineal' and self.ejercicio_lineal:
                self.ui3.seleccion_ejercicio.addItem(d['nombre'])
            elif d['tipo'] =='circular' and not self.ejercicio_lineal:
                self.ui3.seleccion_ejercicio.addItem(d['nombre'])
        self.ui3.puntos_seleccionados.clear()
        d={}
        d['nombre']='Nuevo_ejercicio'
        self.ejercicio_actual=d
        self.enviar_nombre_ejercicio()

    def reiniciar_posiciones(self):
        self.lista_puntos_nuevo_ejercicio.clear()
        self.ui3.puntos_seleccionados.clear()
    def nueva_posicion(self):
        self.lista_puntos_nuevo_ejercicio.append([self.ui3.value_x.value(),self.ui3.value_y.value(),self.ui3.value_z.value(),math.radians(self.ui3.value_rx.value()),math.radians(self.ui3.value_ry.value()),math.radians(self.ui3.value_rz.value())])
        # print(self.lista_ejercicios)
        # print('nueva posicion')
        self.ui3.puntos_seleccionados.append('Punto '+str(len(self.lista_puntos_nuevo_ejercicio))+':  '+str(self.ui3.value_x.value())+'  '+str(self.ui3.value_y.value())+'  '+str(self.ui3.value_z.value())+'  '+str(self.ui3.value_rx.value())+'  '+str(self.ui3.value_ry.value())+'  '+str(self.ui3.value_rz.value())+'\n')
    def abrir_ventana_guardar_ejercicio(self):
        self.window4.show()
        self.ui4.info_ejercicio.setText(self.ui3.puntos_seleccionados.toPlainText())

    def cerrar_ventana_guardar_ejercicio(self):
        tree= ET.parse("filename2.xml")
        xmlRoot = tree.getroot()
        child = ET.Element("ejercicio")
        xmlRoot.append(child)
        doc2 = ET.SubElement(xmlRoot, "ejercicio")
        ET.SubElement(doc2, "nombre").text = self.ui4.nombre_ejercicio.toPlainText()
        # print('............        ',self.ui4.nombre_ejercicio.toPlainText(),'           ..........')
        ET.SubElement(doc2, "tipo").text = self.ui3.seleccion_ejercicio.currentText()
        if self.ui3.seleccion_ejercicio.currentText()=='Lineal':
            for i in range(len(self.lista_puntos_nuevo_ejercicio)):
                ET.SubElement(doc2, "punto").text = str(self.lista_puntos_nuevo_ejercicio[i])
        else:
            ET.SubElement(doc2, "punto_inicial").text = str(self.lista_puntos_nuevo_ejercicio[-1])
            ET.SubElement(doc2, "radio").text = str(self.ui3.radio_nuevo_ejercicio.value())
            ET.SubElement(doc2, "angulo").text = str(self.ui3.angulo_nuevo_ejercicio.value())

        tree.write("filename2.xml")
        self.window4.close()
    def ejecutar_ejercicio(self):
        # rep=self.ui3.repeticiones.value()
        # duracion=self.ui3.duracion.value()
        # data=Float32MultiArray(data=[duracion,rep])
        # pub_ejecutar_ejercicio.publish(data)
        #el último elemento 0 o 1 indica el tipo de ejercicio
        if self.ejercicio_actual['nombre']=='Nuevo ejercicio':
            # pdb.set_trace()
            if self.ui3.boton_ej_lineal.isChecked():
                info=[]
                for i in range(len(self.lista_puntos_nuevo_ejercicio)):
                    info.append(self.lista_puntos_nuevo_ejercicio[i])
                rep=self.ui3.repeticiones.value()
                duracion=self.ui3.duracion.value()
                info.append([duracion,rep,0.0,0.0,0.0,0.0])
                array=np.array(info)
                array_1d=array.flatten()
                # print(type(array_1d))
                array_1d=array_1d.tolist()
                # print(array_1d)
                array_float=[float(i) for i in array_1d]
                # print(array_float)
                # array=np.array(array)
                data= Float32MultiArray(data=array_float)
            else:
                info=[]
                info.append(self.lista_puntos_nuevo_ejercicio[-1])
                rep=self.ui3.repeticiones.value()
                duracion=self.ui3.duracion.value()
                radio=self.ui3.radio_nuevo_ejercicio
                angulo=self.ui3.angulo_nuevo_ejercicio()
                info.append([duracion,rep,radio,angulo,0.0,1.0])
                array=np.array(info)
                array_1d=array.flatten()
                # print(type(array_1d))
                array_1d=array_1d.tolist()
                # print(array_1d)
                array_float=[float(i) for i in array_1d]
                # print(array_float)
                # array=np.array(array)
                data= Float32MultiArray(data=array_float)
        else:
            rep=self.ui3.repeticiones.value()
            duracion=self.ui3.duracion.value()
            data=Float32MultiArray(data=[duracion,rep])
        # print(data.data)
        pub_ejecutar_ejercicio.publish(data)
    def enviar_nombre_ejercicio(self):
        pub_nombre_ejercicio.publish(self.ejercicio_actual['nombre'])
    def parar_ejercicio(self):
        pub_parar_ejercicio.publish(True)
    def tipo_de_control(self):
        if self.ui3.tabWidget.currentIndex()==0:
            self.tipo_control='posicion'
            art = self.arm_group.get_current_joint_values()
            # print(art)

        else:
            self.tipo_control='fuerza'
        pub_tipo_control.publish(self.tipo_control)
        # print(self.tipo_control)
    def obtener_valores_articulares(self):
        # print('obteniendo valores articulares')
        # print(self.timer_valores_art.interval())
        art = self.arm_group.get_current_joint_values()
        pos=self.arm_group.get_current_pose()
        # print('pose:  ',pos)

        #actualizar sliders multiplicando porque tienen un rango de +-3600
        self.ui3.slider_q1.setValue(int(math.degrees(art[0])*10))
        self.ui3.slider_q2.setValue(int(math.degrees(art[1])*10))
        self.ui3.slider_q3.setValue(int(math.degrees(art[2])*10))
        self.ui3.slider_q4.setValue(int(math.degrees(art[3])*10))
        self.ui3.slider_q5.setValue(int(math.degrees(art[4])*10))
        self.ui3.slider_q6.setValue(int(math.degrees(art[5])*10))
        self.ui3.value_q1.setValue(int(math.degrees(art[0])))
        self.ui3.value_q2.setValue(int(math.degrees(art[1])))
        self.ui3.value_q3.setValue(int(math.degrees(art[2])))
        self.ui3.value_q4.setValue(int(math.degrees(art[3])))
        self.ui3.value_q5.setValue(int(math.degrees(art[4])))
        self.ui3.value_q6.setValue(int(math.degrees(art[5])))

        #actualizar valores cartesianos
        self.ui3.value_x.setValue(pos.pose.position.x)
        self.ui3.value_y.setValue(pos.pose.position.y)
        self.ui3.value_z.setValue(pos.pose.position.z)
        qx=pos.pose.orientation.x
        qy=pos.pose.orientation.y
        qz=pos.pose.orientation.z
        qw=pos.pose.orientation.w

        # self.ui3.value_rx.setValue(pos[3])
        # self.ui3.value_ry.setValue(pos[4])
        # self.ui3.value_rz.setValue(pos[5])
        rx,ry,rz=self.euler_from_quaternion(qx,qy,qz,qw)
        self.ui3.value_rx.setValue(math.degrees(rx))
        self.ui3.value_ry.setValue(math.degrees(ry))
        self.ui3.value_rz.setValue(math.degrees(rz))
        #obtener valores articulares
        self.q1=self.ui3.value_q1.value()
        self.q2=self.ui3.value_q2.value()
        self.q3=self.ui3.value_q3.value()
        self.q4=self.ui3.value_q4.value()
        self.q5=self.ui3.value_q5.value()
        self.q6=self.ui3.value_q6.value()

    def euler_from_quaternion(self,x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

if __name__ == "__main__":
    
    pub = rospy.Publisher('iniciar_driver', String, queue_size=10)
    pub_iniciar_move_group = rospy.Publisher('move_group_fake_execution', Bool, queue_size=10)

    pub_calibrar = rospy.Publisher('calibrar', String, queue_size=10)
    pub_tipo_control= rospy.Publisher('tipo_control', String, queue_size=10)
    pub_admitancia_on=rospy.Publisher('control_admitancia',Bool,queue_size=10)
    pub_configuracion=rospy.Publisher('configuracion',Float32MultiArray,queue_size=10)
    pub_valores_art = rospy.Publisher('valores_articulares',Int32MultiArray , queue_size=100)
    pub_girar = rospy.Publisher('girar',String, queue_size=10)
    pub_parar_giro = rospy.Publisher('parar_giro',Bool , queue_size=10)
    pub_ejecutar_ejercicio = rospy.Publisher('ejecutar_ejercicio',Float32MultiArray , queue_size=10)
    pub_nombre_ejercicio=rospy.Publisher('nombre_ejercicio',String, queue_size=10)
    pub_parar_ejercicio = rospy.Publisher('parar_ejercicio',Bool , queue_size=10)

    # pub_cartesiano = rospy.Publisher('girar_cartesiano',String, queue_size=100)

    rospy.init_node('ventana_config', anonymous=True)

    app=QtWidgets.QApplication(sys.argv)
    myapp=Miformulario()
    myapp.show()
    sys.exit(app.exec_())


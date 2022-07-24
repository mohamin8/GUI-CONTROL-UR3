# GUI-CONTROL-UR3
Este respositorio incluye el control de posición y admitancia del robot UR3 mediante una interfaz gráfica y ROS

# Instalación

Para poder usar la aplicación primero hay que realizar una serie de instalaciones de paquetes de las que depende el proyecto: 

Paso 1: instalar ROS

Este proyecto se ha desarrollado en Ubuntu 20.4. En esta versión solo está disponible una distribución de ROS: Noetic. Para poder instalarla hay que seguir los pasos de http://wiki.ros.org/noetic/Installation/Ubuntu

Paso 2: Crear el espacio de trabajo

El siguiente paso es crear el Workspace ejecutando los siguientes comandos en el terminal 

$ source /opt/ros/noetic/setup.bash 
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make

Paso 3: Instalación del driver del UR3

Como se ha comentado con anterioridad, este driver permite la conexión entre el pc y el robot. Todos los pasos a seguir para su correcta instalación se encuentran en el repositorio del driver https://github.com/UniversalRobots/Universal_Robots_ROS_Driver . En resumidas cuentas, hay que ejecutar los siguientes comandos:

#source global ros
$ source /opt/ros/<your_ros_version>/setup.bash

#create a catkin workspace
$ mkdir -p catkin_ws/src && cd catkin_ws

#clone the driver
$ git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver

#clone fork of the description. This is currently necessary, until the changes are merged upstream.
$ git clone -b calibration_devel https://github.com/fmauch/universal_robot.git src/fmauch_universal_robot

#install dependencies
$ sudo apt update -qq
$ rosdep update
$ rosdep install --from-paths src --ignore-src -y

#build the workspace
$ catkin_make

#activate the workspace (ie: source it)
$ source devel/setup.bash

Paso 4: Configuración del UR3

Para poder trabajar con el robot primero hay que prepararlo. Al igual que en el pc hay que instalar el driver, en el robot hay que instalar externalcontrol-1.0.4.urcap. Cabe recalcar que para poder instalarlo hay que tener PolyScope, versión 3.7 o superior, si se tratara de un eUR3 valdría desde la versión 5.1.
Todo lo necesario para instalar y crear el programa de URCap se encuentra en https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/install_urcap_cb3.md , ahí se comenta que hay que introducir una dirección IP, esa será la que tendrá el PC. Es importante destacar que el robot tiene que estar conectado a la misma red que el ordenador.

Paso 5: Integración de la interfaz
	
	Es necesario introducir el paquete de la interfaz y control ‘gui_control_ur3’ dentro de la carpeta src del espacio de trabajo. Para posteriormente situado en el espacio de trabajo ejecutar catkin_make.
	
Paso 6: Instalación de dependencias:

	Puesto que se ha desarrollado la interfaz con PyQt, este software ha de ser instalado en el ordenador con los siguientes comandos:
	
		pip3 install --user pyqt5  
		sudo apt-get install python3-pyqt5  
		sudo apt-get install pyqt5-dev-tools
		sudo apt-get install qttools5-dev-tools
 	
	Además, es necesario instalar otras dependencias para asegurar el correcto funcionamiento de la interfaz:
	
	Para obtener la IP:
		sudo apt install net-tools

	Para plotear:
		pip install qtpy	
# Ejecución del programa
Una vez instalado todo correctamente, se puede empezar a trabajar con la interfaz

Paso 2: Preparar el espacio de trabajo

Desde la terminal, situado en la carpeta de trabajo, hay que ejecutar:

	Source devel/setup.bash
	
Paso 3: Lanzar el ejecutable

	Roslaunch gui_control_ur3 gui_control_ur3_launch.launch
	
Paso 4: Iniciar conexión 

Se abrirá la ventana de configuración. Se pulsar sobre ‘conectar robot’ e introducir la IP de este último. Si es la primera vez que usa se ha de pulsar ‘calibrar el robot’. Una vez finalizado el proceso de calibración se pulsa ‘iniciar driver’ y se siguen las indicaciones que aparecen en la ventana de carga. Si ya se ha calibrado el robot con anterioridad se ha de pulsar en ‘buscar archivo’ para seleccionar el archivo de calibración. Terminado el proceso, aparece la ventana de control. Ya se puede trabajar con el UR3.



#!/usr/bin/env python3
import rospy, os, sys, math, time

from std_msgs.msg import Header, String, Int32, Float64
from geometry_msgs.msg import Twist, Vector3, Pose
from sensor_msgs.msg import Joy
from math import pi

import numpy as np
import cv2
import cv2.aruco as aruco
import time

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge() 

arucoMarkerLength = 0.057 
font = cv2.FONT_HERSHEY_SIMPLEX

# Inicializacion de la clase
class AR():
    
    # Diccionarios
    def __init__(self, videoPort, cameraMatrix, distortionCoefficients):
        # Matriz de parámetros intrínsecos (calibración) y de distorsión
        self.cameraMatrix = cameraMatrix
        self.distortionCoefficients = distortionCoefficients
        
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50) # Rojo: DICT_4X4_50   Verde: DICT_6X6_250   Azul: DICT_5X5_50
        self.dictionary2 = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.dictionary3 = aruco.getPredefinedDictionary(aruco.DICT_5X5_50)
        
        # Número de bloques a manipular
        self.N = 3
        
        # Lista de bloques en pantalla (0 - 2: actual | 3 - 5: previas)
        self.blocks = []

        
        # Lista de la posición actual y anterior de los bloques (0 - 2: actual | 3 - 5: previas)
        self.blocks_pos = []
        
        # Mensajes a enviar por el topic
        self.Pose_msgs = []

        for i in range(self.N):
            self.Pose_msgs.append(String())
        
        # Lista del tiempo que llevan en pantalla (0 - 2: actual | 3 - 5: previas)
        self.blocks_time = []

        self.sec = [0, 0, 0]
        self.prev_sec = [0,0,0]
        
        for k in range(self.N * 2):
            self.blocks_time.append(0.0)
            self.blocks_pos.append([-1, -1])
            self.blocks.append(False)

            
        # Publisher para comunicar al nodo del robot
        self.publisher = rospy.Publisher("/store", String, queue_size=10)

        rospy.Subscriber("/voice_ui", String, self.store_cb)


    # Función que detecta donde está el aruco a partir del frame
    def find_ARMarker(self, frame):
        self.frame = frame
        if len(self.frame.shape) == 3:
            self.Height, self.Width, self.channels = self.frame.shape[:3]
        else:
            self.Height, self.Width = self.frame.shape[:2]
            self.channels = 1
        self.halfHeight = int(self.Height / 2)
        self.halfWidth = int(self.Width / 2)
        
        # Detecta los arucos a partir de los diccionarios y obtiene las esquinas y el ID
        self.corners, self.ids, self.rejectedImgPoints = aruco.detectMarkers(self.frame, self.dictionary)
        aruco.drawDetectedMarkers(self.frame, self.corners, self.ids, (0,255,0))
        
        self.corners2, self.ids2, self.rejectedImgPoints2 = aruco.detectMarkers(self.frame, self.dictionary2)
        aruco.drawDetectedMarkers(self.frame, self.corners2, self.ids2, (0,255,0))
        
        self.corners3, self.ids3, self.rejectedImgPoints3 = aruco.detectMarkers(self.frame, self.dictionary3)
        aruco.drawDetectedMarkers(self.frame, self.corners3, self.ids3, (0,255,0))
        
        
    # Muestra imagen de la cámara      (ELIMINAR EN VERSIÓN FINAL)
    def show(self):
        cv2.imshow("result", self.frame)
        cv2.waitKey(1)

    # Función para determinar si existe el marcador aruco a partir de las esquinas (0 esquinas = no hay aruco)
    # Hay un par para cada bloque
    def get_exist_Marker(self):
        return len(self.corners)

    def is_exist_marker(self, i):
        num = self.get_exist_Marker()
        if i >= num:
            return False
        else:
            return True


    def get_exist_Marker2(self):
        return len(self.corners2)

    def is_exist_marker2(self, i):
        num = self.get_exist_Marker2()
        if i >= num:
            return False
        else:
            return True
        
        
    def get_exist_Marker3(self):
        return len(self.corners3)

    def is_exist_marker3(self, i):
        num = self.get_exist_Marker3()
        if i >= num:
            return False
        else:
            return True


    def get_ARMarker_points(self, i):
        if self.is_exist_marker(i):
            return self.corners[i]


    def get_ARMarker_points2(self, i):
        if self.is_exist_marker2(i):
            return self.corners2[i]


    def get_ARMarker_points3(self, i):
        if self.is_exist_marker3(i):
            return self.corners3[i]


    # Obtiene la ubicación de los marcadores Aruco para cada caso:
    '''         FUNCIONAMIENTO
        Se obtienen los puntos de las esquinas detectados en "find_ARMarker" si existe el marcador. Se calcula el centro y se dibuja un
    círculo en esa posición en el frame actual, luego se guardan las poses en la lista de posiciones.
        Si no existe el aruco, es porque alguien o el robot lo ha quitado, por lo que la posición actual se considera inválida (-1, -1)  y su 
    existencia a False  
    '''
    def get_average_point_marker(self, i):
        
        # Primer caso
        if self.is_exist_marker(i):
            points = self.get_ARMarker_points(i)
            points_reshape = np.reshape(np.array(points), (4, -1))
            G = np.mean(points_reshape, axis = 0)
            cv2.circle(self.frame, (int(G[0]), int(G[1])), 10, (255, 255, 255), 5)
            
            self.blocks_pos[0] = [int(G[0]), int(G[1])]
            self.blocks[0] = True
            
        else:
            self.blocks_pos[0] = [-1, -1]
            self.blocks[0] = False
        
        
        # Segundo caso
        if self.is_exist_marker2(i):
            points = self.get_ARMarker_points2(i)
            points_reshape = np.reshape(np.array(points), (4, -1))
            G = np.mean(points_reshape, axis = 0)
            cv2.circle(self.frame, (int(G[0]), int(G[1])), 10, (255, 255, 255), 5)
            
            self.blocks_pos[1] = [int(G[0]), int(G[1])]
            self.blocks[1] = True

        else:
            self.blocks_pos[1] = [-1, -1]
            self.blocks[1] = False
            
        # Tercer caso
        if self.is_exist_marker3(i):
            points = self.get_ARMarker_points3(i)
            points_reshape = np.reshape(np.array(points), (4, -1))
            G = np.mean(points_reshape, axis = 0)
            cv2.circle(self.frame, (int(G[0]), int(G[1])), 10, (255, 255, 255), 5)
            
            self.blocks_pos[2] = [int(G[0]), int(G[1])]
            self.blocks[2] = True
        else:   
            self.blocks_pos[2] = [-1, -1]
            self.blocks[2] = False
        
        
    def release(self):
        self.cap.release()
        
    
    # Funciones para cada Aruco para obtener los vectores de rotación y tralsación en el sistema de la cámara
    def get_ARMarker_pose(self, i):
        if self.is_exist_marker(i):
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(self.corners[i], arucoMarkerLength, self.cameraMatrix, self.distortionCoefficients)
            return rvec, tvec
    
    def get_ARMarker_pose2(self, i):
        if self.is_exist_marker2(i):
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(self.corners2[i], arucoMarkerLength, self.cameraMatrix, self.distortionCoefficients)
            return rvec, tvec
    
    def get_ARMarker_pose3(self, i):
        if self.is_exist_marker3(i):
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(self.corners3[i], arucoMarkerLength, self.cameraMatrix, self.distortionCoefficients)
            return rvec, tvec


    # Se obtiene la posición en el sistema global de cada uno de los arucos
    def get_degrees(self, i):
        if self.is_exist_marker(i):
            
            # Obtiene rotación y traslación
            rvec, tvec, = self.get_ARMarker_pose(i)
            
            # Se pasa a coordenadas globales (pi/2, es el ángulo que está girado la cámara
            #                                 [0.0, -0.5, 0.5 son las coordenadas de la cámara])
            Xtemp = tvec[0][0][0]
            Ytemp = tvec[0][0][2]*math.cos(pi/2) - tvec[0][0][1]*math.sin(pi/2)
            Ztemp = tvec[0][0][2]*math.sin(pi/2) + tvec[0][0][1]*math.cos(pi/2)

            # print(tvec[0][0])
            Xtemp2 = Xtemp + 0.35
            Ytemp2 = Ytemp - 0.35
            Ztemp2 = Ztemp - 0.5

            Xtarget = Xtemp2
            Ytarget = Ytemp2
            Ztarget = -Ztemp2

            # print(f"(X, Y, Z) : {Xtarget}, {Ytarget}, {Ztarget}")

            # Se obtienen los ángulos de Euler
            (roll_angle, pitch_angle, yaw_angle) =  rvec[0][0][0]*180/pi, rvec[0][0][1]*180/pi, rvec[0][0][2]*180/pi
            if pitch_angle < 0:
                roll_angle, pitch_angle, yaw_angle = -roll_angle, -pitch_angle, -yaw_angle
            
            # Se construye el mensaje
            self.Pose_msgs[0] = str(-Xtarget) + " "
            self.Pose_msgs[0] += str(-Ytarget) + " 0"

            
        if self.is_exist_marker2(i):
            
            # Obtiene rotación y traslación
            rvec, tvec, = self.get_ARMarker_pose2(i)
            
            # Se pasa a coordenadas globales (pi/2, es el ángulo que está girado la cámara
            #                                 [0.0, -0.5, 0.5 son las coordenadas de la cámara])
            Xtemp = tvec[0][0][0]
            Ytemp = tvec[0][0][2]*math.cos(pi/2) - tvec[0][0][1]*math.sin(pi/2)
            Ztemp = tvec[0][0][2]*math.sin(pi/2) + tvec[0][0][1]*math.cos(pi/2)

            Xtemp2 = Xtemp + 0.35
            Ytemp2 = Ytemp - 0.35
            Ztemp2 = Ztemp - 0.5


            
            Xtarget = Xtemp2
            Ytarget = Ytemp2
            Ztarget = -Ztemp2

            # print(f"(X, Y, Z) : {Xtarget}, {Ytarget}, {Ztarget}")

            (roll_angle, pitch_angle, yaw_angle) =  rvec[0][0][0]*180/pi, rvec[0][0][1]*180/pi, rvec[0][0][2]*180/pi
            if pitch_angle < 0:
                roll_angle, pitch_angle, yaw_angle = -roll_angle, -pitch_angle, -yaw_angle
            
            # Se construye el mensaje
            self.Pose_msgs[1] = str(-Xtarget) + " "
            self.Pose_msgs[1] += str(-Ytarget) + " 1"
          
            
        if self.is_exist_marker3(i):
            
            # Obtiene rotación y traslación
            rvec, tvec, = self.get_ARMarker_pose3(i)
            
            # Se pasa a coordenadas globales (pi/2, es el ángulo que está girado la cámara
            #                                 [0.0, -0.5, 0.5 son las coordenadas de la cámara])
            Xtemp = tvec[0][0][0]
            Ytemp = tvec[0][0][2]*math.cos(pi/2) - tvec[0][0][1]*math.sin(pi/2)
            Ztemp = tvec[0][0][2]*math.sin(pi/2) + tvec[0][0][1]*math.cos(pi/2)

            Xtemp2 = Xtemp + 0.35
            Ytemp2 = Ytemp - 0.35
            Ztemp2 = Ztemp - 0.5

            Xtarget = Xtemp2
            Ytarget = Ytemp2
            Ztarget = -Ztemp2

            # print(f"(X, Y, Z) : {Xtarget}, {Ytarget}, {Ztarget}")

            # Se obtienen los ángulos de Euler
            (roll_angle, pitch_angle, yaw_angle) =  rvec[0][0][0]*180/pi, rvec[0][0][1]*180/pi, rvec[0][0][2]*180/pi
            if pitch_angle < 0:
                roll_angle, pitch_angle, yaw_angle = -roll_angle, -pitch_angle, -yaw_angle
            
            # Se construye el mensaje
            self.Pose_msgs[2] = str(-Xtarget) + " "
            self.Pose_msgs[2] += str(-Ytarget) + " 2"
                
    def store_cb(self, data):
        s = String()

        if data.data == "sal" and self.blocks[0]:
            s.data = self.Pose_msgs[0] + " " + data.data
            

        elif data.data == "azúcar" and self.blocks[1]:
            s.data = self.Pose_msgs[1] + " " + data.data
            

        elif data.data == "pimienta" and self.blocks[2]:
            s.data = self.Pose_msgs[2] + " " + data.data

        self.publisher.publish(s)
            
        
            
camera_matrix = np.matrix([[381.36246688113556, 0.0, 320.5], [0.0, 381.36246688113556, 240.5], [0.0, 0.0, 1.0]])
distortion = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

# Se declara el objeto del reconocedor
myCap = AR(0, camera_matrix, distortion)


# Callback de la cámara
'''          FUNCIONAMIENTO
    Se pasa la imagen a una matriz (en ROS se pasa como un vector uint8). Luego, se detectan los aruco en la imagen para después obtener sus
posiciones y actualizar su presencia. Luego se obtienen sus posiciones en elsistema de referencia del mundo
    Luego, para cada bloque se guarda la posición actual y la previa. (de la cámara, es más estable)
    En orden de IFs:
        - Si la posición previa es inválida y la actual no: acaba de detectarse --> se considera que el bloque existe y se empieza la cuenta del tiempo
        - Si el bloque existe: 
          - Si el bloque no se ha movido más allá de un margen: se actualiza el tiempo
          - Si el tiempo supera un máximo (10 s): se manda el mensaje de recogida y se considera que el bloque no está
          
    Se actualizan las posiciones previas      
''' 
def callback_color_img(data):
    cv_color_image = bridge.imgmsg_to_cv2(data, "bgr8")
    myCap.find_ARMarker(cv_color_image)
    myCap.get_average_point_marker(0)
    myCap.get_degrees(0)  

    
    
# Bucle infinito mientras funcione ROS
def commander():

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        try:
            rate.sleep()
        except rospy.ROSException:
            print("restart simulation")

# Main
if __name__ == '__main__':
    rospy.init_node('commander', anonymous=True)
    rospy.Subscriber("/robot_camera2/image_raw", Image, callback_color_img)
    commander()
    rospy.spin()
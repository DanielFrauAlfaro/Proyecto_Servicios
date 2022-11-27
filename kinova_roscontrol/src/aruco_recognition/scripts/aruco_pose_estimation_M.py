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
    def __init__(self, videoPort):
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50) # Rojo: DICT_4X4_50   Verde: DICT_6X6_250   Azul: DICT_5X5_50
        self.dictionary2 = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.dictionary3 = aruco.getPredefinedDictionary(aruco.DICT_5X5_50)
        
        # Número de bloques a manipular
        self.N = 3
        
        # Lista de bloques en pantalla (0 - 2: actual | 3 - 5: previas)
        self.blocks = []

        
        # Lista de la posición actual y anterior de los bloques (0 - 2: actual | 3 - 5: previas)
        self.blocks_pos = []

        
        # Lista del tiempo que llevan en pantalla (0 - 2: actual | 3 - 5: previas)
        self.blocks_time = []

        
        for k in range(self.N * 2):
            self.blocks_time.append(0.0)
            self.blocks_pos.append([-1, -1])
            self.blocks.append(False)
            
        # Publisher para comunicar al nodo del robot
        self.publisher = rospy.Publisher("/ready", String, queue_size=10)

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
            
        else:   
            self.blocks_pos[2] = [-1, -1]
            self.blocks[2] = False
        
        
    def release(self):
        self.cap.release()
    

# Se declara el objeto del reconocedor
myCap = AR(0)

# Callback de la cámara
'''          FUNCIONAMIENTO
    Se pasa la imagen a una matriz (en ROS se pasa como un vector uint8). Luego, se detectan los aruco en la imagen para después obtener sus
posiciones y actualizar su presencia.
    Luego, para cada bloque se guarda la posición actual y la previa. 
    En orden de IFS:
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
    
    for i in range(myCap.N):
        x = myCap.blocks_pos[i][0]
        y = myCap.blocks_pos[i][1]
        
        prev_x = myCap.blocks_pos[i + myCap.N][0]
        prev_y = myCap.blocks_pos[i + myCap.N][1]
        
        margin = 0.2
        
        if myCap.blocks_pos[i+myCap.N] == [-1, -1] and myCap.blocks_pos[i] != [-1, -1]:
            myCap.blocks[i] = True
            myCap.blocks_time[i] = time.time()
            print("OBJECT ", i)
        
        if myCap.blocks[i]:
            if (x >= prev_x-prev_x*margin and x <= prev_x+prev_x*margin) and \
                (y >= prev_y-prev_y*margin and y <= prev_y+prev_y*margin):
                myCap.blocks_time[i + myCap.N] = time.time()
                print("wait ", myCap.blocks_time[i + myCap.N] - myCap.blocks_time[i])
            
            if (myCap.blocks_time[i + myCap.N] - myCap.blocks_time[i]) > 10.0:
                print("################### READY #######################")
                msg = String()
                msg.data = str(i)
                myCap.publisher.publish(msg)
                myCap.blocks[i]= False
     
    for i in range(myCap.N):
        myCap.blocks_pos[i + myCap.N] = myCap.blocks_pos[i]   
    
    myCap.show()
    if cv2.waitKey(1) > 0:
        myCap.release()
        cv2.destroyAllWindows()
    
    
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
    rospy.Subscriber("/robot_camera/image_raw", Image, callback_color_img)
    commander()
    rospy.spin()
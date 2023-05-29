#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import RegionOfInterest
from cv_bridge import CvBridge
import ctypes
from rospy import Time
from geometry_msgs.msg import Point

class detector:
    def detect_green_object(self,image):
        # Convertir la imagen a formato HSV
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Definir el rango de colores verdes en HSV
        lower_green = (40, 50, 50)
        upper_green = (70, 255, 255)

        # Aplicar una máscara para detectar los píxeles verdes en la imagen
        mask = cv2.inRange(hsv_image, lower_green, upper_green)

        # Encontrar los contornos de los objetos verdes
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Encontrar el contorno más grande
        largest_contour = max(contours, key=cv2.contourArea)

        # Obtener el rectángulo delimitador y las coordenadas del objeto más grande
        x, y, w, h = cv2.boundingRect(largest_contour)

        return x, y, w, h

    def __init__(self):
        # Inicializar el nodo ROS
        rospy.init_node('green_detector')
        rate=rospy.Rate(10)
        # Crear el publicador para el tópico "green_object"
        self.pub = rospy.Publisher('/coordenadas', Point, queue_size=10)
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()

    def update(self):
        # Crear el objeto CvBridge
        bridge = CvBridge()

        # Leer la imagen
        image = cv2.imread('/home/robotics/catkin_ws/src/mi_proyecto/src/imagen.jpg')

        # Detectar el objeto verde
        x, y, w, h = self.detect_green_object(image)

        # Crear el mensaje RegionOfInterest
        roi_msg = RegionOfInterest()
        roi_msg.x_offset = x
        roi_msg.y_offset = y
        roi_msg.width = w
        roi_msg.height = h

        
        print("coordenadas X,Y: {}, {}".format(x,y))
        #cpp
        lib = ctypes.CDLL("/home/robotics/catkin_ws/src/mi_proyecto/lib/libcoordinate_multiplier.so")

        lib.multiplyCoordinates.argtypes = [ctypes.c_int]
        lib.multiplyCoordinates.restype = ctypes.c_int
        cx100 = lib.multiplyCoordinates(ctypes.c_int(cx)) 
        cy100 = lib.multiplyCoordinates(ctypes.c_int(cy))
        rospy.loginfo("Green object found at coordinates X: " + str(cx100) + ", Y: " + str(cy100) + "\ln")

        point_msg = Point()
        point_msg.x = cx100
        point_msg.y = cy100 
        point_msg.header.stamp = rospy.Time.now()
        # Publicar el mensaje en el tópico "green_object"
        self.pub.publish(point_msg)

    # Mostrar la imagen con el objeto detectado
    #cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
    #cv2.imshow('Green Object Detection', image)

if __name__ == '__main__':
    det = detector()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass



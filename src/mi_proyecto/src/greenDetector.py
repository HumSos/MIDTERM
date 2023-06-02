#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import RegionOfInterest
from cv_bridge import CvBridge
import ctypes
from rospy import Time
from geometry_msgs.msg import PoseStamped
import grpc
from concurrent import futures
import coordinates_pb2
import coordinates_pb2_grpc


class CoordinateServiceServicer(coordinates_pb2_grpc.CoordinateServiceServicer):
    """
    @class CoordinateServiceServicer
    Clase para manejar los servicios de las coordenadas
    """

    def get_coordinates(self):
        """
        @brief Obtener coordenadas
        Obtiene las coordenadas y las devuelve
        """
        while True:
            # Obtener las coordenadas del objeto
            x, y = get_object_coordinates()

            # Crear un mensaje Coordinate
            coordinate = coordinates_pb2.Coordinate()
            coordinate.x = x
            coordinate.y = y

            yield coordinate
            
    def GetCoordinates(self, request, context):
        """
        @brief GetCoordinates
        Request de coordenadas
        """
        # Implementa la lógica para obtener las coordenadas del objeto
        # y enviarlas como un flujo de mensajes
        for coordinate in self.get_coordinates():
            yield coordinate

    
class detector:
    """
    @class detector
    Clase donde se van a realizar los procesos relacionados con la deteccion del objeto y los calculos principales
    """
    
    def detect_green_object(self,image):
        """
        @brief detect_green_object
        Funcion para detectar el objeto verde mas grande de la imagen y devolver sus coordenadas 
        """
        # Convertir la imagen a formato HSV
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Rango de colores verdes en HSV
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

        return x, y

    def __init__(self):
        """
        @brief constructor
        Funcion para inicializar el nodo del detector de objetos y procesos principales
        """
        # Inicializar el nodo ROS
        rospy.init_node('green_detector')
        rate=rospy.Rate(10)
        # Crear el publicador para el tópico "green_object"
        self.pub = rospy.Publisher('/coordenadas', PoseStamped, queue_size=10)

        # Crear un servidor gRPC
        server = grpc.server(futures.ThreadPoolExecutor())
        # Registrar el servicio en el servidor
        coordinates_pb2_grpc.add_CoordinateServiceServicer_to_server(CoordinateServiceServicer(), server)
        # Configurar el puerto en el que el servidor escuchará las solicitudes RPC
        server.add_insecure_port('[::]:50051')
        server.start()

        while not rospy.is_shutdown():
            self.update()
            rate.sleep()

    def update(self):
        """
        @brief update
        Funcion para hacer los calculos con las coordenadas y publicarlas
        """
        # Crear el objeto CvBridge
        bridge = CvBridge()

        # Leer la imagen
        image = cv2.imread('/home/robotics/catkin_ws/src/mi_proyecto/src/imagen.jpg')

        # Detectar el objeto verde
        x, y = self.detect_green_object(image)

        # Crear el mensaje PoseStamped
        pose_msg = PoseStamped()
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y

        #cpp
        lib = ctypes.CDLL("/home/robotics/catkin_ws/src/mi_proyecto/lib/libcoordinate_multiplier.so")

        lib.multiplyCoordinates.argtypes = [ctypes.c_int]
        lib.multiplyCoordinates.restype = ctypes.c_int
        cx100 = lib.multiplyCoordinates(ctypes.c_int(x)) 
        cy100 = lib.multiplyCoordinates(ctypes.c_int(y))
        rospy.loginfo("Green object found at coordinates X: " + str(cx100) + ", Y: " + str(cy100) + "\ln")

        pose_msg100 = PoseStamped()
        pose_msg100.pose.position.x = cx100
        pose_msg100.pose.position.y = cy100 
        pose_msg100.header.stamp = rospy.Time.now()
        # Publicar el mensaje en el tópico "green_object"
        self.pub.publish(pose_msg100)

if __name__ == '__main__':
    det = detector()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass



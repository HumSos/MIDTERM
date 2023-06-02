#!/usr/bin/python3
import rospy
import grpc
from concurrent import futures
import coordinates_pb2 as coordinates_pb
import coordinates_pb2_grpc as coordinates_pb_grpc
from geometry_msgs.msg import PoseStamped

class GreenObjectServicer(coordinates_pb_grpc.GreenObjectServiceServicer):
    """@class GreenObjectServicer
    @brief gRPC GreenObjectServicer

    """
    def __init__(self):
        """@brief Constructor

        Inicializa el servidor gRPC4.
        """
        self.x = 0
        self.y = 0
        self.server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
        coordinates_pb_grpc.add_GreenObjectServiceServicer_to_server(self, self.server)
        self.server.add_insecure_port('[::]:50051')

    def PublishObjectCoordinates(self, request, context):
        """@brief Publicar coordenadas.

        Obtiene solicitud y devuelve coordenadas
        @param request: Solicitud.
        @param context: Contexto de la solicitud.
        @return Coordenadas del objeto.
        """
        result = coordinates_pb.ObjectCoordinates()
        result.x = self.x
        result.y = self.y
        result.timestamp = self.timestamp

        return result

    def coordsCallback(self, data:PoseStamped):
        """@brief Procesamiento de coordenadas

        Obtiene los datos de las coordenadas y los almacena correspondientemente
        @param data: Coordenadas del objeto.
        """
        self.x = data.data[0]
        self.y = data.data[1]
        self.timestamp = data.data[3]

def srv():
    """@brief Método para iniciar el servidor gRPC y suscribirse al tópico de coordenadas del objeto."""
    rospy.init_node('100_server')

    service = GreenObjectServicer()
    rospy.Subscriber('/coordenadas', PoseStamped, service.coordsCallback)
    service.server.start()
    rospy.loginfo("Server started")
    rospy.spin()

if __name__ == '__main__':
    serv = srv()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
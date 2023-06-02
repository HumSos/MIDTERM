import grpc
import coordinates_pb2
import coordinates_pb2_grpc
import rospy

class client:
    """@class client
    @brief gRPC client

    """
    def __init__(self):
        """
        @brief Constructor

        Función para inicializar el nodo de ros
        """
        # Inicializar el nodo ROS
        rospy.init_node('grpc_client')
        rate=rospy.Rate(10)
        
        while not rospy.is_shutdown():
            self.consume_coordinates()
            rate.sleep()

    def consume_coordinates():
        """@brief Consumir coordenadas.

        Crea el cliente y hace la llamada
        """
        with grpc.insecure_channel('localhost:50051') as channel:
            # Crea un cliente para el servicio CoordinateService
            stub = coordinates_pb2_grpc.GreenObjectServiceStub(channel)

            # Realiza la llamada al método GetCoordinates
            response = stub.PublishObjectCoordinates(coordinates_pb2.Empty())

            # imprimir la respuesta
            print("Coordenadas recibidas: x={}, y={}".format(response.x, response.y))

if __name__ == '__main__':
    cli = client()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
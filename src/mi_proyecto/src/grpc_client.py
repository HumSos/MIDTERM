import grpc
import coordinates_pb2
import coordinates_pb2_grpc
import rospy

##
# Clase del cliente
#
class client:
    def __init__(self):
        # Inicializar el nodo ROS
        rospy.init_node('grpc_client')
        rate=rospy.Rate(10)
        
        while not rospy.is_shutdown():
            self.consume_coordinates()
            rate.sleep()

    ##
    # Funcion para consumir las coordenadas
    #
    def consume_coordinates():
        with grpc.insecure_channel('localhost:50051') as channel:
            # Crea un cliente para el servicio CoordinateService
            stub = coordinates_pb2_grpc.CoordinateServiceStub(channel)

            # Realiza la llamada al método GetCoordinates
            response_iterator = stub.GetCoordinates(coordinates_pb2.Empty())

            # Itera sobre el flujo de coordenadas recibido
            for coordinate in response_iterator:
                print('Coordinate received:', coordinate)

if __name__ == '__main__':
    cli = client()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
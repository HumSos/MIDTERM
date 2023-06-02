#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <iostream>
using namespace std;

////
//Funcion que multiplica la coordenada por 100
//
extern "C" {
  int multiplyCoordinates(int x) {
    int result = x * 100;
    return result;
  }
}

int main(int argc, char** argv)
{
  // Inicializar el nodo ROS
  ros::init(argc, argv, "coordinate_multiplier");

  // Crear un objeto de tipo NodeHandle
  ros::NodeHandle nh;

  // Obtiene coords
  int x = 10;
  // ...

  // Multiplicar las coordenadas por 100
  x = multiplyCoordinates(x);

  // Imprimir las coordenadas multiplicadas
  ROS_INFO("Coordenadas después de la multiplicación:");
  ROS_INFO_STREAM("x*100 = " << x);

  ros::spin();  

  return 0;
}   
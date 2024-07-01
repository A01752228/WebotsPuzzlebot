#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from controller import Robot, Camera, Lidar

v = 0.0
w = 0.0

radio = 0.05
dllantas = 0.19

def velocidad_callback(data):
    global v, w
    v = data.linear.x
    w = data.angular.z

def main():
    rospy.init_node("puzzlebot")

    # Suscriptores
    rospy.Subscriber("/cmd_vel", Twist, velocidad_callback)
    # Publicadores
    pub_wl = rospy.Publisher("/wl", Float32, queue_size=10)
    pub_wr = rospy.Publisher("/wr", Float32, queue_size=10)
    
    # Creacion del Robot
    robot = Robot()

    timestep = int(robot.getBasicTimeStep())
    
    # Declaracion de los encoders
    right_motor = robot.getDevice('wr')
    left_motor = robot.getDevice('wl')
    
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)

    # Configuracion de la camara
    camera = robot.getDevice("camera")
    camera.enable(timestep)
    
    # Configuracion del LIDAR
    lidar = robot.getDevice('lidar')
    lidar.enable(timestep)
    lidar.enablePointCloud()
    
    while robot.step(timestep) != -1:
        current_time = robot.getTime()
        
        # Obtencion del Rango del Lidar
        range_image = lidar.getRangeImage()
        #print(range_image[0:5])
        
        # Calculo de las velocidades de las llantas
        wl = (v - w * (dllantas / 2.0)) / radio
        wr = (v + w * (dllantas / 2.0)) / radio
        
        # Instanciar la velocidad en los motores
        left_motor.setVelocity(wl)
        right_motor.setVelocity(wr)
        
        # Publicacióp de
        pub_wl.publish(wl)
        pub_wr.publish(wr)

if __name__ == "__main__":
    main()
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from controller import Robot, Camera
from cv_bridge import CvBridge
import numpy as np

#Variables globales de las velocidades
v = 0.0
w = 0.0

#Callback de las velocidades
def velocidad_callback(data):
    global v, w
    #Velocidades del suscriptor
    v = data.linear.x
    w = data.angular.z

#Inicializar el robot
def setup_robot():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    
    #Declarar motores
    right_motor = robot.getDevice('wr')
    left_motor = robot.getDevice('wl')

    #Declarar su posicion y velocidad
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)

    #Iniciar la camara
    camera = robot.getDevice("camera")
    camera.enable(timestep)

    #Iniciar LiDar
    lidar = robot.getDevice('lidar')
    lidar.enable(timestep)
    lidar.enablePointCloud()

    return robot, timestep, right_motor, left_motor, camera, lidar

#Publicar valores de LiDar
def publish_lidar_data(lidar):

    #Inicar mensaje tipo scan e ir llenando valores
    range_image = lidar.getRangeImage()
    msg = LaserScan()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'lidar_link'
    
    #Valores pueden cambiar dependiendo del robot
    msg.angle_min = -1.57  
    msg.angle_max = 1.57
    msg.angle_increment = 3.14 / len(range_image)

    #Valores minimos y maximos
    msg.range_min = lidar.getMinRange()
    msg.range_max = lidar.getMaxRange()
    msg.ranges = range_image

    #Publicador del mensaje del LiDar
    scan_pub.publish(msg)

def main():
    global scan_pub
    
    #Nodo
    rospy.init_node("puzzlebot")

    #Suscriptor
    rospy.Subscriber("/cmd_vel", Twist, velocidad_callback)

    #Publicadores
    pub_wl = rospy.Publisher("/wl", Float32, queue_size=10)
    pub_wr = rospy.Publisher("/wr", Float32, queue_size=10)
    image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
    scan_pub = rospy.Publisher('/scan', LaserScan, queue_size=10)

    #Imagen de opencv
    bridge = CvBridge()

    robot, timestep, right_motor, left_motor, camera, lidar = setup_robot()

    #Medidas del puzzlebot
    wheel_radius = 0.05
    distance_between_wheels = 0.19

    #10Hz
    rate = rospy.Rate(10)  

    while robot.step(timestep) != -1 and not rospy.is_shutdown():
        publish_lidar_data(lidar)

        #Ecuaciones para sacar wl y wr
        wl = (v - w * (distance_between_wheels / 2.0)) / wheel_radius
        wr = (v + w * (distance_between_wheels / 2.0)) / wheel_radius

        #Publicar las velocidades en webots
        left_motor.setVelocity(wl*6)
        right_motor.setVelocity(wr*6)

        #Publicar velocidades en ROS
        pub_wl.publish(Float32(wl))
        pub_wr.publish(Float32(wr))

        #Sacar imagen de la camara
        image = camera.getImage()
        if image:

            #Procesar la imagen
            image_array = camera.getImageArray()
            image_np = np.array(image_array, dtype=np.uint8)

            #Voltear la camara vertical 
            image_np = np.flipud(np.array(image_np, dtype=np.uint8))  
            image_ros = bridge.cv2_to_imgmsg(image_np, "rgb8")

            #Publicar la imagen en tiempo real
            image_pub.publish(image_ros)

        rate.sleep()

if __name__ == "__main__":
    try:
        #Codigo principal
        main()
    except rospy.ROSInterruptException:
        pass
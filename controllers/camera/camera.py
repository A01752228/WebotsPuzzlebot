#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from controller import Camera, Supervisor
from cv_bridge import CvBridge
import numpy as np

def main():
    # Inicializar Webots
    supervisor = Supervisor()

    # Inicializar ROS
    rospy.init_node('webots_camera_node', anonymous=True)

    # Obtener la cámara
    camera = supervisor.getDevice("camera")
    camera.enable(10)  # Habilitar la cámara y establecer la frecuencia de muestreo

    # Esperar a que la cámara se inicialice
    for _ in range(10):
        supervisor.step(32)

    # Publicador de ROS
    image_pub = rospy.Publisher('/webots/camera/image_raw', Image, queue_size=10)
    bridge = CvBridge()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # Capturar imagen de la cámara
        image = camera.getImage()
        if image:
            image_array = camera.getImageArray()
            image_np = np.array(image_array, dtype=np.uint8)
            image_np = np.flipud(np.array(image_np, dtype=np.uint8))
            image_ros = bridge.cv2_to_imgmsg(image_np, "rgb8")
            image_pub.publish(image_ros)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

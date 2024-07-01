from controller import Robot, Motor, PositionSensor
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from tf.transformations import quaternion_from_euler
import tf
import math
import numpy as np

# Parámetros del robot (ajusta estos valores según tu robot)
wheel_radius = 0.05  # Radio de la rueda (en metros)
wheel_base = 0.18    # Distancia entre las dos ruedas (en metros)

# Variables de odometría
x = 0.0
y = 0.0
theta = 0.0

# Variables para las velocidades de las ruedas
left_speed = 0.0
right_speed = 0.0

def cmd_vel_callback(msg):
    global left_speed, right_speed
    linear_velocity = msg.linear.x
    angular_velocity = msg.angular.z

    # Calcular las velocidades de las ruedas a partir de las velocidades lineales y angulares
    left_speed = (linear_velocity - angular_velocity * wheel_base / 2.0) / wheel_radius
    right_speed = (linear_velocity + angular_velocity * wheel_base / 2.0) / wheel_radius

def main():
    global x, y, theta, left_speed, right_speed

    rospy.init_node('my_robot_controller', anonymous=True)
    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)
    pub = rospy.Publisher('/pose', PoseStamped, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    br = tf.TransformBroadcaster()

    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    # Inicializar motores y encoders
    left_motor = robot.getDevice("wl")
    right_motor = robot.getDevice("wr")
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    
    left_encoder = robot.getDevice("ps_1")
    right_encoder = robot.getDevice("ps_2")
    left_encoder.enable(timestep)
    right_encoder.enable(timestep)

    prev_left_encoder = left_encoder.getValue()
    prev_right_encoder = right_encoder.getValue()

    while robot.step(timestep) != -1 and not rospy.is_shutdown():
        # Leer valores de los encoders
        current_left_encoder = left_encoder.getValue()
        current_right_encoder = right_encoder.getValue()

        # Calcular el desplazamiento de las ruedas
        delta_left = (current_left_encoder - prev_left_encoder) * wheel_radius
        delta_right = (current_right_encoder - prev_right_encoder) * wheel_radius

        # Guardar los valores de los encoders actuales para la próxima iteración
        prev_left_encoder = current_left_encoder
        prev_right_encoder = current_right_encoder

        # Calcular el cambio en la orientación del robot
        delta_theta = (delta_right - delta_left) / wheel_base

        # Calcular la distancia recorrida
        distance = (delta_right + delta_left) / 2.0

        # Verificar que los valores no sean NaN o inf
        if np.isnan(delta_theta) or np.isinf(delta_theta):
            rospy.logwarn("Detected NaN or Inf in odometry calculations")
            continue

        # Actualizar la posición y orientación del robot
        x += distance * math.cos(theta + delta_theta / 2.0)
        y += distance * math.sin(theta + delta_theta / 2.0)
        theta += delta_theta

        # Verificar que los valores no sean NaN o inf
        if np.isnan(x) or np.isinf(x) or np.isnan(y) or np.isinf(y) or np.isnan(theta) or np.isinf(theta):
            rospy.logwarn("Detected NaN or Inf in position/orientation")
            continue

        # Crear un mensaje de tipo PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"  # Cambia a tu frame de referencia si es necesario
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = 0
        quaternion = quaternion_from_euler(0, 0, theta)

        # Verificar que los valores del quaternion no sean NaN o inf
        if any(np.isnan(quaternion)) or any(np.isinf(quaternion)):
            rospy.logwarn("Detected NaN or Inf in quaternion")
            continue

        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]

        # Publicar el mensaje
        pub.publish(pose_msg)

        # Publicar la transformación del robot
        br.sendTransform((x, y, 0),
                         quaternion,
                         rospy.Time.now(),
                         "base_link",
                         "map")

        # Publicar la transformación de las ruedas
        left_wheel_quat = quaternion_from_euler(0, 0, left_speed * timestep)
        right_wheel_quat = quaternion_from_euler(0, 0, right_speed * timestep)
        
        br.sendTransform((wheel_base / 2.0, 0.09, 0),
                         left_wheel_quat,
                         rospy.Time.now(),
                         "leftWheel",
                         "base_link")

        br.sendTransform((wheel_base / 2.0, -0.09, 0),
                         right_wheel_quat,
                         rospy.Time.now(),
                         "rightWheel",
                         "base_link")

        # Establecer las velocidades de los motores
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

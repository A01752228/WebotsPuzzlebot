from controller import Robot
import rospy
from sensor_msgs.msg import LaserScan

class WebotsLidarNode:
    def __init__(self):
        # Inicialización del nodo
        rospy.init_node('webots_lidar_node', anonymous=True)

        # Publicación del LIDAR
        self.publisher = rospy.Publisher('/webots/scan', LaserScan, queue_size=10)
        
        # Creación del Robot
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # Inicializacion del LiDAR
        self.lidar = self.robot.getDevice('lidar')
        self.lidar.enable(self.timestep)
        self.lidar.enablePointCloud()
        self.rate = rospy.Rate(1.0 / (self.timestep / 1000.0))

    def run(self):
        while not rospy.is_shutdown() and self.robot.step(self.timestep) != -1:
            #Definición de los parametros del LIDAR
            range_image = self.lidar.getRangeImage()
            msg = LaserScan()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'lidar_link'
            
            msg.angle_min = -1.57
            msg.angle_max = 1.57
            msg.angle_increment = 3.14 / len(range_image)
            msg.range_min = self.lidar.getMinRange()
            msg.range_max = self.lidar.getMaxRange()
            msg.ranges = range_image
            self.publisher.publish(msg)
            self.rate.sleep()

if __name__ == "__main__":
    node = WebotsLidarNode()
    node.run()

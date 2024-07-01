from controller import Robot,Keyboard
from controller import Camera

TIME_STEP = 150
MAX_VELOCITY = 3.14

# Creación del Robot
robot = Robot()
keyboard = Keyboard()
keyboard.enable(TIME_STEP)


wheels = []
wheelsName = ["wl","wr"]

# Declaración de la camara
camera = robot.getDevice("camera")
camera.enable(10)

# Declaración del LIDAR 
lidar = robot.getDevice('lidar')
lidar.enable(TIME_STEP)
lidar.enablePointCloud()

# Lectura de las llantas de una lista
for i in range(2):
    wheels.append(robot.getDevice(wheelsName[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)

while robot.step(TIME_STEP) != -1:
    #Imprecion del rango del LIDAR
    range_image = lidar.getRangeImage()
    print(range_image[0:5])

    # Declaracion del movimiento teleoperado
    key = keyboard.getKey()
    leftspeed = 0.0
    rightspeed = 0.0

    if(key==keyboard.LEFT):
        leftspeed= -MAX_VELOCITY
        rightspeed = MAX_VELOCITY
    elif(key==keyboard.RIGHT):
        leftspeed= MAX_VELOCITY
        rightspeed = -MAX_VELOCITY
    elif(key==keyboard.UP):
        leftspeed= MAX_VELOCITY
        rightspeed = MAX_VELOCITY
    elif(key==keyboard.DOWN):
        leftspeed= -MAX_VELOCITY
        rightspeed = -MAX_VELOCITY
    else:
        leftspeed = 0.0
        rightspeed = 0.0

    #Instanciacion de las velocidades
    wheels[0].setVelocity(leftspeed)
    wheels[1].setVelocity(rightspeed)
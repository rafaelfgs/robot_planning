#!/usr/bin/env python
import rospy
import numpy as np
import time
import sys
from math import cos, sin, pi
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan



# Frequencia de atualizacao do programa
freq_stage = 20.0

# Parametros da leminiscata (centro e raio em 'x' e 'y')
cx = float(sys.argv[1])
cy = float(sys.argv[2])
rx = float(sys.argv[3])
ry = float(sys.argv[4])

# Frequencia da leminiscata
freq_curve = float(sys.argv[5])

# Ponto de controle do feedback linearization
d = 0.1

# Parametro de controle para a convergencia (k>0)
k = 1.0

# Inicia a variavel da pose do robo
x = 0.0
y = 0.0
th = 0.0



# Rotina callback para a obtencao da pose do robo
def callback_pose(data):

    global x, y, th

    # Dados de entrada da pose do robo
    x_q = data.pose.pose.orientation.x
    y_q = data.pose.pose.orientation.y
    z_q = data.pose.pose.orientation.z
    w_q = data.pose.pose.orientation.w
    euler = euler_from_quaternion([x_q, y_q, z_q, w_q])

    # Pose atual do robo
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    th = euler[2]



# Rotina para definicao da posicao e velocidade do ponto desejado
def lemniscate(t):

    # Posicao atual da leminiscata
    xd = rx * sin(2.0*pi*freq_curve*t) + cx
    yd = ry * sin(4.0*pi*freq_curve*t) + cy

    # Velocidade atual da leminiscata
    dxd = rx * 2.0*pi*freq_curve * cos(2.0*pi*freq_curve*t)
    dyd = ry * 4.0*pi*freq_curve * cos(4.0*pi*freq_curve*t)

    return xd, yd, dxd, dyd



# Rotina para definicao do sinal de controle
def control_signal(xd, yd, dxd, dyd):

    Ux = k * (xd-x) + dxd
    Uy = k * (yd-y) + dyd

    return Ux, Uy



# Rotina para definicao das velocidades
def feedback_linearization(Ux, Uy):

    # Velocidades linear e angular
    v = cos(th) * Ux + sin(th) * Uy
    w = -(sin(th) * Ux) / d + (cos(th) * Uy) / d

    return v, w



# Rotina primaria
def follow_curve():

    # Inicializa o no
    rospy.init_node("follow_curve_node")

    # Declaracao do topico para pose atual
    rospy.Subscriber("/base_pose_ground_truth", Odometry, callback_pose)

    # Declaracao do topico para comando de velocidade
    pub_stage = rospy.Publisher("/cmd_vel", Twist, queue_size=1) 

    # Inicia a variavel de velocidade
    vel = Twist()

    # Variavel de controle da frequencia de execucao do no
    rate = rospy.Rate(freq_stage)

    # Tempo inicial do programa
    ti = time.time()

    # Enquanto o programa nao for interrompido
    while not rospy.is_shutdown():

        # Tempo atual do programa
        t = time.time() - ti

        # Determina a posicao e a velocidade atual da leminiscata
        xd, yd, dxd, dyd = lemniscate(t)

        # Determina o sinal de controle
        Ux, Uy = control_signal(xd, yd, dxd, dyd)

        # Calcula as velocidades
        vel.linear.x, vel.angular.z = feedback_linearization(Ux, Uy)

        # Publica as velocidades
        pub_stage.publish(vel)

        # Espera o tempo necessario para a frequencia desejada
        rate.sleep()



# Funcao inicial
if __name__ == '__main__':
    try:
        follow_curve()
    except rospy.ROSInterruptException:
        pass

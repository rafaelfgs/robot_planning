#!/usr/bin/env python
import rospy
import numpy as np
import time
import sys
import cv2
from copy import copy
from math import sqrt, pi, sin, cos, tan, asin, acos, atan2, tanh
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan



# Frequencia de atualizacao do programa
freq_stage = 20.0

# Ponto de controle do feedback linearization
d = 0.1

# Parametro de controle para a convergencia (k>0)
k = 1.0

# Tolerancia para o ponto objetivo
dist_tol = 0.3

# Distancias maxima para ignorar o obstaculo
laser_max = 5.0

# Ganhos do potencial de atracao e repulsao
zeta = 1.0
nu = 1.0

# Status para o criterio de parada
stop = True

# Inicia a variavel da pose do robo
x = 0.0
y = 0.0
th = 0.0

# Inicia a variavel do ponto objetivo
x_goal = 0.0
y_goal = 0.0



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



# Rotina callback para a obtencao dos dados do laser
def callback_laser(data):

    global d_obst, th_obst, laser_max

    # Dados de entrada do laser
    num = len(data.ranges)
    th_min = data.angle_min
    th_max = data.angle_max
    laser_max = data.range_max

    # Distancia e angulo de cada amostra obtida do laser
    d_obst = np.array(data.ranges)
    th_obst = np.linspace(th_min,th_max,num)



# Rotina callback para a obtencao do ponto objetivo
def callback_goal(data):

    global x_goal, y_goal

    # Posicao do ponto objetivo
    x_goal = data.pose.pose.position.x
    y_goal = data.pose.pose.position.y



# Rotina para definicao do potencial atrativo
def attractive_pot():

    # Gradiente atrativo
    gradU = zeta * (np.array([x,y]) - np.array([x_goal,y_goal]))

    # Sinal de controle em x e y
    Ux = -tanh(gradU[0])
    Uy = -tanh(gradU[1])

    return Ux, Uy



# Rotina para definicao do potencial repulsivo
def repulsive_pot():

    # Variaveis do obstaculo mais proximo
    i_min = np.argmin(d_obst)
    d_min = d_obst[i_min]
    th_min = th_obst[i_min]

    # Gradiente do obstaculo
    gradD = -np.array([cos(th+th_min),sin(th+th_min)])

    # Gradiente repulsivo
    if d_min <= laser_max:
        gradU = nu * (1.0/laser_max - 1.0/d_min) * 1.0/d_min**2 * gradD
    else:
        gradU = np.array([0.0, 0.0])

    # Sinal de controle em x e y
    Ux = -tanh(gradU[0])
    Uy = -tanh(gradU[1])

    return Ux, Uy



# Rotina para definicao das velocidades
def feedback_linearization(Ux, Uy):

    # Velocidades linear e angular
    v = cos(th) * Ux + sin(th) * Uy
    w = -(sin(th) * Ux) / d + (cos(th) * Uy) / d

    return v, w



# Rotina primaria
def potfunc():

    # Inicializa o no
    rospy.init_node("potfunc_node")

    # Declaracao do topico para pose atual do robo
    rospy.Subscriber("/robot_0/base_pose_ground_truth", Odometry, callback_pose)

    # Declaracao do topico para as variaveis do laser
    rospy.Subscriber("/robot_0/base_scan", LaserScan, callback_laser)

    # Declaracao do topico para pose atual do ponto objetivo
    rospy.Subscriber("/robot_1/base_pose_ground_truth", Odometry, callback_goal)

    # Declaracao do topico para comando de velocidade do robo
    pub_stage = rospy.Publisher("/robot_0/cmd_vel", Twist, queue_size=1) 

    # Inicia a variavel de velocidade
    vel = Twist()

    # Variavel de controle da frequencia de execucao do no
    rate = rospy.Rate(freq_stage)

    # Tempo para realizar os callbacks
    time.sleep(0.5)

    # Status para o criterio de parada
    global stop

    # Enquanto o programa nao for interrompido
    while not rospy.is_shutdown():

        # Se estiver longe do objetivo
        if sqrt((x-x_goal)**2+(y-y_goal)**2) > dist_tol:

            # Imprime uma mensagem na tela
            if stop:
                print "Movendo-se para o ponto objetivo"
                stop = False

            # Determina o potencial atrativo
            Uattx, Uatty = attractive_pot()

            # Determina o potencial repulsivo
            Urepx, Urepy = repulsive_pot()

            # Calcula a funcao de potencial total
            Ux = Uattx + Urepx
            Uy = Uatty + Urepy

        # Se estiver proximo do objetivo
        else:

            # Imprime uma mensagem na tela
            if not stop:
                print "Ponto objetivo alcancado"
                stop = True

            # Forca a parada do robo
            Ux = 0.0
            Uy = 0.0
 

        # Calcula as velocidades
        vel.linear.x, vel.angular.z = feedback_linearization(Ux, Uy)

        # Publica as velocidades
        pub_stage.publish(vel)

        # Espera o tempo necessario para a frequencia desejada
        rate.sleep()



# Funcao inicial
if __name__ == '__main__':
    try:
        potfunc()
    except rospy.ROSInterruptException:
        pass

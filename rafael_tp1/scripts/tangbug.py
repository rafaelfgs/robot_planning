#!/usr/bin/env python
import rospy
import numpy as np
import time
import sys
from math import sqrt, pi, sin, cos, tan, asin, acos, atan2
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan



# Posicao final desejada para o robo
x_goal = float(sys.argv[1])
y_goal = float(sys.argv[2])

# Variaveis de controle
freq_stage = 20.0  # Frequencia de atualizacao do programa
prec = 2           # Precisao das aproximacoes
tol = 0.5          # Tolerancia da distancia ao ponto objetivo
d = 0.1            # Ponto de controle do feedback linearization
k = 1.0            # Parametro de controle para a convergencia (k>0)
v_max = 1.0        # Velocidade linear maxima
w_max = 1.0        # Velocidade angular maxima
d1 = 1.0           # Distancia do obstaculo ao ponto de contorno
d2 = 3.0           # Distancia do robo ao ponto de contorno
th_open = pi/4.0   # Distancia do robo ao ponto de contorno



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



# Rotina de obtencao da menor distancia do obstaculo no caminho do desejado
def get_obstacle_goal():

    # Angulo do ponto desejado
    th_goal = atan2(y_goal-y, x_goal-x) - th

    # Faixa do angulo max e min para o desejado
    idx = np.array([0,0])
    idx[0] = max(0,np.argmax(th_obst > th_goal - th_open/2.0))
    idx[1] = min(len(th_obst),np.argmax(th_obst > th_goal + th_open/2.0)-1)

    # Menor distancia do obstaculo no caminho para o desejado
    if idx[0] < idx[1]:
        d_goal = min(d_obst[idx[0]:idx[1]])
    else:
        d_goal = laser_max

    return d_goal



# Rotina de obtencao do ponto do obstaculo com menor distancia ao desejado
def get_obstacle_min():

    # Coordenadas dos pontos iniciadas como infinitas
    x_obst = np.full(len(d_obst),np.inf)
    y_obst = np.full(len(d_obst),np.inf)

    # Indices onde o laser atingiu a distancia maxima
    idx = d_obst < laser_max

    # Coordenadas x e y dos pontos do obstaculo
    x_obst[idx] = x + d_obst[idx] * np.cos(th + th_obst[idx])
    y_obst[idx] = y + d_obst[idx] * np.sin(th + th_obst[idx])

    # Distancia dos pontos ate o desejado
    d_obst_goal = np.sqrt((x_goal-x_obst)**2 + (y_goal-y_obst)**2)

    # Variaveis em relacao ao ponto do obstaculo de minimia distancia
    idx = np.argmin(d_obst_goal)
    x_min = x_obst[idx]
    y_min = y_obst[idx]
    d_min = round(d_obst_goal[idx],prec)
    th_min = th_obst[idx]

    return x_min, y_min, d_min, th_min



# Rotina de obtencao do ponto desejado na descontinuidade
def set_xy_descont(x_min, y_min, th_min):

    # Menor angulo ao obstaculo
    th_obst_min = th_obst[np.argmin(d_obst)]

    # Para o obstaculo a esquerda
    if th_obst_min < th_min:

        # Ponto de descontinuidade deslocado a +90
        x_desc = x_min + d1 * cos(th + th_min + pi/2.0)
        y_desc = y_min + d1 * sin(th + th_min + pi/2.0)

    # Para o obstaculo a direita
    else:

        # Ponto de descontinuidade deslocado a -90
        x_desc = x_min + d1 * cos(th + th_min - pi/2.0)
        y_desc = y_min + d1 * sin(th + th_min - pi/2.0)

    return x_desc, y_desc



# Rotina de obtencao do ponto desejado durante o contorno do obstaculo
def set_xy_contour():

    # Menores distancias a frente, a direita e a esquerda
    d_front = min(d_obst[np.argmax(th_obst > -th_open/4.0):np.argmax(th_obst > th_open/4.0)-1])
    d_right = min(d_obst[np.argmax(th_obst > -th_open/4.0-pi/2.0):np.argmax(th_obst > th_open/4.0-pi/2.0)-1])
    d_left = min(d_obst[np.argmax(th_obst > -th_open/4.0+pi/2.0):np.argmax(th_obst > th_open/4.0+pi/2.0)-1])

    # Menor angulo ao obstaculo
    th_obst_min = th_obst[np.argmin(d_obst)]

    # Angulo do ponto de contorno
    th_cont = acos((min(d_obst)-d1)/d2)

    # Para um obstaculo a frente e a direita
    if d_front < 2.0*d1 and d_right < d_left and d_right < 2.0*d1:

        # Ponto de contorno deslocado a +90
        x_cont = x + d1 * cos(th + th_obst_min + th_cont)
        y_cont = y + d1 * sin(th + th_obst_min + th_cont)

    # Para um obstaculo a frente e a esquerda
    elif d_front < 2.0*d1 and d_left < d_right and d_left < 2.0*d1:

        # Ponto de contorno deslocado a +90
        x_cont = x + d1 * cos(th + th_obst_min - th_cont)
        y_cont = y + d1 * sin(th + th_obst_min - th_cont)

    # Para o obstaculo somente a esquerda
    elif th_obst_min < 0.0:

        # Ponto de contorno deslocado a +90
        x_cont = x + d1 * cos(th + th_obst_min + th_cont)
        y_cont = y + d1 * sin(th + th_obst_min + th_cont)

    # Para o obstaculo somente a direita
    else:

        # Ponto de contorno deslocado a +90
        x_cont = x + d1 * cos(th + th_obst_min - th_cont)
        y_cont = y + d1 * sin(th + th_obst_min - th_cont)

    return x_cont, y_cont



def get_current_goal(mode, hit_point, d_min):

    # Menor distancia do obstaculo no caminho para o desejado
    d_goal = get_obstacle_goal()

    # Posicao, angulo e distancia do ponto de descontinuidade
    d_min[1] = min(d_min)
    x_min, y_min, d_min[0], th_min = get_obstacle_min()

    # Para um ponto atual muito proximo ao objetivo final
    if sqrt((x-x_goal)**2 + (y-y_goal)**2) < tol:
        mode[1] = 4
        xd, yd = x_goal, y_goal

    # Para um caminho sem obstaculos e fora do modo de contorno
    elif d_goal >= 2.0*d1 and mode[0] != 3:

        # Restauracao das variaveis relacionadas ao modo de contorno
        mode[1] = 0
        hit_point = np.array([np.inf,np.inf])

        # Modo principal de operacao para ir para o objetivo final
        mode[0] = 1
        xd, yd = x_goal, y_goal

    # Para um obstaculo proximo ou no modo de contorno
    else:

        # Distancia do ponto atual ao ponto inicial de contorno
        hit_dist = sqrt((x-hit_point[0])**2 + (y-hit_point[1])**2)

        # Modo auxiliar de operacao para determinar o hit-point
        if mode[1] == 0:
            mode[1] = 1
            hit_point = np.array([x, y])

        # Modo auxiliar de operacao para sair do hit-point
        elif mode[1] == 1 and hit_dist > 2.0*d1:
            mode[1] = 2

        # Modo auxiliar de operacao para retornar ao hit-point
        elif mode[1] == 2 and hit_dist < 2.0*tol:
            mode[1] = 3

        # Para atualizacoes na distancia minima
        if d_min[0] < d_min[1] or min(d_obst) > 2.0*d1:

            # Modo principal de operacao para ir para a descontinuidade
            mode[0] = 2
            xd, yd = set_xy_descont(x_min, y_min, th_min)

        # Para permanencia sa distancia minima e proximo ao obstaculo
        else:

            # Modo principal de operacao para contornar o obstaculo
            mode[0] = 3
            xd, yd = set_xy_contour()

    return xd, yd, mode, hit_point, d_min



# Rotina para exibicao do modo atual
def print_mode(mode, mode_old):

    # Exibe o modo de operacao principal
    if mode[0] != mode_old[0]:
        if mode[0] == 1:
            print "Nao ha obstaculos no caminho. Indo para o objetivo final."
        elif mode[0] == 2:
            print "Obstaculo encontrado. Indo para o ponto de descontinuidade."
        elif mode[0] == 3:
            print "Minimo local encontrado. Contornando o obstaculo."

    # Exibe os modos auxiliares de finalizacao do programa
    if mode[1] != mode_old[1]:
        if mode[1] == 3:
            print "Ponto inicial de contorno reencontrado. Nao e possivel alcancar o ponto objetivo."
        elif mode[1] == 4:
            print "Ponto objetivo alcancado."



# Rotina para definicao do sinal de controle
def control_signal(xd, yd, dxd, dyd):

    # Sinais de controle em x e y
    Ux = k * (xd-x) + dxd
    Uy = k * (yd-y) + dyd

    return Ux, Uy



# Rotina para definicao das velocidades
def feedback_linearization(Ux, Uy):

    # Velocidades linear e angular
    v = cos(th) * Ux + sin(th) * Uy
    w = -(sin(th) * Ux) / d + (cos(th) * Uy) / d

    # Saturacao das velocidades
    if abs(v) > v_max:
        v = np.sign(v) * v_max
    if abs(w) > w_max:
        w = np.sign(w) * w_max

    return v, w



# Rotina primaria
def tangbug():

    # No principal
    rospy.init_node("tangbug_node")

    # Declaracao do topico para pose atual
    rospy.Subscriber("/base_pose_ground_truth", Odometry, callback_pose)

    # Declaracao do topico para as variaveis do laser
    rospy.Subscriber("/base_scan", LaserScan, callback_laser)

    # Declaracao do topico para comando de velocidade
    pub_stage = rospy.Publisher("/cmd_vel", Twist, queue_size=1) 

    # Variavel de velocidade a ser publicada
    vel = Twist()

    # Variavel de controle da frequencia de execucao do no
    rate = rospy.Rate(freq_stage)

    # Inicializacao das variaveis necessarias
    mode = np.array([0,0])
    hit_point = np.array([np.inf,np.inf])
    d_min = np.array([np.inf,np.inf])

    # Tempo para realizar os callbacks
    time.sleep(0.5)

    # Enquanto nao interromper e nao entrar nos modos de finalizar
    while not rospy.is_shutdown() and mode[1] < 3:

        # Salva o ultimo modo de operacao
        mode_old = mode+0

        # Modo de operacao e o ponto objetivo atual
        xd, yd, mode, hit_point, d_min = get_current_goal(mode, hit_point, d_min)

        # Exibicao do modo de operacao
        print_mode(mode, mode_old)

        # Calculo do sinal de controle
        Ux, Uy = control_signal(xd, yd, 0.0, 0.0)

        # Calculo das velocidades
        vel.linear.x, vel.angular.z = feedback_linearization(Ux, Uy)

        # Publicacao das velocidades
        pub_stage.publish(vel)

        # Tempo necessario para a frequencia desejada
        rate.sleep()



# Funcao inicial
if __name__ == '__main__':
    try:
        tangbug()
    except rospy.ROSInterruptException:
        pass

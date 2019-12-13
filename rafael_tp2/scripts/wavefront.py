#!/usr/bin/env python
import rospy
import numpy as np
import time
import sys
import cv2
from copy import copy
from math import sqrt, pi, sin, cos, tan, asin, acos, atan2, ceil, floor
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan



# Arquivo do mapa utilizado
img_path = sys.argv[1]

# Tamanho [x,y] do mapa
map_size = np.array([float(sys.argv[2]),float(sys.argv[3])])

# Resolucao da discretizacao do mapa [x,y]
resolution_map = np.array([int(sys.argv[4]),int(sys.argv[5])])

# Frequencia de atualizacao do programa
freq_stage = 20.0

# Ponto de controle do feedback linearization
d = 0.1

# Parametro de controle para a convergencia (k>0)
k = 1.0

# Status para o criterio de parada
stop = np.array([True,True])

# Tolerancia para o ponto objetivo
dist_tol = np.mean(map_size/resolution_map)/10.0

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



# Rotina callback para a obtencao do ponto objetivo
def callback_goal(data):

    global x_goal, y_goal

    # Posicao do ponto objetivo
    x_goal = data.pose.pose.position.x
    y_goal = data.pose.pose.position.y



# Rotina para a discretizacao do mapa identificando os obstaculos
def set_grid_map():

    # Inicial o grid com zeros
    grid = np.zeros((resolution_map[1],resolution_map[0]))
    
    # Carrega a imagem do arquivo especificado
    img_cv = cv2.imread(img_path, 0)
    img = np.array(img_cv) > 127

    # Determina os tamanhos e a relacao entre as discretizacoes
    img_size = np.shape(img)
    ratio = np.array([float(resolution_map[1])/img_size[0],
                      float(resolution_map[0])/img_size[1]])

    # Percorre a imagem e marca como inf os pontos correspondes no grid
    for i in range(img_size[0]):
        for j in range(img_size[1]):
            if not img[i,j]:
                i_min = int(i*ratio[0])
                i_max = int(ceil((i+1)*ratio[0]))
                j_min = int(j*ratio[1])
                j_max = int(ceil((j+1)*ratio[1]))
                grid[i_min:i_max,j_min:j_max] = np.inf

    return grid



# Rotina do metodo wavefront no grid
def apply_wavefront(grid):

    # Limpa o grid criado anteriormente
    grid[grid!=np.inf] = 0.0

    # Determina a relacao entre o mapa e a discretizacao
    ratio = np.array([float(resolution_map[0])/map_size[0],
                      float(resolution_map[1])/map_size[1]])

    # Encontra a posicao do ponto objetivo discretizado
    i_goal = int( ((map_size[1]/2) - y_goal) * ratio[1] )
    j_goal = int( ((map_size[0]/2) + x_goal) * ratio[0] )

    # Inicia os vetores com os indices
    curr_idx = np.zeros((sum(resolution_map),2)).astype(int)
    last_idx = np.zeros((sum(resolution_map),2)).astype(int)

    # Valor atual do wavefront
    value = 2.0
    curr_k = 0

    # Marca o ponto objetivo no grid
    if 0 <= i_goal < resolution_map[1] and 0 <= j_goal < resolution_map[0]:
        if grid[i_goal,j_goal] != np.inf:
            grid[i_goal,j_goal] = value
            curr_idx[0] = [i_goal,j_goal]
            curr_k = 1

    # Enquanto tiver algum vizinho
    while curr_k > 0:

        # Prapara as variaveis de estado
        value = value + 1.0
        last_k = curr_k
        last_idx = curr_idx
        curr_k = 0
        curr_idx = np.zeros((sum(resolution_map),2)).astype(int)

        # Percorre os pontos passados
        for i in range(last_k):

            # Determina os vizinhos
            u_idx = [max(0,last_idx[i,0]-1),last_idx[i,1]]
            l_idx = [last_idx[i,0],max(0,last_idx[i,1]-1)]
            r_idx = [last_idx[i,0],min(resolution_map[0]-1,last_idx[i,1]+1)]
            d_idx = [min(resolution_map[1]+1,last_idx[i,0]+1),last_idx[i,1]]

            # Marca o ponto de cima do atual no grid
            if grid[u_idx[0],u_idx[1]] == 0:
                grid[u_idx[0],u_idx[1]] = value
                curr_idx[curr_k] = [u_idx[0],u_idx[1]]
                curr_k = curr_k + 1

            # Marca o ponto a esquerda do atual no grid
            if grid[l_idx[0],l_idx[1]] == 0:
                grid[l_idx[0],l_idx[1]] = value
                curr_idx[curr_k] = [l_idx[0],l_idx[1]]
                curr_k = curr_k + 1

            # Marca o ponto a direita do atual no grid
            if grid[r_idx[0],r_idx[1]] == 0:
                grid[r_idx[0],r_idx[1]] = value
                curr_idx[curr_k] = [r_idx[0],r_idx[1]]
                curr_k = curr_k + 1

            # Marca o ponto de baixo do atual no grid
            if grid[d_idx[0],d_idx[1]] == 0:
                grid[d_idx[0],d_idx[1]] = value
                curr_idx[curr_k] = [d_idx[0],d_idx[1]]
                curr_k = curr_k + 1

    return grid



# Rotina para definicao da posicao e velocidade do ponto desejado
def set_next_point(grid):

    # Status para as impressoes na tela
    global stop

    # Determina a relacao entre o mapa e a discretizacao
    ratio = np.array([float(resolution_map[0])/map_size[0],
                      float(resolution_map[1])/map_size[1]])

    # Encontra a posicao do ponto atual discretizado
    i_curr = int( ((map_size[1]/2) - y) * ratio[1] )
    j_curr = int( ((map_size[0]/2) + x) * ratio[0] )

    # Se houver solucao para o ponto atual
    if grid[i_curr,j_curr] > 0.0:

        # Exibe uma mensagem na tela
        if stop[0]:
            print "Movendo-se para o ponto objetivo"
        stop[0] = False

        # Determina os vizinhos
        u_idx = [max(0,i_curr-1),j_curr]
        l_idx = [i_curr,max(0,j_curr-1)]
        r_idx = [i_curr,min(resolution_map[0]-1,j_curr+1)]
        d_idx = [min(resolution_map[1]-1,i_curr+1),j_curr]

        # Determina o valor e o ponto atual do wavefront
        min_val = grid[i_curr,j_curr]
        min_pos = np.array([i_curr,j_curr])

        # Verifica se ha minimizacao com o ponto de cima
        if grid[u_idx[0],u_idx[1]] < min_val:
            min_val = grid[u_idx[0],u_idx[1]]
            min_pos = u_idx

        # Verifica se ha minimizacao com o ponto da esquerda
        if grid[l_idx[0],l_idx[1]] < min_val:
            min_val = grid[l_idx[0],l_idx[1]]
            min_pos = l_idx

        # Verifica se ha minimizacao com o ponto da direita
        if grid[r_idx[0],r_idx[1]] < min_val:
            min_val = grid[r_idx[0],r_idx[1]]
            min_pos = r_idx

        # Verifica se ha minimizacao com o ponto de baixo
        if grid[d_idx[0],d_idx[1]] < min_val:
            min_val = grid[d_idx[0],d_idx[1]]
            min_pos = d_idx

        # Proxima posicao desejada do robo
        xd = (min_pos[1]+0.5)/ratio[0] - (map_size[0]/2)
        yd = -(min_pos[0]+0.5)/ratio[1] + (map_size[1]/2)

    else:

        # Determina o ponto desejado de forma a parar o robo
        xd, yd = x, y

        # Exibe uma mensagem na tela
        if not stop[0] and not stop[1]:
            stop[0] = True
            print "Nao ha solucao para o problema"

    # Velocidade da posicao desejada
    dxd = 0.0
    dyd = 0.0

    return xd, yd, dxd, dyd



# Rotina para definicao do sinal de controle
def control_signal(xd, yd, dxd, dyd):

    # Sinal de controle em x e y
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
def wavefront():

    # Inicializa o no
    rospy.init_node("wavefront_node")

    # Declaracao do topico para pose atual do ponto objetivo
    rospy.Subscriber("/robot_1/base_pose_ground_truth", Odometry, callback_goal)

    # Declaracao do topico para pose atual do robo
    rospy.Subscriber("/robot_0/base_pose_ground_truth", Odometry, callback_pose)

    # Declaracao do topico para comando de velocidade do robo
    pub_stage = rospy.Publisher("/robot_0/cmd_vel", Twist, queue_size=1) 

    # Inicia a variavel de velocidade
    vel = Twist()

    # Variavel de controle da frequencia de execucao do no
    rate = rospy.Rate(freq_stage)

    # Tempo para realizar os callbacks
    time.sleep(0.5)

    # Determina o grid do mapa com obstaculos
    grid = set_grid_map()

    # Status para as impressoes na tela
    global stop

    # Inicia a posicao do ponto objetivo anterior
    x_goal_old = np.inf
    y_goal_old = np.inf

    # Enquanto o programa nao for interrompido
    while not rospy.is_shutdown():

        # Realiza o metodo wavefront no grid
        if x_goal != x_goal_old or y_goal != y_goal_old:
            t = time.time()
            grid = apply_wavefront(grid)
            print time.time() - t
            x_goal_old = x_goal
            y_goal_old = y_goal

        # Determina a posicao desejada e sua derivada
        xd, yd, dxd, dyd = set_next_point(grid)

        # Verifica se alcancou o ponto objetivo
        if sqrt((x-xd)**2+(y-yd)**2) >= dist_tol:

            # Exibe uma mensagem na tela
            if stop[1]:
                print "Movendo-se para o ponto objetivo"
            stop[1] = False

        else:

            # Determina o ponto desejado de forma a parar o robo
            xd, yd = x, y

            # Exibe uma mensagem na tela
            if not stop[0] and not stop[1]:
                stop[1] = True
                print "Ponto objetivo alcancado"

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
        wavefront()
    except rospy.ROSInterruptException:
        pass

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
stop = True

# Tolerancia para o ponto objetivo
dist_tol = 0.1

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
def set_map():

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
        sys.stdout.write("\rCreating map grid: %3.0f%%" % (100*float(i)/float(img_size[0])))
        for j in range(img_size[1]):
            if not img[i,j]:
                i_min = int(i*ratio[0])
                i_max = int(ceil((i+1)*ratio[0]))
                j_min = int(j*ratio[1])
                j_max = int(ceil((j+1)*ratio[1]))
                grid[i_min:i_max,j_min:j_max] = np.inf
    sys.stdout.write("\n")
    return grid



# Rotina para a discretizacao do mapa com a heuristica de manhattan
def apply_manhattan(grid_obst):

    # Inicial o grid com zeros
    grid = np.zeros((resolution_map[1],resolution_map[0]))

    # Determina a relacao entre o mapa e a discretizacao
    ratio = np.array([float(resolution_map[0])/map_size[0],
                      float(resolution_map[1])/map_size[1]])

    # Encontra a posicao do ponto objetivo discretizado
    i_goal = int( ((map_size[1]/2) - y_goal) * ratio[1] )
    j_goal = int( ((map_size[0]/2) + x_goal) * ratio[0] )

    # Se o ponto objetivo esta dentro da discretizacao
    if 0 <= i_goal < resolution_map[1] and 0 <= j_goal < resolution_map[0]:

        # Determina o valor da distancia de manhattan dos pontos
        for i in range(resolution_map[1]):
            for j in range(resolution_map[0]):
                grid[i,j] = abs(i-i_goal) + abs(j-j_goal) + grid_obst[i,j]

    return grid



# Rotina do metodo A estrela no grid
def apply_star(grid_obst):

    # Cria o grid de saida com as posicoes anteriores
    grid = np.inf + np.ones((resolution_map[1],resolution_map[0], 2))

    # Determina os grids de F, G e H com a heuristica de manhattan
    grid_f = np.full((resolution_map[1],resolution_map[0]), np.inf)
    grid_g = copy(grid_obst)
    grid_h = apply_manhattan(grid_obst)

    # Determina a relacao entre o mapa e a discretizacao
    ratio = np.array([float(resolution_map[0])/map_size[0],
                      float(resolution_map[1])/map_size[1]])

    # Encontra a posicao do ponto objetivo discretizado
    i_goal = int( ((map_size[1]/2) - y_goal) * ratio[1] )
    j_goal = int( ((map_size[0]/2) + x_goal) * ratio[0] )

    # Se o ponto objetivo esta dentro do espaco livre
    if 0 <= i_goal < resolution_map[1] and \
       0 <= j_goal < resolution_map[0] and \
       grid_obst[i_goal,j_goal] != np.inf:

        # Encontra a posicao do ponto atual discretizado
        i_curr = int( ((map_size[1]/2) - y) * ratio[1] )
        j_curr = int( ((map_size[0]/2) + x) * ratio[0] )

        # Determina os valores dos grids do ponto inicial
        grid[i_curr,j_curr] = [-1,-1]
        grid_g[i_curr,j_curr] = 1.0
        grid_f[i_curr,j_curr] = grid_g[i_curr,j_curr] + \
                                grid_h[i_curr,j_curr]

        # Enquanto nao chegar ao ponto objetivo
        while np.any(grid[i_goal,j_goal] == np.inf):

            # Encontra as posicoes do menores valores de F
            k = np.where(grid_f == np.min(grid_f))

            # Percore os pontos de menor valor de F
            for i in range(len(k[0])):

                # Determina os vizinhos
                up = [max(0,k[0][i]-1),k[1][i]]
                lf = [k[0][i],max(0,k[1][i]-1)]
                rg = [k[0][i],min(resolution_map[0]-1,k[1][i]+1)]
                dw = [min(resolution_map[1]-1,k[0][i]+1),k[1][i]]

                # Determina os valores dos grids para o vizinho de cima
                if grid_g[up[0],up[1]] == 0:
                    grid[up[0],up[1]] = [k[0][i],k[1][i]]
                    grid_g[up[0],up[1]] = grid_g[k[0][i],k[1][i]] + \
                                          abs(k[0][i]-up[0]) + \
                                          abs(k[1][i]-up[1])
                    grid_f[up[0],up[1]] = grid_g[up[0],up[1]] + \
                                          grid_h[up[0],up[1]]

                if grid_g[lf[0],lf[1]] == 0:
                    grid[lf[0],lf[1]] = [k[0][i],k[1][i]]
                    grid_g[lf[0],lf[1]] = grid_g[k[0][i],k[1][i]] + \
                                          abs(k[0][i]-lf[0]) + \
                                          abs(k[1][i]-lf[1])
                    grid_f[lf[0],lf[1]] = grid_g[lf[0],lf[1]] + \
                                          grid_h[lf[0],lf[1]]

                if grid_g[rg[0],rg[1]] == 0:
                    grid[rg[0],rg[1]] = [k[0][i],k[1][i]]
                    grid_g[rg[0],rg[1]] = grid_g[k[0][i],k[1][i]] + \
                                          abs(k[0][i]-rg[0]) + \
                                          abs(k[1][i]-rg[1])
                    grid_f[rg[0],rg[1]] = grid_g[rg[0],rg[1]] + \
                                          grid_h[rg[0],rg[1]]

                if grid_g[dw[0],dw[1]] == 0:
                    grid[dw[0],dw[1]] = [k[0][i],k[1][i]]
                    grid_g[dw[0],dw[1]] = grid_g[k[0][i],k[1][i]] + \
                                          abs(k[0][i]-dw[0]) + \
                                          abs(k[1][i]-dw[1])
                    grid_f[dw[0],dw[1]] = grid_g[dw[0],dw[1]] + \
                                          grid_h[dw[0],dw[1]]

                grid_f[k[0][i],k[1][i]] = np.inf

    return grid



# Rotina para definicao da posicao e velocidade do ponto desejado
def set_next_point(grid):

    # Status para o criterio de parada
    global stop

    # Determina a relacao entre o mapa e a discretizacao
    ratio = np.array([float(resolution_map[0])/map_size[0],
                      float(resolution_map[1])/map_size[1]])

    # Encontra a posicao do ponto atual discretizado
    i_curr = int( ((map_size[1]/2) - y) * ratio[1] )
    j_curr = int( ((map_size[0]/2) + x) * ratio[0] )

    # Encontra a posicao do ponto objetivo discretizado
    i_goal = int( ((map_size[1]/2) - y_goal) * ratio[1] )
    j_goal = int( ((map_size[0]/2) + x_goal) * ratio[0] )

    # Se nao ha solucao, imprime uma mensagem e para o robo
    if np.any(grid[i_curr,j_curr] == np.inf):
        if not stop:
            print "Nao ha solucao para o problema"
            stop = True
        xd, yd = copy(x), copy(y)
    # Se alcancou o ponto objetivo, imprime uma mensagem e para o robo
    elif sqrt((x-x_goal)**2+(y-y_goal)**2) < dist_tol:
        if not stop:
            print "Ponto objetivo alcancado"
            stop = True
        xd, yd = copy(x), copy(y)

    # Caso contrario, imprime uma mensagem e determina o proximo ponto
    else:
        if stop:
            print "Movendo-se para o ponto objetivo"
            stop = False

        # Determina o indice desejado como o ponto objetivo
        i, j = copy(i_goal), copy(j_goal)

        # Em condicoes normais, retorna o indice para proximo ao robo
        while not ( np.all([i,j] == [i_curr,j_curr]) or \
                    np.all(grid[i,j] == [i_curr,j_curr]) or \
                    np.all(grid[i,j] == np.inf) or \
                    np.all(grid[i,j] == -1.0) ):
            i, j = grid[i,j].astype(int)

        # Determina a proxima posicao desejada do robo
        xd =  (j+0.5)/ratio[0] - map_size[0]/2
        yd = -(i+0.5)/ratio[1] + map_size[1]/2

    # Velocidade da posicao desejada
    dxd, dyd = 0.0, 0.0

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
def star():

    # Inicializa o no
    rospy.init_node("star_node")

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
    grid_obst = set_map()

    # Inicia a posicao do ponto objetivo anterior
    x_goal_old = np.inf
    y_goal_old = np.inf

    # Enquanto o programa nao for interrompido
    while not rospy.is_shutdown():

        # Realiza o metodo A estrela nos grids
        if x_goal != x_goal_old or y_goal != y_goal_old:
            grid = apply_star(grid_obst)
            x_goal_old = copy(x_goal)
            y_goal_old = copy(y_goal)

        # Determina a posicao desejada e sua derivada
        xd, yd, dxd, dyd = set_next_point(grid)

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
        star()
    except rospy.ROSInterruptException:
        pass

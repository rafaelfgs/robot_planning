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

# Tamanho do robo
robot_size = 0.5

# Ponto de controle do feedback linearization
d = 0.1

# Parametro de controle para a convergencia (k>0)
k = 1.0

# Status para o criterio de parada
stop = True

# Tolerancia para o ponto objetivo
dist_tol = 0.05

# Inicia a variavel da pose do robo
x_curr = 0.0
y_curr = 0.0
th_curr = 0.0

# Inicia a variavel do ponto objetivo
x_goal = 0.0
y_goal = 0.0

# Determina a relacao entre o mapa e a discretizacao
ratio = np.array([float(resolution_map[0])/map_size[0],
                 float(resolution_map[1])/map_size[1]])



# Rotina callback para a obtencao da pose do robo
def callback_pose(data):

    global x_curr, y_curr, th_curr

    # Dados de entrada da pose do robo
    x_q = data.pose.pose.orientation.x
    y_q = data.pose.pose.orientation.y
    z_q = data.pose.pose.orientation.z
    w_q = data.pose.pose.orientation.w
    euler = euler_from_quaternion([x_q, y_q, z_q, w_q])

    # Pose atual do robo
    x_curr = data.pose.pose.position.x
    y_curr = data.pose.pose.position.y
    th_curr = euler[2]



# Rotina callback para a obtencao do ponto objetivo
def callback_goal(data):

    global x_goal, y_goal

    # Posicao do ponto objetivo
    x_goal = data.pose.pose.position.x
    y_goal = data.pose.pose.position.y



# Rotina para a discretizacao do mapa identificando os obstaculos
def set_map():

    # Inicia a contagem do tempo
    t = time.time()

    # Inicial o grid com zeros
    grid = np.zeros((resolution_map[1],resolution_map[0]))
    
    # Carrega a imagem do arquivo especificado
    img_cv = cv2.imread(img_path, 0)
    img = np.array(img_cv) > 127

    # Determina os tamanhos e a relacao entre as discretizacoes
    img_size = np.shape(img)
    ratio_img = np.array([float(img_size[0])/resolution_map[1],
                          float(img_size[1])/resolution_map[0]])

    # Encontra o tamanho do robo na imagem
    i_robot = int(robot_size*img_size[0]/float(map_size[1])+1)
    j_robot = int(robot_size*img_size[1]/float(map_size[0])+1)


    # Percorre o grid
    for i in range(len(grid)):
        for j in range(len(grid[0])):

            # Determina os indices da imagem correspondentes
            i_min = max(0, int(i*ratio_img[0]) - int(i_robot/2))
            i_max = min(img_size[0], int(ceil((i+1)*ratio_img[0])) +
                                     int(i_robot/2)+1)
            j_min = max(0, int(j*ratio_img[1]) - int(j_robot/2))
            j_max = min(img_size[1], int(ceil((j+1)*ratio_img[1])) +
                                     int(j_robot/2)+1)

            # Caso haja algum obstaculo, marca como inf os pontos do grid
            if not np.all(img[i_min:i_max,j_min:j_max]): 
                grid[i,j] = np.inf

            # Imprime o andamento atual da funcao
            sys.stdout.write("\rCreating map grid: %3.0f%%" %
                             (100*float(i*len(grid[0])+j+1) /
                              np.prod(resolution_map)))
    sys.stdout.write("\nGrid created in %.4f segundos\n" % (time.time()-t))

    return grid



# Rotina para a discretizacao do mapa com a heuristica de manhattan
def apply_manhattan(grid_obst):

    # Inicial o grid com zeros
    grid = np.zeros((resolution_map[1],resolution_map[0]))

    # Determina o valor da distancia de manhattan dos pontos
    for i in range(resolution_map[1]):
        for j in range(resolution_map[0]):
            grid[i,j] = abs(i-i_goal) + abs(j-j_goal) + grid_obst[i,j]

    return grid



# Rotina do metodo A estrela no grid
def apply_star(grid_obst):

    # Inicia a contagem do tempo
    t = time.time()

    # Cria o grid de saida com as posicoes anteriores
    grid = np.full((resolution_map[1],resolution_map[0],2), np.inf)

    # Se o ponto objetivo esta dentro do espaco livre
    if 0 <= i_goal < resolution_map[1] and \
       0 <= j_goal < resolution_map[0] and \
       grid_obst[i_goal,j_goal] != np.inf:

        # Determina os grids de F, G e H com a heuristica de manhattan
        grid_f = np.full((resolution_map[1],resolution_map[0]), np.inf)
        grid_g = copy(grid_obst)
        grid_h = apply_manhattan(grid_obst)

        # Determina os valores dos grids do ponto inicial
        grid[i_curr,j_curr] = [-1,-1]
        grid_g[i_curr,j_curr] = 1.0
        grid_f[i_curr,j_curr] = grid_g[i_curr,j_curr] + grid_h[i_curr,j_curr]

        # Enquanto nao chegar ao ponto objetivo
        while np.any(grid[i_goal,j_goal] == np.inf):

            # Encontra as posicoes do menores valores de F
            k = np.where(grid_f == np.min(grid_f))

            # Percore os pontos de menor valor de F
            for i in range(len(k[0])):

                # Determina os vizinhos
                nb = [[max(0,k[0][i]-1),max(0,k[1][i]-1)],
                      [max(0,k[0][i]-1),k[1][i]],
                      [max(0,k[0][i]-1),min(resolution_map[0]-1,k[1][i]+1)],
                      [k[0][i],max(0,k[1][i]-1)],
                      [k[0][i],min(resolution_map[0]-1,k[1][i]+1)],
                      [min(resolution_map[1]-1,k[0][i]+1),max(0,k[1][i]-1)],
                      [min(resolution_map[1]-1,k[0][i]+1),k[1][i]],
                      [min(resolution_map[1]-1,k[0][i]+1),min(resolution_map[0]-1,k[1][i]+1)]]

                # Determina os valores dos grids para o vizinhos
                for j in range(8):
                    if grid_g[nb[j][0],nb[j][1]] == 0:
                        grid[nb[j][0],nb[j][1]] = [k[0][i],k[1][i]]
                        grid_g[nb[j][0],nb[j][1]] = grid_g[k[0][i],k[1][i]] + abs(k[0][i]-nb[j][0]) + abs(k[1][i]-nb[j][1])
                        grid_f[nb[j][0],nb[j][1]] = grid_g[nb[j][0],nb[j][1]] + grid_h[nb[j][0],nb[j][1]]

                grid_f[k[0][i],k[1][i]] = np.inf

    sys.stdout.write("A* performed in %.4f segundos\n" % (time.time()-t))

    return grid



# Rotina para definicao do indice desejado no grid
def set_next_index(grid):

    # Indice desejado no grid
    global i_next, j_next

    # Status para o criterio de parada
    global stop

    # Se nao ha solucao, imprime uma mensagem e para o robo
    if np.any(grid[i_curr,j_curr] == np.inf):
        if not stop:
            sys.stdout.write("Nao ha solucao para o problema\n")
            stop = True
        i_next, j_next = copy(i_curr), copy(j_curr)

    # Se alcancou o ponto objetivo, imprime uma mensagem e para o robo
    elif i_curr == i_goal and j_curr == j_goal:
        if not stop:
            sys.stdout.write("Ponto objetivo alcancado\n")
            stop = True
        i_next, j_next = copy(i_curr), copy(j_curr)

    # Caso contrario, imprime uma mensagem e determina o proximo ponto
    else:
        if stop:
            sys.stdout.write("Movendo-se para o ponto objetivo\n")
            stop = False

        # Determina o indice desejado como o ponto objetivo
        i_next, j_next = copy(i_goal), copy(j_goal)
        #print grid[:,:,0]
        #print grid[:,:,1]
        #print sdaf
        # Em condicoes normais, retorna o indice para proximo ao robo
        while not ( np.all([i_next,j_next] == [i_curr,j_curr]) or \
                    np.all(grid[i_next,j_next] == [i_curr,j_curr]) or \
                    np.all(grid[i_next,j_next] == np.inf) or \
                    np.all(grid[i_next,j_next] == -1.0) ):
            i_next, j_next = grid[i_next,j_next].astype(int)



# Rotina para definicao do proximo ponto desejado
def set_next_point():

    # Determina a proxima posicao desejada do robo
    x_next =  (j_next+0.5)/ratio[0] - map_size[0]/2
    y_next = -(i_next+0.5)/ratio[1] + map_size[1]/2

    # Velocidade da posicao desejada
    th_next = atan2(y_next-y_curr,x_next-x_curr)
    dx_next = cos(th_next)
    dy_next = sin(th_next)

    # Caso o robo esteja proximo ao ponto, para o robo
    if sqrt((x_curr-x_next)**2+(y_curr-y_next)**2) <= dist_tol:
        x_next = copy(x_curr)
        y_next = copy(y_curr)
        dx_next, dy_next = 0.0, 0.0

    return x_next, y_next, dx_next, dy_next



# Rotina para definicao do sinal de controle
def control_signal(x_next, y_next, dx_next, dy_next):

    # Sinal de controle em x e y
    Ux = k * (x_next-x_curr) + dx_next
    Uy = k * (y_next-y_curr) + dy_next

    # Saturacao do sinal de controle
    if Ux > 1: Ux = 1
    if Uy > 1: Uy = 1

    return Ux, Uy



# Rotina para definicao das velocidades
def feedback_linearization(Ux, Uy):

    # Velocidades linear e angular
    v = cos(th_curr) * Ux + sin(th_curr) * Uy
    w = -(sin(th_curr) * Ux) / d + (cos(th_curr) * Uy) / d

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
    pub = rospy.Publisher("/robot_0/cmd_vel", Twist, queue_size=1) 

    # Variavel de controle da frequencia de execucao do no
    rate = rospy.Rate(freq_stage)

    # Tempo para realizar os callbacks
    time.sleep(0.5)

    # Inicia a variavel de velocidade
    vel = Twist()

    # Determina o grid do mapa com obstaculos
    grid_obst = set_map()

    # Variaveis globais dos indices no grid
    global i_curr, j_curr, i_next, j_next, i_goal, j_goal

    # Inicia as indices anteriores no grid do ponto objetivo e do desejado
    i_goal_old = -2
    j_goal_old = -2
    i_next = -2
    j_next = -2

    # Enquanto o programa nao for interrompido
    while not rospy.is_shutdown():

        # Encontra a posicao dos pontos atual e objetivo no grid
        i_curr = int( ((map_size[1]/2) - y_curr) * ratio[1] )
        j_curr = int( ((map_size[0]/2) + x_curr) * ratio[0] )
        i_goal = int( ((map_size[1]/2) - y_goal) * ratio[1] )
        j_goal = int( ((map_size[0]/2) + x_goal) * ratio[0] )

        # Caso haja alteracao dos pontos, realiza o metodo A estrela
        if i_goal != i_goal_old or j_goal != j_goal_old or \
           abs(i_curr-i_next) >= 2 or abs(j_curr-j_next) >= 2:
            grid = apply_star(grid_obst)
            i_goal_old = copy(i_goal)
            j_goal_old = copy(j_goal)

        #print grid[:,:,0]
        #print grid[:,:,1]
        #print adsf

        # Determina o indice desejado no grid
        set_next_index(grid)

        # Determina o ponto desejado e sua derivada
        x_next, y_next, dx_next, dy_next = set_next_point()

        # Determina o sinal de controle
        Ux, Uy = control_signal(x_next, y_next, dx_next, dy_next)

        # Calcula as velocidades
        vel.linear.x, vel.angular.z = feedback_linearization(Ux, Uy)

        # Publica as velocidades
        pub.publish(vel)

        # Espera o tempo necessario para a frequencia desejada
        rate.sleep()



# Funcao inicial
if __name__ == '__main__':
    try:
        star()
    except rospy.ROSInterruptException:
        pass

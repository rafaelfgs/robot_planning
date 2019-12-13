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

# Ganho de importancia da luminosidade
k_lumin = 1.0

# Frequencia de atualizacao do programa
freq_stage = 20.0

# Tamanho do robo
robot_size = 1.0

# Ponto de controle do feedback linearization
d = 0.1

# Parametro de controle para a convergencia (kp e kd)
k = [1.0, 0.2]

# Tolerancia para o ponto objetivo
dist_tol = 0.1

# Inicia a variavel da pose do robo
x_curr = 0.0
y_curr = 0.0
th_curr = 0.0

# Inicia a variavel do ponto objetivo
x_goal = 0.0
y_goal = 0.0

# Inicia as indices no grid dos pontos necessarios
i_next = -2
j_next = -2
i_goal_old = -2
j_goal_old = -2

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



# Rotina para a discretizacao do mapa de obstaculos e de luminosidade
def create_grid():

    # Inicia a contagem do tempo
    t = time.time()

    # Inicia os grids com zeros
    grid_obst = np.zeros((resolution_map[1],resolution_map[0]))
    grid_lumin = np.zeros((resolution_map[1],resolution_map[0]))
    
    # Carrega a imagem do arquivo do mapa de obstaculos
    path_obst = img_path
    cv_obst = cv2.imread(path_obst, 0)
    img_obst = np.array(cv_obst) > 127
    
    # Carrega a imagem do arquivo do mapa de luminosidade
    path_lumin = img_path[0:len(img_path)-4] + "_lux.png"
    cv_lumin = cv2.imread(path_lumin, 0)
    img_lumin = np.array(cv_lumin)

    # Determina os tamanhos e a relacao entre as discretizacoes
    img_size = np.shape(img_obst)
    ratio_img = np.array([float(img_size[0])/resolution_map[1],
                          float(img_size[1])/resolution_map[0]])

    # Encontra o tamanho do robo na imagem
    i_robot = int(robot_size*img_size[0]/float(map_size[1])+1)
    j_robot = int(robot_size*img_size[1]/float(map_size[0])+1)

    # Percorre o grid
    for i in range(len(grid_obst)):
        for j in range(len(grid_obst[0])):

            # Determina os indices para o mapa de obstaculos
            i_min = max(0, int(i*ratio_img[0]) - int(i_robot/2))
            i_max = min(img_size[0], int(ceil((i+1)*ratio_img[0])) +
                                     int(i_robot/2)+1)
            j_min = max(0, int(j*ratio_img[1]) - int(j_robot/2))
            j_max = min(img_size[1], int(ceil((j+1)*ratio_img[1])) +
                                     int(j_robot/2)+1)

            # Caso haja algum obstaculo, marca como inf os pontos do grid
            if not np.all(img_obst[i_min:i_max,j_min:j_max]): 
                grid_obst[i,j] = np.inf

            # Determina os indices para o mapa de luminosidade
            i_min = max(0, int(i*ratio_img[0]))
            i_max = min(img_size[0], int(ceil((i+1)*ratio_img[0])) + 1)
            j_min = max(0, int(j*ratio_img[1]))
            j_max = min(img_size[1], int(ceil((j+1)*ratio_img[1])) + 1)

            # Determina a luminosidade do ponto como a media
            grid_lumin[i,j] = np.mean(img_lumin[i_min:i_max,j_min:j_max])

            # Imprime o andamento atual da funcao
            sys.stdout.write("\rDiscretizing map image: %3.1f%%" %
                             (100*float(i*len(grid_obst[0])+j+1) /
                                  np.prod(resolution_map)))
    sys.stdout.write("\rDiscretization performed in %.4f segundos\n" %
                     (time.time()-t))
    #print grid_obst[108:114,168:174]
    #print grid_lumin[108:114,168:174]

    return grid_obst, grid_lumin



# Rotina para a discretizacao do mapa identificando os obstaculos
def verify_changes():

    # Variaveis globais dos indices no grid
    global i_curr, i_next, i_goal, i_goal_old
    global j_curr, j_next, j_goal, j_goal_old

    # Flag de ativacao do A*
    pos_changed = False

    # Atualiza os valores atuais
    i_curr = int( ((map_size[1]/2) - y_curr) * ratio[1] )
    j_curr = int( ((map_size[0]/2) + x_curr) * ratio[0] )
    i_goal = int( ((map_size[1]/2) - y_goal) * ratio[1] )
    j_goal = int( ((map_size[0]/2) + x_goal) * ratio[0] )

    # Caso ocorra mudanca da posicao do objetivo ou do robo
    while i_goal != i_goal_old or j_goal != j_goal_old or \
          abs(i_curr-i_next) >= 2 or abs(j_curr-j_next) >= 2:

        # Habilita a flag do A*
        sys.stdout.write("Goal or Robot position has changed\n")
        pos_changed = True

        # Atualiza os valores passados
        i_next = copy(i_curr)
        j_next = copy(j_curr)
        i_goal_old = copy(i_goal)
        j_goal_old = copy(j_goal)

        # Atualiza os valores atuais
        time.sleep(1)
        i_curr = int( ((map_size[1]/2) - y_curr) * ratio[1] )
        j_curr = int( ((map_size[0]/2) + x_curr) * ratio[0] )
        i_goal = int( ((map_size[1]/2) - y_goal) * ratio[1] )
        j_goal = int( ((map_size[0]/2) + x_goal) * ratio[0] )

    return(pos_changed)



# Rotina para a aplicacao da heuristica de manhattan no grid
def apply_manhattan(grid_obst):

    # Inicial o grid com zeros
    grid = np.zeros((resolution_map[1],resolution_map[0]))

    # Determina o valor da distancia de manhattan dos pontos
    for i in range(resolution_map[1]):
        for j in range(resolution_map[0]):
            d_max = max(abs(i-i_goal),abs(j-j_goal))
            d_min = min(abs(i-i_goal),abs(j-j_goal))
            grid[i,j] = d_max-d_min + 1.5*d_min + grid_obst[i,j]

    return grid



# Rotina para a aplicacao da heuristica de luminosidade no grid
def apply_luminosity(grid_lumin):

    # Determina pesos para o grid de acordo com a luminosidade
    grid = (50-copy(grid_lumin))**k_lumin

    return grid



# Rotina do metodo A estrela no grid
def apply_star(grid_obst, grid_lumin):

    # Reinicia o status atual
    global status
    status = 0

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
        grid_l = apply_luminosity(grid_lumin)

        # Determina os valores dos grids do ponto inicial
        grid[i_curr,j_curr] = [-1,-1]
        grid_g[i_curr,j_curr] = 1.0
        grid_f[i_curr,j_curr] = grid_g[i_curr,j_curr] + grid_h[i_curr,j_curr]

        # Enquanto nao chegar ao ponto objetivo
        while np.any(grid_f != np.inf) and grid[i_goal,j_goal,0] == np.inf:

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
                      [min(resolution_map[1]-1,k[0][i]+1),
                       min(resolution_map[0]-1,k[1][i]+1)]]

                # Determina os valores dos grids para os vizinhos
                for j in range(8):
                    if grid_g[nb[j][0],nb[j][1]] == 0:
                        grid[nb[j][0],nb[j][1]] = [k[0][i],k[1][i]]
                        grid_g[nb[j][0],nb[j][1]] = grid_g[k[0][i],k[1][i]] + abs(k[0][i]-nb[j][0]) + abs(k[1][i]-nb[j][1])
                        grid_f[nb[j][0],nb[j][1]]=grid_g[nb[j][0],nb[j][1]] + grid_h[nb[j][0],nb[j][1]] + grid_l[nb[j][0],nb[j][1]]

            grid_f[k[0][i],k[1][i]] = np.inf

            # Imprime o andamento atual da funcao
            sys.stdout.write("\rApplying A* algorithm: %3.1f%%" %
                             (100*(1-(float(np.sum(grid_g==0)) /
                                      np.sum(grid_obst==0)))))

    sys.stdout.write("\nA* performed in %.4f segundos\n" % (time.time()-t))

    return grid



# Rotina para definicao do indice desejado no grid
def set_next_index(grid):

    # Indice desejado no grid
    global i_next, j_next

    # Status para o criterio de parada
    global status

    # Determina os vizinhos
    nb = [[max(0,i_curr-1),i_curr,i_curr,min(resolution_map[1]-1,i_curr+1)],
          [j_curr,max(0,j_curr-1),min(resolution_map[0]-1,j_curr+1),j_curr]]

    # Se ha como alcancar o objetivo, imprime uma mensagem e para o robo
    if grid[i_goal,j_goal,0] == np.inf:
        if status != 1:
            sys.stdout.write("Solution not found!!!\n")
            status = 1
        i_next, j_next = copy(i_curr), copy(j_curr)

    # Se alcancou o ponto objetivo, imprime uma mensagem e para o robo
    elif i_curr == i_goal and j_curr == j_goal:
        if status != 2:
            sys.stdout.write("Goal reached!!!\n")
            status = 2
        i_next, j_next = copy(i_curr), copy(j_curr)

    # Caso contrario, imprime uma mensagem e determina o proximo ponto
    else:
        if status != 3:
            sys.stdout.write("Moving to goal...\n")
            status = 3

        if i_next == -2 or (i_curr == i_next and j_curr == j_next):

            # Determina o indice desejado como o ponto objetivo
            i_next, j_next = copy(i_goal), copy(j_goal)

            # Em condicoes normais, retorna o indice para proximo ao robo
            while not ( np.all([i_next,j_next] == [i_curr,j_curr]) or \
                        np.all(grid[i_next,j_next] == [i_curr,j_curr]) or \
                        np.all(grid[i_next,j_next] == np.inf) or \
                        np.all(grid[i_next,j_next] == -1.0) ):
                i_next, j_next = grid[i_next,j_next].astype(int)



# Rotina para definicao do proximo ponto desejado
def set_next_point():

    # Caso nao tenha solucao
    if status == 1:
        x_next = copy(x_curr)
        y_next = copy(y_curr)
        dx_next, dy_next = 0.0, 0.0

    # Caso tenha alcancado o indice do ponto objetivo
    elif status == 2:

        # Verifica se o robo esta proximo ao ponto objetivo
        if sqrt((x_curr-x_goal)**2+(y_curr-y_goal)**2) > dist_tol:
            x_next = copy(x_goal)
            y_next = copy(y_goal)
            dx_next, dy_next = 0.0, 0.0
        else:
            x_next = copy(x_curr)
            y_next = copy(y_curr)
            dx_next, dy_next = 0.0, 0.0

    # Caso esteja movendo para o objetivo
    else:

        # Determina a proxima posicao desejada do robo
        x_next =  (j_next+0.5)/ratio[0] - map_size[0]/2
        y_next = -(i_next+0.5)/ratio[1] + map_size[1]/2

        # Velocidade da posicao desejada
        #d_next = sqrt((y_next-y_curr)**2+(x_next-x_curr)**2)
        th_next = atan2(y_next-y_curr,x_next-x_curr)
        dx_next = cos(th_next)
        dy_next = sin(th_next)

    return x_next, y_next, dx_next, dy_next



# Rotina para definicao do sinal de controle
def control_signal(x_next, y_next, dx_next, dy_next):

    # Sinal de controle em x e y
    Ux = k[0] * (x_next-x_curr) + k[1] * dx_next
    Uy = k[0] * (y_next-y_curr) + k[1] * dy_next

    # Saturacao do sinal de controle
    if sqrt(Ux**2+Uy**2) > 1:
       Ux = Ux / sqrt(Ux**2+Uy**2)
       Uy = Uy / sqrt(Ux**2+Uy**2)

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

    # Tempo para iniciar o mapa e os callbacks
    time.sleep(2.0)

    # Inicia a variavel de velocidade
    vel = Twist()

    # Determina o grid do mapa de obstaculos e de luminosidade
    grid_obst, grid_lumin = create_grid()
    #a = copy(grid_obst)
    #a[a==0] = 255
    #a[a>255] = 0
    #cv2.imwrite("/home/rafael/Desktop/catacombs_obst4.png",a)
    #print asdf

    # Enquanto o programa nao for interrompido
    while not rospy.is_shutdown():

        # Verifica se houve mudancas nas posicoes
        pos_changed = verify_changes()

        #print i_curr, i_next, i_goal, i_goal_old
        #print j_curr, j_next, j_goal, j_goal_old

        # Realiza o metodo A estrela
        if pos_changed:
            grid = apply_star(grid_obst, grid_lumin)
            pos_changed = False
        #time.sleep(8)
        #print asdf
        # Determina o indice desejado no grid
        set_next_index(grid)

        # Determina o ponto desejado e sua derivada
        x_next, y_next, dx_next, dy_next = set_next_point()

        #print i_curr, i_next, i_goal, j_curr, j_next, j_goal
        #print x_curr, x_next, x_goal, y_curr, y_next, y_goal

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

RAFAEL FERNANDES GONÇALVES DA SILVA
PLANEJAMENTO DE MOVIMENTO OD ROBÔS
TRABALHO PRÁTICO 1

INSTRUÇÕES PARA A EXECUCAO
FAVOR ATENTAR NO CAMINHO DO MAPA



CONFIGURAÇÕES INICIAOS

1. Utilize uma workspace já existente uu crie uma nova workspace com o comando:

   mkdir -p ~/catkin_ws/src

2. Descompacte o arquivo .zip e copie a pasta rafael_tp1 para a pasta ~/catkin_ws/src

3. Entre na pasta ~/catkin_ws e construa a workspace com o comando:

   catkin_make

4. Adicione as entradas ao arquivo setup.bash com o comando:

   source devel/setup.bash



LEMINISCATA:

Utilize o comando:

   roslaunch rafael_tp1 lemin.launch map:="CAMINHO_DA_WORKSPACE/src/rafael_tp1/maps/map_blank.world" cx:=4.0 cy:=3.0 rx:=2.0 ry:=1.0 freq:=0.05

   onde o CAMINHO_DA_WORKSPACE deve ser alterado para o caminho da workspace utilizada (/home/USER/catkin_ws)

   e os valores cx, cy, rx e ry são respectivamente os centros e os raios em x e y da curva e freq é a frequencia da trajetória da curva (inverso do período)

   O caminho do mapa DEVE ser passado como parâmetro, mas os valores numéricos (cx, cy, rx e ry) podem ser omitidos (padrão utilizado cx=0.0 cy=0.0 rx=8.0 ry=4.0 freq=0.01)



TANGENT BUG:

Utilize o comando:

   roslaunch rafael_tp1 tangbug.launch map:="CAMINHO_DA_WORKSPACE/src/rafael_tp1/maps/map_obst.world" x_goal:=-24.0 y_goal:=6.0

   onde o CAMINHO_DA_WORKSPACE deve ser alterado para o caminho da workspace utilizada (/home/USER/catkin_ws)

   e os valores x_goal e y_goal representam as coordenadas do ponto alvo

   O caminho do mapa DEVE ser passado como parâmetro, mas os valores numéricos (x_goal e y_goal) podem ser omitidos (padrão utilizado x_goal=-24.0 e y_goal:=6.0

   Há 3 mapas na pasta .../maps: map_obst.world, map_room.world, map_maze.world e map_blank.world

   Para alterar a posição inicial do robô, pode-se arrastar o robô na interface do Stage, ou pode-se alterar a penúltima linha dos mapas.

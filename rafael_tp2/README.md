RAFAEL FERNANDES GONCALVES DA SILVA
PLANEJAMENTO DE MOVIMENTO DE ROBOS
TRABALHO PRATICO 2



A) DESCRICAO DE CADA UM DOS ARQUIVOS DE CODIGO


OBS: Os arquivos de código estão na pasta ~/catkin_ws/src/rafael_tp2/scripts/


1. FUNCAO POTENCIAL (potfunc.py):

    Inicializacao das constantes
    Funcao callback da pose do robo
    Funcao callback do laser
    Funcao callback da posicao do goal

    Funcao de potencial atrativo
        Gradiente atrativo
        Tangente hiperbólica para saturação do sinal

    Funcao de potencial repulsivo
        Gradiente do obstaculo
        Gradiente repulsivo
        Tangente hiperbólica para saturação do sinal

    Funcao de feedback linearization

    Funcao principal
        Inicializacao das funcoes do ROS
        Enquanto nao encerrar o programa
            Se estiver longe do alvo
                Chama a funcao de potencial atrativo
                Chama a uncao de potencial repulsivo
                Determina o sinal de controle como a soma
            Caso contrario
                Determina o sinal de controle nulo
            Chama a funcao de feedback linearization


2. WAVEFRONT (wavefront.py):

    Inicializacao das constantes
    Funcao callback da pose do robo
    Funcao callback da posicao do goal

    Funcao da discetrizacao do mapa
        Inicia o grid nulo
        Le a imagem png
        Calcula a taxa de conversao (resolucao do grid pelo tamanho da imagem)
        Atribui no grid os valores 0 para livre e inf para obstaculo

    Funcao do wavefront
        Inicia as variaveis
        Converte o ponto do alvo para o grid
        Marca o alvo no grid
        Enquanto houver vizinhos
            Determina os vizinhos
            Se possivel, atribui valores a eles

    Funcao do proximo ponto desejado
        Converte o ponto atual para o grid
        Se houver valor no ponto atual
            Determina os vizinhos
            Encontra qual deles possui menor valor
            Converte a posicao do menor valor para o mapa
        Caso contrario
            Determina o ponto desejado como o atual
    

    Funcao do sinal de controle
    Funcao de feedback linearization

    Funcao principal
        Inicializacao das funcoes do ROS
        Enquanto nao encerrar o programa
            Se estiver modificacao do alvo
                Chama a funcao do wavefront
            Chama a funcao do proximo ponto desejado
            Se estiver proximo ao alvo
                Determina o ponto desejado como o atual
            Chama a funcao do sinal de controle
            Chama a funcao de feedback linearization


3. A-ESTRELA (star.py):

    Inicializacao das constantes
    Funcao callback da pose do robo
    Funcao callback da posicao do goal

    Funcao da discetrizacao do mapa
        Inicia o grid nulo
        Le a imagem png
        Calcula a taxa de conversao (resolucao do grid pelo tamanho da imagem)
        Atribui no grid os valores 0 para livre e inf para obstaculo

    Funcao da heuristica manhattan
        Inicia o grid
        Converte o ponto do alvo para o grid
        Se o alvo estiver dentro do mapa
            Percorre o grid
                Determina o valor da norma 1 para cada ponto

    Funcao do A_estrela
        Inicia a matriz de indices (grafo)
        Inicia as grids F e G
        Chama a funcao da heuristica manhattan para o grid H
        Converte o ponto do alvo para o grid
        Se o alvo estiver no espaço livre
            Converte o ponto atual do robo para o grid
            Marca o ponto atual nos grids F e G
                Enquanto nao alcancar o alvo
                    Encontra pontos com os menores valores de F
                    Percorre cada um desses pontos
                        Determina os vizinhos
                        Se possivel, atribui valores aos grids F e G aos vizinhos
                        Marca o ponto atual do grid F com inf
                        

    Funcao do proximo ponto desejado
        Converte o ponto atual para o grid
        Se nao houver solucao
            Determina o ponto desejado como o atual
        Se houver alcancado o alvo
            Determina o ponto desejado como o atual
        Caso contrario
            Inicia o indice no alvo
            Percorre o grid de indices ate chegar ao ponto anterior ao robo
                Determina o proximo ponto do grid
            Converte a posicao do proximo ponto do para o mapa
    

    Funcao do sinal de controle
    Funcao de feedback linearization

    Funcao principal
        Inicializacao das funcoes do ROS
        Enquanto nao encerrar o programa
            Se estiver modificacao do alvo
                Chama a funcao do A_estrela
            Chama a funcao do proximo ponto desejado
            Chama a funcao do sinal de controle
            Chama a funcao de feedback linearization



B) INSTRUCOES DE COMO COMPILAR E EXECUTAR OS PROGRAMAS


CONFIGURACOES INICIAIS:

1. Utilize uma workspace ja existente ou crie uma nova workspace com o comando:

    mkdir -p ~/catkin_ws/src

2. Copie a pasta rafael_tp2 do arquivo .zip para a pasta ~/catkin_ws/src

3. Entre na pasta ~/catkin_ws e construa a workspace com os comandos:

    cd ~/catkin_ws/
    catkin_make
    chmod +x src/rafael_tp2/scripts/*.py
    source devel/setup.bash


COMANDOS GERAIS:

1. FUNCAO POTENCIAL:

    roslaunch rafael_tp2 potfunc.launch

2. WAVEFRONT:

    roslaunch rafael_tp2 wavefront.launch

3. A-ESTRELA:

    roslaunch rafael_tp2 star.launch


OBS: O robo (azul) e o alvo (verde) podem ser movimentados com o mouse.


PARA TESTAR OUTROS MAPAS E PARAMETROS (NOME, TAMANHO E RESOLUCAO DO MAPA):

1. FUNCAO POTENCIAL:

    roslaunch rafael_tp2 potfunc.launch name:="big"
    roslaunch rafael_tp2 potfunc.launch name:="room"
    roslaunch rafael_tp2 potfunc.launch name:="maze"

2. WAVEFRONT:

    roslaunch rafael_tp2 wavefront.launch name:="big" size:="16 12" resolution:="16 12"
    roslaunch rafael_tp2 wavefront.launch name:="room" size:="40 40" resolution:="20 20"
    roslaunch rafael_tp2 wavefront.launch name:="maze" size:="40 40" resolution:="20 20"

3. A-ESTRELA:

    roslaunch rafael_tp2 star.launch name:="big" size:="16 12" resolution:="16 12"
    roslaunch rafael_tp2 star.launch name:="room" size:="40 40" resolution:="20 20"
    roslaunch rafael_tp2 star.launch name:="maze" size:="40 40" resolution:="20 20"
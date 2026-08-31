
#!/usr/bin/env pybricks-micropython

from calibracao.constantes import *
from .movimentosBasicos import *


def ler_linha():

    s1 = sensor_corDr.reflection()
    s2 = sensor_corEs.reflection()

    return s1, s2


def encontrou_linha():

    s1, s2 = ler_linha()

    if s1 < LIMIAR_PRETO:
        return True

    if s2 < LIMIAR_PRETO:
        return True

    return False


def desviar():

    # -----------------------------------------------------
    # 1. PARA
    # -----------------------------------------------------

    parar()
    wait(1000)
  #aqui

    # -----------------------------------------------------
    # 2. SAI DA LINHA
    # -----------------------------------------------------

    virarEsquerda()
    wait(2000)
  #aqui
    parar()
    wait(1000)


    # -----------------------------------------------------
    # 3. ANDA AO LADO DO OBSTÁCULO
    # -----------------------------------------------------

    andar(200)

    wait(1500)

    # Continua andando enquanto o objeto estiver
    # sendo detectado pelo sensor lateral
    while sensor_distanciaLateral.distance() < 15:
        wait(50)


    # -----------------------------------------------------
    # 4. GARANTE QUE PASSOU COMPLETAMENTE DO OBJETO
    # -----------------------------------------------------

    # Quando o sensor perde o objeto, ainda anda
    # um pouco para garantir que a traseira do robô
    # também ultrapassou o obstáculo.

    andar(200)
    wait(2000)

    parar()
    wait(500)
  #aqui

    distancia_lateral = sensor_distanciaLateral.distance()

    ev3.screen.clear()
    ev3.screen.print("OBJETO PASSOU!")
    ev3.screen.print("Dist:", distancia_lateral)

    wait(500)


    # -----------------------------------------------------
    # 5. VIRA PARA A ESQUERDA
    # -----------------------------------------------------

    virarEsquerda()
    wait(2000)
  #aqui
    parar()
    wait(500)

   


    # -----------------------------------------------------
    # 6. VIRA PARA A DIREITA
    # -----------------------------------------------------

    virarDireita()
    wait(2000)
  #aqui
    parar()
    wait(500)
  #aqui

    andar(200)
    wait(2000)
  #aqui
    virarEsquerda()
    wait(2000)
  #aqui
    parar()
    wait(500)
    #aqui


    # -----------------------------------------------------
    # 8. PROCURA A LINHA
    # -----------------------------------------------------

    andar(200)

    while not encontrou_linha():
        wait(50)


    # -----------------------------------------------------
    # 9. ENCONTROU A LINHA
    # -----------------------------------------------------
    
    parar()
    wait(500)
    #aqui

    ev3.screen.clear()
    ev3.screen.print("LINHA ENCONTRADA!")

    wait(1000)


#arrumar os wait e as funções de virar
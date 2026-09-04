from calibracao.constantes import *
from .movimentosBasicos import *
from .seguidorLinha import*

def sala3():
    direcao= None
    # Entrou na sala
    andar(200)
    wait(3000)

    # Espera encontrar a primeira parede
    while sensor_distanciaFrente.distance() > 30:
        wait(50)

    parar()
    wait(300)

    # Primeira curva
    virarEsquerda()
    wait(4500)

    if sensor_distanciaFrente.distance()<25:
        virar180()
        wait(4500)
        direcao= "Esquerda"
        

    parar()
    wait(300)

    # Anda até encontrar a segunda parede
    andar(300)

    while sensor_distanciaFrente.distance() > 20:
        wait(50)

    parar()
    wait(300)

    # Segunda curva

    if direcao==" Esquerda":
     virarDireita()
     wait(4300)

    else:
        virarDireita()
        wait(4300)
    parar()
    wait(300)

    # Sai da sala
    andar(200)

def virar180():
    motorDr.run(200)
    motorEs.run(-200)


def verificar_cinza(ev3):

    cor_esq, cor_dir = verificar_cor()

    print("ESQ:", cor_esq)
    print("DIR:", cor_dir)

    esq = (cor_esq == "CINZA")
    dir_ = (cor_dir == "CINZA")
    

    if not esq and not dir_:
         return False
    if esq and dir_:

        ev3.screen.clear()
        ev3.screen.print("CINZA!")


        return True

    
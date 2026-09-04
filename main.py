#!/usr/bin/env pybricks-micropython

from calibracao.constantes import *
from movimentos.verde import *
from movimentos.seguidorLinha import seguidor_linha
from movimentos.desvia import desviar
from menus.principal import menu_principal
from menus.principal import *
from movimentos.movimentosBasicos import *
from movimentos.sala3 import*


def programa_principal():

    ev3.screen.clear()
    ev3.screen.print("Executando:")
    ev3.screen.print("Principal")

    wait(1000)

    while True:

        if Button.CENTER in ev3.buttons.pressed():

            while ev3.buttons.pressed():
                wait(10)

            parar()
            ev3.screen.clear()
            ev3.screen.print("Voltando ao menu...")
            wait(500)
            return


        
     
        if verificar_verde():
            continue

        
        distancia = sensor_distanciaFrente.distance()
        
        if distancia < DISTANCIA_OBJETO:

            desviar()
    
        if verificar_vermelho(ev3):

            parar()
            ev3.speaker.beep(400)

            wait(5000)

        if verificar_cinza(ev3):
            ev3.speaker.beep(700)
            wait(100)
            ev3.speaker.beep(400)
            wait(100)
            sala3()


        seguidor_linha()

        wait(10)

def main():

    menu_principal(programa_principal)


if __name__ == "__main__":

    main()
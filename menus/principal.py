#!/usr/bin/env pybricks-micropython

from calibracao.constantes import *
from calibracao import *
from movimentos.movimentosBasicos import  *
from calibracao.hsv import *
from movimentos.seguidorLinha import  *
from movimentos.desvia import  *
from movimentos.sala3 import  *
from movimentos.verde import  *

NOTE_E6 = 1318
NOTE_D6 = 1175
NOTE_F_SHARP_5 = 740
NOTE_G_SHARP_5 = 831
NOTE_C_SHARP_6 = 1109
NOTE_B5 = 988
NOTE_D_SHARP_5 = 622
NOTE_E5 = 659
NOTE_A5 = 880
NOTE_C_SHARP_5 = 554 

nokia = [
    (NOTE_E6, 120), (NOTE_D6, 120), (NOTE_F_SHARP_5, 240), (NOTE_G_SHARP_5, 240),
    (NOTE_C_SHARP_6, 120), (NOTE_B5, 120), (NOTE_D_SHARP_5, 240), (NOTE_E5, 240),
    (NOTE_B5, 120), (NOTE_A5, 120), (NOTE_C_SHARP_5, 240), (NOTE_E5, 240),
    (NOTE_A5, 480)
]
funcoes = ["Principal","Calibracao", "Debug", "Sair"]

indice = 0
total_opcoes = len(funcoes)
def  musicaInicializacao():
    for nota, duracao in nokia:
        ev3.speaker.beep(frequency=nota, duration=duracao)
        wait(30)
        

#musicaInicializacao()


def mostrar_menu():
    ev3.screen.clear()
    ev3.screen.print("A S T R O B O T")
    ev3.screen.print("-> " + funcoes[indice])
    ev3.screen.print("[Esq/Dir] Mudar")
    ev3.screen.print("[Centro] Executar")

def menu_principal(programa_principal):
    global indice
    while True:
        mostrar_menu()
        
        while not ev3.buttons.pressed():
            wait(10)
            
        botoes = ev3.buttons.pressed()
        
    
        if Button.RIGHT in botoes:
            indice = (indice + 1) % total_opcoes
            ev3.speaker.beep(frequency=1000, duration=50)
        elif Button.LEFT in botoes:
            indice = (indice - 1) % total_opcoes
            ev3.speaker.beep(frequency=1000, duration=50)
        elif Button.CENTER in botoes:
            ev3.speaker.beep(frequency=1500, duration=150)
            ev3.screen.clear()
            
            if indice == 0:
                ev3.screen.print("Executando:")
                ev3.screen.print(funcoes[0])
                #ev3.speaker.say("VAMOS LÀ")
                programa_principal()
                
                wait(1000) 
                
            elif indice == 1:
                ev3.screen.print("Executando:")
                ev3.screen.print(funcoes[1])
                #ev3.speaker.say("Calibração")
                menu_calibracao_cores()
                wait(1000)
                
            elif indice == 2:
                ev3.screen.print("Executando:")
                ev3.screen.print(funcoes[2])
                #ev3.speaker.say("Debug")
                menu_testes()
                wait(1000)


            
                
            elif indice == 3:
                ev3.screen.print("Saindo do Menu...")
                #ev3.speaker.say("Tchau ")
                wait(10)
                #ev3.speaker.say("Tchau")
                break
                
        while ev3.buttons.pressed():
            wait(10)



def menu_testes():

    funcoes = [
        "Movimentos",
        "Valor Cores",
        "Valor Distancia",
        "Testes Independetes",
        "Voltar"
    ]

    indice = 0

    while True:

        ev3.screen.clear()
        ev3.screen.print("TESTES")
        ev3.screen.print("> " + funcoes[indice])

        ev3.screen.print("")
        ev3.screen.print("[Esq/Dir] Mudar")
        ev3.screen.print("[Centro] Executar")

        while not ev3.buttons.pressed():
            wait(10)

        botoes = ev3.buttons.pressed()

        if Button.RIGHT in botoes:
            indice = (indice + 1) % len(funcoes)
            ev3.speaker.beep(frequency=1000, duration=50)

        elif Button.LEFT in botoes:
            indice = (indice - 1) % len(funcoes)
            ev3.speaker.beep(frequency=1000, duration=50)

        elif Button.CENTER in botoes:
            ev3.speaker.beep(frequency=1500, duration=150)

            if indice == 0:
                ev3.screen.clear()
                ev3.screen.print("Movimentos Basicos")
                ev3.speaker.say("MOVIMENTOS BÁSICOS")
                manuais()
                wait(500)
                

            elif indice == 1:
                
                pass
             
               

            elif indice == 2:
                
                pass
            elif indice == 3:
                
                menu_indepente()
                wait(500)

            elif indice == 4:
                # Voltar
                return

        while ev3.buttons.pressed():
            wait(10)


         

def menu_indepente():

    funcoes = [
        "Seguidor Linha",
        "Desvia",
        "Verde",
        "Sala 3",
        "Voltar"
    ]

    indice = 0

    while True:

        ev3.screen.clear()
        ev3.screen.print("TESTES")
        ev3.screen.print("> " + funcoes[indice])

        ev3.screen.print("")
        ev3.screen.print("[Esq/Dir] Mudar")
        ev3.screen.print("[Centro] Executar")

        while not ev3.buttons.pressed():
            wait(10)

        botoes = ev3.buttons.pressed()

        if Button.RIGHT in botoes:
            indice = (indice + 1) % len(funcoes)
            ev3.speaker.beep(frequency=1000, duration=50)

        elif Button.LEFT in botoes:
            indice = (indice - 1) % len(funcoes)
            ev3.speaker.beep(frequency=1000, duration=50)

        elif Button.CENTER in botoes:
            ev3.speaker.beep(frequency=1500, duration=150)

            if indice == 0:
                ev3.screen.clear()
                ev3.screen.print("Seguidor linha")
                ev3.speaker.say("Seguidor de linha")
                seguidor_linha()
                wait(500)
                

            elif indice == 1:
                ev3.screen.clear()
                ev3.screen.print("Desvia")
                ev3.speaker.say("Desvia")
                seguidor_linha()
                wait(500)
                

             
               

            elif indice == 2:
                
                pass
            elif indice == 3:
                
                menu_indepente()
                wait(500)

            elif indice == 4:
                # Voltar
                return

        while ev3.buttons.pressed():
            wait(10)

        
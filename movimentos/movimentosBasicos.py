#!/usr/bin/env pybricks-micropython

from calibracao.constantes import*

def andar(vel=velocidade):
    motorDr.run(vel)
    motorEs.run(vel)


def parar():
    motorDr.stop()
    motorEs.stop()


def curvaSuaveDireita(vel=velocidade):
    motorDr.run(vel)
    motorEs.run(-vel * 0.02)


def curvaSuaveEsquerda(vel=velocidade):
    motorDr.run(-vel * 0.02)
    motorEs.run(vel)


def virarDireita(vel=velocidade_curva):
    motorEs.run(-vel)
    motorDr.run(vel)


def virarEsquerda(vel=velocidade_curva):
    motorEs.run(vel)
    motorDr.run(-vel)


def re():
    motorDr.run(-velocidade_curva)
    motorEs.run(-velocidade_curva)

def virar_90(direcao):
    if direcao == "Es":
        parar()
        wait(500)
        virarEsquerda()

    else:
        virarDireita()    

def manuais():

    while True:
        

        botoes = ev3.buttons.pressed()
        if Button.CENTER in ev3.buttons.pressed():
            break
                          

        # ↑ + ←
        elif Button.UP in botoes and Button.LEFT in botoes:
            curvaSuaveEsquerda()

        # ↑ + →
        elif Button.UP in botoes and Button.RIGHT in botoes:
            curvaSuaveDireita()

        # ↑
        elif Button.UP in botoes:
            andar()

        # ↓
        elif Button.DOWN in botoes:
            re()

        # ←
        elif Button.LEFT in botoes:
            virarEsquerda()

        # →
        elif Button.RIGHT in botoes:
            virarDireita()

        # Centro

        else:
            parar()

        wait(50)
#!/usr/bin/env pybricks-micropython

from calibracao.constantes import *
from movimentos.movimentosBasicos import *
from calibracao.hsv import *
from movimentos.desvia import *


# ============================================================
# ESTADO DO PID
# ============================================================

erro_anterior = 0
integral = 0

# Impede que o robô detecte outra curva enquanto ainda está
# executando a curva atual.
fazendo_curva = False


# ============================================================
# SEGUIDOR DE LINHA
# ============================================================

def seguidor_linha():

    global fazendo_curva

    # --------------------------------------------------------
    # Se já está fazendo uma curva, NÃO executa o PID.
    # --------------------------------------------------------
    if fazendo_curva:
        return

    # --------------------------------------------------------
    # Primeiro verifica se existe uma curva de 90 graus.
    # --------------------------------------------------------
    curva = detectar_curva_90()

    if curva is not None:

        ev3.speaker.beep(200)

        fazendo_curva = True

        virar_90(curva)

        fazendo_curva = False

        return


    # ========================================================
    # LEITURA DOS SENSORES DE COR
    # ========================================================

    r1, g1, b1 = sensor_corEs.rgb()
    r2, g2, b2 = sensor_corDr.rgb()

    _, _, v1 = rgb_to_hsv(r1, g1, b1)
    _, _, v2 = rgb_to_hsv(r2, g2, b2)

    erro = v1 - v2


    # ========================================================
    # PID
    # ========================================================

    global erro_anterior
    global integral

    proporcional = erro * KP

    integral = integral + erro

    if integral > 100:
        integral = 100

    elif integral < -100:
        integral = -100

    i_output = integral * KI

    derivada = (erro - erro_anterior) * KD

    # Sinal total de correção
    turn = proporcional + i_output + derivada


    # ========================================================
    # VELOCIDADE DOS MOTORES
    # ========================================================

    velocidade_esquerda = VELOCIDADE_BASE + turn
    velocidade_direita = VELOCIDADE_BASE - turn

    motorEs.run(velocidade_esquerda)
    motorDr.run(velocidade_direita)

    erro_anterior = erro

    wait(10)


# ============================================================
# DETECÇÃO DE CURVA 90
# ============================================================

def detectar_curva_90():

    """
    Lê TODOS os sensores de reflexão usados na detecção
    da curva.

    Sensores:
        sensor_corEs_Multi
        sensorDr_Multi

    Retorna:
        'direita'
        'esquerda'
        'ambos'
        None
    """

    # --------------------------------------------------------
    # LEITURA DOS SENSORES DE REFLEXÃO
    # --------------------------------------------------------

    reflexao_esquerda = sensor_corEs_Multi.reflection()
    reflexao_direita = sensorDr_Multi.reflection()


    # --------------------------------------------------------
    # Ambos detectaram a condição de curva
    # --------------------------------------------------------

    if (reflexao_esquerda < LIMIAR and
            reflexao_direita < LIMIAR):

        return 'ambos'


    # --------------------------------------------------------
    # Sensor esquerdo detectou
    # --------------------------------------------------------

    if reflexao_esquerda < LIMIAR:

        return 'esquerda'


    # --------------------------------------------------------
    # Sensor direito detectou
    # --------------------------------------------------------

    if reflexao_direita < LIMIAR:

        return 'direita'


    return None


# ============================================================
# VIRAR 90 GRAUS
# ============================================================

def virar_90(direcao):

    """
    Faz a curva de 90 graus e só libera o PID depois que
    a linha for encontrada novamente.
    """

    parar()

    wait(100)


    # --------------------------------------------------------
    # CURVA PARA A DIREITA
    # --------------------------------------------------------

    if direcao == 'direita':

        # Pequena pausa para estabilizar
        parar()
        wait(100)

        # IMPORTANTE:
        # Use a função que realmente faz o robô girar
        # para a direita.
        virarDireita(200)


    # --------------------------------------------------------
    # CURVA PARA A ESQUERDA
    # --------------------------------------------------------

    elif direcao == 'esquerda':

        parar()
        wait(100)

        virarEsquerda(200)


    # --------------------------------------------------------
    # Caso ambos os sensores detectem
    # --------------------------------------------------------

    elif direcao == 'ambos':

        # Se ambos detectarem, pode ser uma interseção.
        # Aqui fazemos uma decisão conservadora.
        parar()

        wait(100)

        # Ajuste conforme o seu percurso.
        virarDireita(200)


    # --------------------------------------------------------
    # Espera terminar o giro
    # --------------------------------------------------------

    wait(500)


    # --------------------------------------------------------
    # Procura novamente a linha
    # --------------------------------------------------------

    esperar_linha_apos_curva()


    # --------------------------------------------------------
    # Zera o PID para evitar uma correção brusca depois
    # da curva.
    # --------------------------------------------------------

    resetar_pid()

    parar()

    wait(50)


# ============================================================
# ESPERAR A LINHA DEPOIS DA CURVA
# ============================================================

def esperar_linha_apos_curva(tentativas=150):

    """
    Depois de fazer a curva, fica procurando a linha.

    Enquanto estiver procurando:
        - PID fica desligado
        - não detecta outra curva
        - não fica corrigindo

    150 x 20 ms ~= 3 segundos
    """

    while tentativas > 0:

        # ----------------------------------------------------
        # Ler novamente TODOS os sensores
        # ----------------------------------------------------

        reflexao_esquerda = sensor_corEs_Multi.reflection()
        reflexao_direita = sensorDr_Multi.reflection()


        # ----------------------------------------------------
        # Se algum dos sensores encontrou a linha,
        # terminou a curva.
        # ----------------------------------------------------

        if (reflexao_esquerda >= LIMIAR or
                reflexao_direita >= LIMIAR):

            return


        wait(20)

        tentativas -= 1


    # --------------------------------------------------------
    # Não encontrou a linha dentro do tempo máximo.
    # --------------------------------------------------------

    parar()


# ============================================================
# RESET DO PID
# ============================================================

def resetar_pid():

    global integral
    global erro_anterior

    integral = 0
    erro_anterior = 0

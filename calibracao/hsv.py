
#!/usr/bin/env pybricks-micropython

from .constantes import *


# ==========================================================
# CONFIGURAÇÃO DA CALIBRAÇÃO
# ==========================================================

CORES_CALIBRACAO = [
    "PRETO",
    "CINZA",
    "VERDE",
    "VERMELHO",
    "Verificar Cores"
      
]

indice_cor = 0

# Quantidade de leituras usadas para calcular a faixa
NUM_LEITURAS = 20

# Margem adicionada aos valores calibrados
MARGEM_H = 20
MARGEM_S = 20
MARGEM_V = 10


# ==========================================================
# VALORES CALIBRADOS
# ==========================================================

calibracao_cores = {
    "PRETO": None,
    "CINZA": None,
    "VERDE": None,
    "VERMELHO": None,
    "BRANCO":None
    
}


# ==========================================================
# RGB -> HSV
# ==========================================================

def rgb_to_hsv(r, g, b):

    r = r / 100
    g = g / 100
    b = b / 100

    maior = max(r, g, b)
    menor = min(r, g, b)

    delta = maior - menor

    v = maior * 100

    if maior == 0:
        s = 0
    else:
        s = (delta / maior) * 100

    if delta == 0:

        h = 0

    elif maior == r:

        h = 60 * (((g - b) / delta) % 6)

    elif maior == g:

        h = 60 * (((b - r) / delta) + 2)

    else:

        h = 60 * (((r - g) / delta) + 4)

    return h, s, v


# ==========================================================
# CALIBRAR UMA COR
# ==========================================================

def calibrar_cor(nome_cor):

    ev3.screen.clear()

    ev3.screen.print("CALIBRANDO:")
    ev3.screen.print(nome_cor)
    ev3.screen.print("CENTRO = iniciar")

    # Espera apertar Centro
    while Button.CENTER not in ev3.buttons.pressed():
        wait(10)

    # Espera soltar o botão
    while Button.CENTER in ev3.buttons.pressed():
        wait(10)

    ev3.screen.clear()
    ev3.screen.print("Lendo...")
    ev3.screen.print(nome_cor)

    h_valores = []
    s_valores = []
    v_valores = []

    # ------------------------------------------------------
    # Faz várias leituras
    # ------------------------------------------------------

    for i in range(NUM_LEITURAS):

        r1, g1, b1 = sensor_corEs.rgb()

        h, s, v = rgb_to_hsv(r1, g1, b1)

        h_valores.append(h)
        s_valores.append(s)
        v_valores.append(v)

        ev3.screen.clear()
        ev3.screen.print(nome_cor)
        ev3.screen.print("Leitura:")
        ev3.screen.print(str(i + 1) + "/" + str(NUM_LEITURAS))
        ev3.screen.print("H:" + str(int(h)))
        ev3.screen.print("S:" + str(int(s)))
        ev3.screen.print("V:" + str(int(v)))

        wait(100)

    # ------------------------------------------------------
    # Calcula mínimo e máximo
    # ------------------------------------------------------

    h_min = min(h_valores) - MARGEM_H
    h_max = max(h_valores) + MARGEM_H

    s_min = min(s_valores) - MARGEM_S
    s_max = max(s_valores) + MARGEM_S

    v_min = min(v_valores) - MARGEM_V
    v_max = max(v_valores) + MARGEM_V

    # Limita os valores
    if h_min < 0:
        h_min = 0

    if h_max > 360:
        h_max = 360

    if s_min < 0:
        s_min = 0

    if s_max > 100:
        s_max = 100

    if v_min < 0:
        v_min = 0

    if v_max > 100:
        v_max = 100

    # ------------------------------------------------------
    # Salva calibração
    # ------------------------------------------------------

    calibracao_cores[nome_cor] = (
        h_min,
        h_max,
        s_min,
        s_max,
        v_min,
        v_max
    )

    ev3.screen.clear()
    ev3.screen.print(nome_cor)
    ev3.screen.print("CALIBRADO!")
    ev3.screen.print("H " + str(int(h_min)) + "-" + str(int(h_max)))
    ev3.screen.print("S " + str(int(s_min)) + "-" + str(int(s_max)))
    ev3.screen.print("V " + str(int(v_min)) + "-" + str(int(v_max)))

    ev3.speaker.beep()

    wait(1500)


# ==========================================================
# IDENTIFICAR COR
# ==========================================================

def identificar_cor(r, g, b):

    h, s, v = rgb_to_hsv(r, g, b)

    for nome_cor in [
        "PRETO",
        "CINZA",
        "VERDE",
        "VERMELHO"
    ]:

        faixa = calibracao_cores[nome_cor]

        if faixa is None:
            continue

        h_min, h_max, s_min, s_max, v_min, v_max = faixa

        # Vermelho
        if nome_cor == "VERMELHO":

            # Caso a faixa atravesse 0°
            if h_min > h_max:
                dentro_h = (
                    h >= h_min or h <= h_max
                )
            else:
                dentro_h = (
                    h_min <= h <= h_max
                )

        else:

            dentro_h = (
                h_min <= h <= h_max
            )

        dentro_s = (
            s_min <= s <= s_max
        )

        dentro_v = (
            v_min <= v <= v_max
        )

        if dentro_h and dentro_s and dentro_v:
            return nome_cor

    return "DESCONHECIDO"



def verificar_cor():

    r1, g1, b1 = sensor_corEs.rgb()
    r2, g2, b2 = sensor_corDr.rgb()

    h1, s1, v1 = rgb_to_hsv(r1, g1, b1)
    h2, s2, v2 = rgb_to_hsv(r2, g2, b2)

    cor_esquerda = identificar_cor(r1, g1, b1)
    cor_direita = identificar_cor(r2, g2, b2)

    print("--------------------------------")
    print("SENSOR ESQUERDO")
    print("RGB:", r1, g1, b1)
    print("HSV:", int(h1), int(s1), int(v1))
    print("COR:", cor_esquerda)

    print("--------------------------------")
    print("SENSOR DIREITO")
    print("RGB:", r2, g2, b2)
    print("HSV:", int(h2), int(s2), int(v2))
    print("COR:", cor_direita)

    ev3.screen.clear()

    ev3.screen.print("ESQ: " + cor_esquerda)
    ev3.screen.print("H:" + str(int(h1)) +
                     " S:" + str(int(s1)))

    ev3.screen.print("DIR: " + cor_direita)
    ev3.screen.print("H:" + str(int(h2)) +
                     " S:" + str(int(s2)))

    return cor_esquerda, cor_direita




def menu_calibracao_cores():

    global indice_cor

    while True:

        ev3.screen.clear()

        ev3.screen.print("CALIBRACAO")
        ev3.screen.print("-> " + CORES_CALIBRACAO[indice_cor])
        ev3.screen.print("Esq/Dir = mudar")
        ev3.screen.print("Centro = executar")
        ev3.screen.print("Baixo = sair")

        # Espera algum botão
        while not ev3.buttons.pressed():
            wait(10)

        botoes = ev3.buttons.pressed()

        # ==================================================
        # BOTAO DIREITA
        # ==================================================

        if Button.RIGHT in botoes:

            indice_cor = (
                indice_cor + 1
            ) % len(CORES_CALIBRACAO)

            ev3.speaker.beep(
                frequency=1000,
                duration=50
            )

        # ==================================================
        # BOTAO ESQUERDA
        # ==================================================

        elif Button.LEFT in botoes:

            indice_cor = (
                indice_cor - 1
            ) % len(CORES_CALIBRACAO)

            ev3.speaker.beep(
                frequency=1000,
                duration=50
            )

        # ==================================================
        # BOTAO CENTRO
        # ==================================================

        elif Button.CENTER in botoes:

            # Espera soltar o botão
            while ev3.buttons.pressed():
                wait(10)

            opcao = CORES_CALIBRACAO[indice_cor]

            # ----------------------------------------------
            # VERIFICAR CORES
            # ----------------------------------------------

            if opcao == "Verificar Cores":

                ev3.screen.clear()
                ev3.screen.print("Verificando...")
                wait(500)

                executar_hsv()

            # ----------------------------------------------
            # CALIBRAR COR
            # ----------------------------------------------

            else:

                calibrar_cor(opcao)

        # ==================================================
        # BOTAO PARA BAIXO
        # ==================================================

        elif Button.DOWN in botoes:

            # Espera soltar o botão
            while ev3.buttons.pressed():
                wait(10)

            ev3.screen.clear()
            ev3.screen.print("Saindo...")
            wait(500)

            return

        # ==================================================
        # ESPERA SOLTAR BOTAO
        # ==================================================

        while ev3.buttons.pressed():
            wait(10)

# ==========================================================
# MODO DE TESTE
# ==========================================================

def executar_hsv():

    print("--- TESTE DE CORES HSV ---")
    print("Centro = sair")
    print("--------------------------")

    while True:

        if Button.CENTER in ev3.buttons.pressed():

            while Button.CENTER in ev3.buttons.pressed():
                wait(10)

            break

        verificar_cor()

        wait(500)



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
MARGEM_H = 10
MARGEM_S = 10
MARGEM_V = 10


# ==========================================================
# VALORES CALIBRADOS
# ==========================================================

calibracao_cores = {
    "PRETO": {
        "ESQ": None,
        "DIR": None
    },

    "CINZA": {
        "ESQ": None,
        "DIR": None
    },

    "VERDE": {
        "ESQ": None,
        "DIR": None
    },

    "VERMELHO": {
        "ESQ": None,
        "DIR": None
    }
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

    while Button.CENTER not in ev3.buttons.pressed():
        wait(10)

    while Button.CENTER in ev3.buttons.pressed():
        wait(10)

    ev3.screen.clear()
    ev3.screen.print("Lendo...")
    ev3.screen.print(nome_cor)

    # Listas do sensor esquerdo
    h_esq = []
    s_esq = []
    v_esq = []

    # Listas do sensor direito
    h_dir = []
    s_dir = []
    v_dir = []

    for i in range(NUM_LEITURAS):

        # SENSOR ESQUERDO
        r1, g1, b1 = sensor_corEs.rgb()

        h1, s1, v1 = rgb_to_hsv(
            r1, g1, b1
        )

        h_esq.append(h1)
        s_esq.append(s1)
        v_esq.append(v1)

        # SENSOR DIREITO
        r2, g2, b2 = sensor_corDr.rgb()

        h2, s2, v2 = rgb_to_hsv(
            r2, g2, b2
        )

        h_dir.append(h2)
        s_dir.append(s2)
        v_dir.append(v2)

        ev3.screen.clear()
        ev3.screen.print(nome_cor)

        ev3.screen.print(
            "ESQ H:" + str(int(h1))
        )

        ev3.screen.print(
            "S:" + str(int(s1)) +
            " V:" + str(int(v1))
        )

        ev3.screen.print(
            "DIR H:" + str(int(h2))
        )

        wait(100)

    # ==========================================
    # CALCULA FAIXA ESQUERDA
    # ==========================================

    faixa_esq = criar_faixa(
        h_esq,
        s_esq,
        v_esq
    )

    # ==========================================
    # CALCULA FAIXA DIREITA
    # ==========================================

    faixa_dir = criar_faixa(
        h_dir,
        s_dir,
        v_dir
    )

    # ==========================================
    # SALVA
    # ==========================================

    calibracao_cores[nome_cor]["ESQ"] = faixa_esq

    calibracao_cores[nome_cor]["DIR"] = faixa_dir

    ev3.screen.clear()
    ev3.screen.print(nome_cor)
    ev3.screen.print("CALIBRADO!")

    ev3.speaker.beep()

    wait(1500)

def criar_faixa(h_valores, s_valores, v_valores):

    # ==========================================
    # MARGENS MAIORES
    # ==========================================

    margem_h = 10
    margem_s = 10
    margem_v = 10

    h_min = min(h_valores) - margem_h
    h_max = max(h_valores) + margem_h

    s_min = min(s_valores) - margem_s
    s_max = max(s_valores) + margem_s

    v_min = min(v_valores) - margem_v
    v_max = max(v_valores) + margem_v

    # ==========================================
    # LIMITES H
    # ==========================================

    if h_min < 0:
        h_min = 0

    if h_max > 360:
        h_max = 360

    # ==========================================
    # LIMITES S
    # ==========================================

    if s_min < 0:
        s_min = 0

    if s_max > 100:
        s_max = 100

    # ==========================================
    # LIMITES V
    # ==========================================

    if v_min < 0:
        v_min = 0

    if v_max > 100:
        v_max = 100

    return (
        h_min,
        h_max,
        s_min,
        s_max,
        v_min,
        v_max
    )
# ==========================================================
# IDENTIFICAR COR
# ==========================================================

def verificar_cor():

    # ==========================================
    # SENSOR ESQUERDO
    # ==========================================

    r1, g1, b1 = sensor_corEs.rgb()

    h1, s1, v1 = rgb_to_hsv(
        r1, g1, b1
    )

    cor_esquerda = identificar_cor(
        r1,
        g1,
        b1,
        "ESQ"
    )

    # ==========================================
    # SENSOR DIREITO
    # ==========================================

    r2, g2, b2 = sensor_corDr.rgb()

    h2, s2, v2 = rgb_to_hsv(
        r2,
        g2,
        b2
    )

    cor_direita = identificar_cor(
        r2,
        g2,
        b2,
        "DIR"
    )

    print("-----------------------------")

    print("ESQUERDO")
    print("RGB:", r1, g1, b1)
    print("HSV:",
          int(h1),
          int(s1),
          int(v1))
    print("COR:", cor_esquerda)

    print("-----------------------------")

    print("DIREITO")
    print("RGB:", r2, g2, b2)
    print("HSV:",
          int(h2),
          int(s2),
          int(v2))
    print("COR:", cor_direita)

    ev3.screen.clear()

    ev3.screen.print(
        "ESQ: " + cor_esquerda
    )

    ev3.screen.print(
        "H:" + str(int(h1)) +
        " S:" + str(int(s1))
    )

    ev3.screen.print(
        "DIR: " + cor_direita
    )

    ev3.screen.print(
        "H:" + str(int(h2)) +
        " S:" + str(int(s2))
    )

    return cor_esquerda, cor_direita



def identificar_cor(r, g, b, lado):

    h, s, v = rgb_to_hsv(r, g, b)

    # ==========================================
    # PRETO - USA APENAS A LUMINOSIDADE
    # ==========================================

    faixa_preto = calibracao_cores["PRETO"][lado]

    if faixa_preto is not None:

        h_min, h_max, s_min, s_max, v_min, v_max = faixa_preto

        if v_min <= v <= v_max:
            return "PRETO"


    # ==========================================
    # OUTRAS CORES
    # ==========================================

    for nome_cor in [
        "CINZA",
        "VERDE",
        "VERMELHO"
    ]:

        faixa = calibracao_cores[nome_cor][lado]

        if faixa is None:
            continue

        h_min, h_max, s_min, s_max, v_min, v_max = faixa

        # H
        if nome_cor == "VERMELHO":

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

        # S
        dentro_s = (
            s_min <= s <= s_max
        )

        # V
        dentro_v = (
            v_min <= v <= v_max
        )

        if dentro_h and dentro_s and dentro_v:
            return nome_cor

    return "DESCONHECIDO"

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


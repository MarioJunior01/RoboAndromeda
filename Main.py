#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, InfraredSensor
from pybricks.parameters import Port, Color
from pybricks.tools import wait, StopWatch

# =====================================================================
# RESUMO DAS MUDANÇAS EM RELAÇÃO AO CÓDIGO BASE
# ---------------------------------------------------------------------
# 1) BUG CRÍTICO CORRIGIDO: todas as funções de movimento tinham um
#    wait(1000) logo na primeira linha. Isso fazia o robô "congelar"
#    por 1 segundo inteiro TODA VEZ que uma manobra era chamada.
#    Removido.
# 2) Detecção de verde usando sensor.color() nos dois sensores de cor
#    já existentes (sem hardware novo), alternando com a leitura de
#    reflexão usada no PID — ver comentário na seção "DETECÇÃO DE
#    VERDE" sobre por que isso é feito só a cada N ciclos.
# 3) Lógica de interseção: 0 verde = segue reto | 1 verde = vira no
#    lado indicado | 2 verdes (juntos OU em sequência) = beco sem
#    saída, meia-volta.
# 4) Recuperação de linha perdida com duas fases: primeiro assume que
#    é um GAP (mantém o último comando dos motores por um tempo
#    curto); se a linha não voltar, assume que é uma CURVA DE 90° e
#    gira no lugar buscando a linha, no sentido para onde o robô já
#    estava curvando.
# 5) Anti-windup no termo integral do PID.
# 6) sensor_Ir agora é realmente verificado no loop principal (antes a
#    função desviar_obstaculo existia mas nunca era chamada).
# =====================================================================

ev3 = EV3Brick()
motorDr = Motor(Port.A)
motorEs = Motor(Port.B)
sensor_Ir = InfraredSensor(Port.S4)
sensor_corEs = ColorSensor(Port.S2)
sensor_corDr = ColorSensor(Port.S1)

DEBUG = True  # True para imprimir valores no visor do EV3 (deixa o loop mais lento)

# --------------------- CONSTANTES — AJUSTE NA PISTA -------------------
ALVO = 50               # reflexo alvo (robô centrado = metade preto/metade branco)
LIMITE_LINHA = 15        # abaixo disso o sensor está "em cima" de preto sólido
LIMITE_BRANCO = 75       # acima disso o sensor está "em cima" de branco sólido
# Calibre ALVO/LIMITE_LINHA/LIMITE_BRANCO na própria pista e iluminação
# do dia da prova: encoste o sensor na linha, no fundo branco e veja os
# valores reais com DEBUG = True antes de fixar esses números.

velocidade = 230         # velocidade máxima em linha reta
velocidade_curva = 160   # velocidade usada em pivôs (curva 90°, meia-volta, busca de linha)
distancia_obstaculo = 16

Kp_base = 2.5
Kd_base = 1.5
Ki = 0.0                 # deixe 0 a menos que precise; integral tende a desestabilizar em loops rápidos
VEL_MIN = 150
INTEGRAL_MAX = 200        # anti-windup

TEMPO_CICLO_MS = 30

INTERVALO_VERDE = 4       # a cada quantos ciclos do loop principal checamos a cor (ver seção DETECÇÃO DE VERDE)
TIMEOUT_VERDE_MS = 4000   # descarta detecção de verde "esquecida" se nenhum cruzamento aparecer depois

TEMPO_GAP_MS = 350        # tempo mantendo o último comando dos motores antes de assumir curva de 90°
TIMEOUT_PIVO_MS = 1500    # tempo máximo de busca da linha numa curva de 90° / recuperação de gap
TIMEOUT_MEIA_VOLTA_MS = 2500

TEMPO_MANOBRA_INICIAL_MS = 250  # tempo para "sair" da faixa preta do cruzamento antes de procurar a nova linha
CICLOS_CRUZAMENTO = 2     # leituras seguidas de preto forte para confirmar cruzamento (evita falso positivo)

# --------------------------- ESTADO GLOBAL -----------------------------
integral = 0
erro_anterior = 0
ultima_velA = 0          # última velocidade aplicada ao motorDr (usada para saber p/ que lado o robô estava curvando)
ultima_velB = 0          # última velocidade aplicada ao motorEs

contador_ciclo = 0
contagem_preto_duplo = 0

verde_contador = 0        # quantos "eventos" de verde novos desde o último cruzamento
verde_ultimo_lado = None  # 'ESQ' ou 'DIR' do último verde visto
verde_ativo_es = False    # sensor esquerdo está vendo verde agora?
verde_ativo_dir = False
tempo_ultimo_verde = None


# =========================== MOVIMENTOS BÁSICOS ==========================
def parar_motores():
    motorDr.stop()
    motorEs.stop()


def andar(vel=velocidade):
    motorDr.run(vel)
    motorEs.run(vel)


def virarDireita(vel=velocidade_curva):
    motorEs.run(-vel)
    motorDr.run(vel)


def virarEsquerda(vel=velocidade_curva):
    motorEs.run(vel)
    motorDr.run(-vel)


def re(vel=velocidade_curva):
    motorDr.run(-vel)
    motorEs.run(-vel)


def avancar_curva(sentido, vel=velocidade_curva):
    """Anda para frente curvando para o lado indicado (roda de fora rápida,
    roda de dentro bem mais devagar). Usado para tirar os sensores de cima
    da faixa preta do cruzamento antes de girar no próprio eixo."""
    if sentido == 'DIR':
        motorDr.run(vel * 0.35)
        motorEs.run(vel)
    else:
        motorDr.run(vel)
        motorEs.run(vel * 0.35)


# ============================ CRUZAMENTOS ================================
def verificar_cruzamento(corEs, corDr):
    """Um cruzamento/interseção aparece, para os dois sensores laterais,
    como uma faixa preta cheia sob os dois ao mesmo tempo (diferente de uma
    curva normal, que só afeta um sensor de cada vez)."""
    global contagem_preto_duplo
    if corEs < LIMITE_LINHA and corDr < LIMITE_LINHA:
        contagem_preto_duplo += 1
    else:
        contagem_preto_duplo = 0
    return contagem_preto_duplo >= CICLOS_CRUZAMENTO


def atravessar_cruzamento_reto():
    andar(velocidade)
    wait(TEMPO_MANOBRA_INICIAL_MS)


def virar90(sentido):
    global integral, erro_anterior
    ev3.speaker.beep(500)

    avancar_curva(sentido, velocidade_curva)
    wait(TEMPO_MANOBRA_INICIAL_MS)

    if sentido == 'DIR':
        virarDireita(velocidade_curva)
    else:
        virarEsquerda(velocidade_curva)

    cronometro = StopWatch()
    while cronometro.time() < TIMEOUT_PIVO_MS:
        corEs = sensor_corEs.reflection()
        corDr = sensor_corDr.reflection()
        if sentido == 'DIR' and corDr < LIMITE_LINHA:
            break
        if sentido == 'ESQ' and corEs < LIMITE_LINHA:
            break
        wait(5)

    parar_motores()
    wait(60)
    integral = 0
    erro_anterior = 0


def meia_volta():
    """Beco sem saída: gira ~180° até reencontrar a linha (que só pode ser
    a mesma linha por onde o robô chegou)."""
    global integral, erro_anterior
    ev3.speaker.beep(300)
    wait(120)
    ev3.speaker.beep(300)

    virarDireita(velocidade_curva)
    wait(TEMPO_MANOBRA_INICIAL_MS)

    cronometro = StopWatch()
    saiu_do_preto = False
    while cronometro.time() < TIMEOUT_MEIA_VOLTA_MS:
        corEs = sensor_corEs.reflection()
        corDr = sensor_corDr.reflection()
        if not saiu_do_preto and corEs > LIMITE_BRANCO and corDr > LIMITE_BRANCO:
            saiu_do_preto = True
        if saiu_do_preto and (corEs < LIMITE_LINHA or corDr < LIMITE_LINHA):
            break
        wait(5)

    parar_motores()
    wait(60)
    integral = 0
    erro_anterior = 0


def resolver_cruzamento():
    if verde_contador >= 2:
        meia_volta()
    elif verde_contador == 1:
        virar90(verde_ultimo_lado)
    else:
        atravessar_cruzamento_reto()
    resetar_verdes()


# ============================ DETECÇÃO DE VERDE ===========================
# sensor.color() usa um modo diferente do sensor (COL-COLOR) do que
# sensor.reflection() (COL-REFLECT). Trocar de modo tem um custo de tempo
# no sensor EV3 (dezenas de ms), e fazer isso em TODO ciclo do PID
# deixaria o seguidor de linha instável/travado. Por isso a checagem de
# verde acontece só a cada INTERVALO_VERDE ciclos (o loop principal "pula"
# a leitura de reflexo nesse ciclo e mantém o último comando dos motores).
# Se seu robô tiver uma porta de sensor livre (neste código, S1 está
# livre), o ideal para competir é usar 1-2 sensores dedicados só para
# verde, sempre em modo cor, sem nunca precisar alternar — isso elimina
# esse trade-off por completo.
def atualizar_deteccao_verde():
    global verde_contador, verde_ultimo_lado, verde_ativo_es, verde_ativo_dir, tempo_ultimo_verde

    corEs_cor = sensor_corEs.color()
    corDr_cor = sensor_corDr.color()

    es_verde_agora = (corEs_cor == Color.GREEN)
    dir_verde_agora = (corDr_cor == Color.GREEN)

    # conta só a BORDA DE SUBIDA (verde que acabou de aparecer), assim um
    # único marcador visto em vários ciclos seguidos conta como 1 evento só,
    # mas dois marcadores (juntos ou em sequência) contam como 2.
    if es_verde_agora and not verde_ativo_es:
        verde_contador += 1
        verde_ultimo_lado = 'ESQ'
        tempo_ultimo_verde = StopWatch()
        if DEBUG:
            ev3.speaker.beep(1000)

    if dir_verde_agora and not verde_ativo_dir:
        verde_contador += 1
        verde_ultimo_lado = 'DIR'
        tempo_ultimo_verde = StopWatch()
        if DEBUG:
            ev3.speaker.beep(1200)

    verde_ativo_es = es_verde_agora
    verde_ativo_dir = dir_verde_agora


def resetar_verdes():
    global verde_contador, verde_ultimo_lado, verde_ativo_es, verde_ativo_dir, tempo_ultimo_verde
    verde_contador = 0
    verde_ultimo_lado = None
    verde_ativo_es = False
    verde_ativo_dir = False
    tempo_ultimo_verde = None


# ==================== LINHA PERDIDA (GAP OU CURVA DE 90°) =================
def tratar_linha_perdida():
    """Quando os dois sensores leem branco ao mesmo tempo, pode ser um GAP
    (falha curta na linha) ou uma CURVA FECHADA que a linha "escapou" dos
    sensores. Estratégia: primeiro mantém o último comando dos motores por
    TEMPO_GAP_MS (é o suficiente pra atravessar um gap comum sem desviar o
    robô do rumo). Se a linha não voltar nesse tempo, assume curva de 90° e
    gira no lugar para o lado que o robô já estava curvando antes de perder
    a linha (roda mais lenta = lado do giro)."""
    global integral, erro_anterior

    cronometro = StopWatch()
    while cronometro.time() < TEMPO_GAP_MS:
        corEs = sensor_corEs.reflection()
        corDr = sensor_corDr.reflection()
        if corEs < LIMITE_BRANCO or corDr < LIMITE_BRANCO:
            return  # linha reencontrada: era só um gap
        wait(10)

    sentido = 'ESQ' if ultima_velB < ultima_velA else 'DIR'
    if sentido == 'DIR':
        virarDireita(velocidade_curva)
    else:
        virarEsquerda(velocidade_curva)

    cronometro.reset()
    while cronometro.time() < TIMEOUT_PIVO_MS:
        corEs = sensor_corEs.reflection()
        corDr = sensor_corDr.reflection()
        if corEs < LIMITE_LINHA + 20 or corDr < LIMITE_LINHA + 20:
            break
        wait(5)

    parar_motores()
    wait(60)
    integral = 0
    erro_anterior = 0


# ============================ DESVIO DE OBSTÁCULO ==========================
def desviar_obstaculo():
    global integral, erro_anterior
    parar_motores()
    wait(150)

    virarEsquerda(200)
    wait(500)

    andar(velocidade)
    wait(600)

    virarDireita(200)
    wait(500)

    andar(velocidade)
    wait(900)

    parar_motores()
    wait(150)

    virarDireita(150)
    wait(400)

    parar_motores()
    wait(150)

    virarEsquerda(150)
    wait(300)

    parar_motores()
    ev3.speaker.beep(600)
    integral = 0
    erro_anterior = 0
    # Os wait() aqui são fixos (baseados em tempo), então são o ponto mais
    # frágil do código: teste na pista real e ajuste os valores conforme o
    # tamanho/posição do obstáculo e o peso do seu robô.


# ================================ PID =====================================
def seguirLinha(corEs, corDr):
    global integral, erro_anterior, ultima_velA, ultima_velB

    erroEs = corEs - ALVO
    erroDr = corDr - ALVO
    erro = erroEs - erroDr
    erro_abs = abs(erro)

    proporcional = erro
    integral += erro
    integral = max(-INTEGRAL_MAX, min(INTEGRAL_MAX, integral))  # anti-windup
    derivativo = erro - erro_anterior
    erro_anterior = erro

    vel_base = velocidade - erro_abs * 2
    vel_base = max(VEL_MIN, min(vel_base, velocidade))

    Kp = Kp_base + (erro_abs * 0.01)
    Kd = Kd_base + (erro_abs * 0.02)

    correcao = Kp * proporcional + (Ki * 0.001) * integral + Kd * derivativo

    velA = vel_base + correcao
    velB = vel_base - correcao

    velA = max(-400, min(400, velA))
    velB = max(-400, min(400, velB))

    if DEBUG:
        ev3.screen.print("Es:", int(corEs), "Dr:", int(corDr), "Cor:", int(correcao))

    ultima_velA = velA
    ultima_velB = velB
    motorDr.run(velA)
    motorEs.run(velB)


# ============================== LOOP PRINCIPAL =============================
while True:
    contador_ciclo += 1

    if contador_ciclo % INTERVALO_VERDE == 0:
        atualizar_deteccao_verde()
        wait(TEMPO_CICLO_MS)
        continue

    corEs = sensor_corEs.reflection()
    corDr = sensor_corDr.reflection()

    if sensor_Ir.distance() < distancia_obstaculo:
        desviar_obstaculo()
        wait(TEMPO_CICLO_MS)
        continue

    if verificar_cruzamento(corEs, corDr):
        resolver_cruzamento()
        wait(TEMPO_CICLO_MS)
        continue

    if corEs > LIMITE_BRANCO and corDr > LIMITE_BRANCO:
        tratar_linha_perdida()
        wait(TEMPO_CICLO_MS)
        continue

    if tempo_ultimo_verde is not None and tempo_ultimo_verde.time() > TIMEOUT_VERDE_MS:
        resetar_verdes()

    seguirLinha(corEs, corDr)
    wait(TEMPO_CICLO_MS)

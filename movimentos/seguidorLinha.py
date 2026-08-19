ALVO = 50      

def seguirLinha():
   
    global integral, erro_anterior

    corDr = sensor_corDr.reflection()
    corEs = sensor_corEs.reflection()
    
    erroEs = corEs-ALVO
    erroDr = corDr - ALVO
    
    
    erro = erroEs - erroDr
    erro_abs = abs(erro)

    proporcional = erro 
    integral += erro
    derivativo = erro - erro_anterior
    erro_anterior = erro

   
    vel_base = velocidade - erro_abs * 2
    vel_base = max(150, min(vel_base, velocidade)) 

 
    Kp = 2.5 + (erro_abs * 0.01)  
    Ki = 0     
    Kd = 1.5 + (erro_abs * 0.02)   

    correcao = Kp * proporcional + (Ki * 0.001) * integral + Kd * derivativo

    velA = vel_base + correcao
    velB = vel_base - correcao


    velA = max(-400, min(400, velA))
    velB = max(-400, min(400, velB))
    ev3.screen.print("Erro Dr: ", int(erroDr), " Es: ", int(erroEs))

    motorDr.run(velA)
    motorEs.run(velB)
    
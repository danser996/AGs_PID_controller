import numpy as np
import matplotlib.pyplot as plt
import control as co

def controlador_P(kp):
    num = 0.006679
    den = [1, 0.2547, 0.008125]
    G = co.tf(num, den)
    data_la = co.step_info(G)
    print(f'Informacion de Funcion de transferencia lazo abierto: \n {data_la}')
    response = co.step_response(G)
    plt.plot(response.time, response.outputs, response.time, response.inputs, linewidth = 2)
    plt.title('Funcion de transferencia lazo abierto')
    plt.legend(['output', 'input'])
    plt.xlabel('Time')
    plt.ylabel('Variable')
    plt.grid()
    plt.show()
    ''' Funcion de transferencia realimentada con controlador '''
    numc = [kp]
    denc = [1]
    Gc = co.tf(numc, denc)
    Gt = co.series(Gc, G)
    Gtc = co.feedback(Gt)
    data_controlador = co.step_info(Gtc)
    print(f'Informacion de Funcion de transferencia lazo abierto: \n {data_controlador}')
    parametros = list(data_controlador.values())
    tss = float(parametros[1])
    mp = float(parametros[4])
    response_controlador = co.step_response(Gtc)
    plt.plot(response_controlador.time, response_controlador.outputs, 
    response_controlador.time, response_controlador.inputs, linewidth = 2)
    plt.title('Funcion de transferencia realimentada con controlador')
    plt.xlabel('Time')
    plt.ylabel('Variable')
    plt.legend(['output', 'input'])
    plt.grid()
    plt.show()
    e_ss = abs(1 - response_controlador.outputs[-1])
    return e_ss

def controlador_PI(x_0, x_1):
    num = 0.90888
    den = [52.32874, 1]
    numc = [x_0 * x_1, x_1]
    denc = [x_1, 0]
    
    ''' Funcion de transferencia lazo abierto '''
    G = co.tf(num, den)
    print(G)
    data = co.step_info(G)
    print(f'Informacion de Funcion de transferencia lazo abierto: \n {data}')
    response = co.step_response(G)
    plt.plot(response.time, response.outputs, response.time, response.inputs, linewidth = 2)
    plt.title('Funcion de transferencia lazo abierto')
    plt.legend(['output', 'input'])
    plt.xlabel('Time')
    plt.ylabel('Variable')
    plt.grid()
    plt.show()
    ''' Funcion de transferencia realimentada sin controlador'''
    G_realimentado = co.feedback(G)
    data2 = co.step_info(G_realimentado)
    print(f'Informacion de Funcion de transferencia realimentada sin controlador: \n {data2}')
    response_G = co.step_response(G_realimentado)
    plt.plot(response_G.time, response_G.outputs, response_G.time, response_G.inputs, linewidth = 2)
    plt.title('Funcion de transferencia realimentada sin controlador')
    plt.xlabel('Time')
    plt.ylabel('Variable')
    plt.legend(['output', 'input'])
    plt.grid()
    plt.show()    
    ''' Funcion de transferencia realimentada con controlador '''
    Gc = co.tf(numc, denc)
    print(Gc)
    Gt = co.series(G, Gc)
    print(Gt)
    Gtc = co.feedback(Gt)
    data3 = co.step_info(Gtc)
    print(f'Informacion de Funcion de transferencia realimentada con controlador: \n {data3}')
    response_controlador = co.step_response(Gtc)
    plt.plot(response_controlador.time, response_controlador.outputs, 
    response_controlador.time, response_controlador.inputs, linewidth = 2)
    plt.title('Funcion de transferencia realimentada con controlador')
    plt.xlabel('Time')
    plt.ylabel('Variable')
    plt.legend(['output', 'input'])
    plt.grid()
    plt.show()
    e = sum(abs(response_controlador.inputs - response_controlador.outputs))
    return e

def controlador_PID(kp, ki, kd):
    num = 0.006679
    den = [1, 0.2547, 0.008125]
    G = co.tf(num, den)
    # print(f'Informacion de Funcion de transferencia lazo abierto: \n {data}')
    response = co.step_response(G)
    plt.plot(response.time, response.outputs, response.time, response.inputs, linewidth = 2)
    plt.title('Funcion de transferencia lazo abierto')
    plt.legend(['output', 'input'])
    plt.xlabel('Time')
    plt.ylabel('Variable')
    plt.grid()
    plt.show()
    ''' Funcion de transferencia realimentada con controlador '''
    numc = [kd,  kp, ki]
    denc = [1, 0]
    Gc = co.tf(numc, denc)
    Gt = co.series(Gc, G)
    Gtc = co.feedback(Gt)
    data_controlador = co.step_info(Gtc)
    print(f'Informacion de Funcion de transferencia lazo abierto: \n {data_controlador}')
    parametros = list(data_controlador.values())
    tss = float(parametros[1])
    mp = float(parametros[4])
    response_controlador = co.step_response(Gtc)
    plt.plot(response_controlador.time, response_controlador.outputs, 
    response_controlador.time, response_controlador.inputs, linewidth = 2)
    plt.title('Funcion de transferencia realimentada con controlador')
    plt.xlabel('Time')
    plt.ylabel('Variable')
    plt.legend(['output', 'input'])
    plt.grid()
    plt.show()
    e_ss = abs(1 - response_controlador.outputs[-1])
    e = (80 - tss) + (1.1 - mp) + e_ss
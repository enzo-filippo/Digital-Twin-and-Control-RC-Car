import matplotlib.pyplot as plt
import os
import numpy as np

# Função para ler os dados de um arquivo .txt
def ler_arquivo(filename):
    data = {}
    with open(filename, 'r') as file:
        for line in file:
            if line.strip():  # Ignora linhas em branco
                key, values = line.strip().split(':')
                data[key.strip()] = list(map(float, values.split()))
    print(type(data['abscisse']))
    t = np.array(data['abscisse'])
    pos = np.array(data['positions_barycentre_corr'])
    x = pos[:int((np.floor(len(pos)/2)))]/100
    y = pos[int((np.floor(len(pos)/2))):]/100
    v = np.array(data['liste_vitesse_long_corr'])/100
    a = np.array(data['acceleration_corr'])/100
    # print(len(t))
    # print(len(x))
    # print(len(y))
    # print(len(v))
    # print(len(a))
    return t, x, y, v, a

def plot(x,y, labelx, labely):
    plt.xlabel(labelx)
    plt.ylabel(labely)
    plt.grid(True)
    # axis('equal')
    lw = 1
    plt.plot(x, y, 'b', linewidth=lw)
# Nome do arquivo .txt

def run(throttlereal):
    filename = os.path.join('data',str(throttlereal)+'.txt')

    # Ler os dados do arquivo
    t, x, y, v, a = ler_arquivo(filename)
    psi0 = np.degrees(np.arctan((y[-1]-y[0])/(x[-1]-x[0])))
    print("tempo max: ", np.max(t)-np.min(t))
    print("psi0: ", psi0)
    print("tamanho_x: ", len(x))
    print("x inicial: ", x[0])
    print("y inicial: ", y[0])

    # plot(x,y,'x [m]','y [m]')
    plot(t,v, 'tempo [s]', 'velocidade [m/s]')
    # plot(t,a, 'tempo [s]','aceleração [m/s²]')
    return t, x, y, v, a

    plt.show()

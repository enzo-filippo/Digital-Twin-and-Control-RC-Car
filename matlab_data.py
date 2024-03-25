import matplotlib.pyplot as plt

# Função para ler os dados de um arquivo .txt
def ler_arquivo(filename):
    data = {}
    with open(filename, 'r') as file:
        for line in file:
            if line.strip():  # Ignora linhas em branco
                key, values = line.strip().split(':')
                data[key.strip()] = list(map(float, values.split()))
    return data

# Função para plotar os dados
def plotar_dados(data):
    for key, values in data.items():
        plt.figure()
        plt.plot(values)
        plt.title(key)
        plt.xlabel('Índice')
        plt.ylabel('Valor')
        plt.grid(True)
        

# Nome do arquivo .txt
filename = 'dados.txt'

# Ler os dados do arquivo
dados = ler_arquivo(filename)

# Plotar os dados
plotar_dados(dados)
plt.show()

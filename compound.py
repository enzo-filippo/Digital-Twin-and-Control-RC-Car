import numpy as np


quantidade = 1600
juros = 6.5/12 # em %
tempo = 240
total = quantidade*(1+juros/100)**tempo 
print("Juros compostos: R$", round(total,2))

#################

taxa = 6.5 # em %
prazo_que_quero = 1
prazo_que_tenho = 12
taxa_equivalente = ((1+ (taxa/100))**(prazo_que_quero/prazo_que_tenho)-1)*100
print("Taxa equivalente:",round(taxa_equivalente,4), "%")

# ##################

quantidade = 1600 # investido por mês
juros = 6.5 # em % a.a.
tempo = 240 # meses
total = quantidade*(1-(1+(juros/100)/12)**(-tempo))/((juros/100)/12)
print("Juros malucos: R$", round(total,2))


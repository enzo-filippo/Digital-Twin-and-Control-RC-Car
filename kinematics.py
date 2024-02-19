## Bycicle model - consider only two wheels 
# https://minesparis-psl.hal.science/hal-01473160/document

import numpy as np

class Bicileta:
    def __init__(self, m_CG, momento_INERCIA_CG, distancia_roda_traseira_CG, distancia_roda_dianteira_CG, p_Longi_CG, p_Perpe_CG, v_Longi_CG, v_Perpe_CG, a_Longi_CG_Veiculo, a_Perpe_CG, ang_Bicicleta, ang_v_Bicileta, comprimento_eixo, ang_guinada_ponto):
        # Psi = ang_Bicicleta 
        # lr = distancia_roda_traseira_CG
        # lf = distancia_roda_dianteira_CG
        

        self.m_CG = m_CG
        self.momento_INERCIA_CG = momento_INERCIA_CG
        self.distancia_roda_traseira_CG = distancia_roda_traseira_CG 
        self.distancia_roda_dianteira_CG = distancia_roda_dianteira_CG
        self.p_Longi_CG = p_Longi_CG
        self.p_Perpe_CG = p_Perpe_CG
        self.v_Longi_CG = v_Longi_CG
        self.v_Perpe_CG = v_Perpe_CG
        self.a_Longi_CG_Veiculo = a_Longi_CG_Veiculo
        self.a_Perpe_CG = a_Perpe_CG
        self.ang_Bicicleta = ang_Bicicleta
        self.ang_v_Bicileta = ang_v_Bicileta
        self.comprimento_eixo = comprimento_eixo
        self.ang_guinada_ponto = ang_guinada_ponto
        
    def Dinamica(self):
        

class Pneu:
    def __init__(self,distancia_Pneu_CG,raio_Efetivo_Pneu,F_Apoio_Pneu,C_Rigidez_Longi_Pneu,Coef_Rigidez_Deriva_Pneu,Coef_Atrito,p_Longi_Pneu,p_Perpe_Pneu,v_Longi_Pneu,v_Perpe_Pneu,a_Longi_CG_Veiculo_Pneu,a_Perpe_Pneu,v_Angular_Pneu,ang_Pneu,ang_v_Roda,diff_Ang_Pneu_Ang_v_Pneu):
        #delta_pneu = ang_Pneu
        self.distancia_Pneu_CG = distancia_Pneu_CG
        self.raio_Efetivo_Pneu = raio_Efetivo_Pneu
        self.F_Apoio_Pneu = F_Apoio_Pneu
        self.C_Rigidez_Longi_Pneu = C_Rigidez_Longi_Pneu
        self.Coef_Rigidez_Deriva_Pneu = Coef_Rigidez_Deriva_Pneu
        self.Coef_Atrito = Coef_Atrito
        self.p_Longi_Pneu = p_Longi_Pneu
        self.p_Perpe_Pneu = p_Perpe_Pneu
        self.v_Longi_Pneu = v_Longi_Pneu
        self.v_Perpe_Pneu = v_Perpe_Pneu
        self.a_Longi_CG_Veiculo_Pneu = a_Longi_CG_Veiculo_Pneu
        self.a_Perpe_Pneu = a_Perpe_Pneu
        self.v_Angular_Pneu = v_Angular_Pneu
        self.ang_Pneu = ang_Pneu
        self.ang_v_Roda = ang_v_Roda
        self.diff_Ang_Pneu_Ang_v_Pneu = diff_Ang_Pneu_Ang_v_Pneu
       
    

    def Dinamica(self,delta_pneu,roda,comprimento_eixo, ang_guinada_ponto,v_Longi_CG_Veiculo, a_Longi_CG_Veiculo): 
        
        ## Taxa Deslizamento -----------------------------
        
        # reff = self.raio_Efetivo_Pneu
        # vx = self.v_Longi_CG_Veiculo
        # a_x = self.a_Longi_CG_Veiculo
        # omega = self.v_Angular_Pneu
        # tau_x = taxa_Desliz_Longi 
        
        if a_Longi_CG_Veiculo > 0: ## glissement positif
            self.taxa_Desliz_Longi = (self.raio_Efetivo_Pneu * self.v_Angular_Pneu - v_Longi_CG_Veiculo)/(self.raio_Efetivo_Pneu * self.v_Angular_Pneu)
        elif a_Longi_CG_Veiculo < 0: ## glissement negatif
            self.taxa_Desliz_Longi = (self.raio_Efetivo_Pneu * self.v_Angular_Pneu - v_Longi_CG_Veiculo)/(abs(v_Longi_CG_Veiculo))
        else:
            self.taxa_Desliz_Longi = 0

        ## DUGOFF -----------------------------
        
        # C_tau_i = self.C_Rigidez_Longi_Pneu
        # C_alpha_i = self.Coef_Rigidez_Deriva_Pneu
        # alpha = self.diff_Ang_Pneu_Ang_v_Pneu
        # tau_x = self.taxa_Desliz_Longi
        # mu = self.Coef_Atrito
        # F_zy = self.F_Apoio_Pneu
        # F_xpi = self.F_Longi_Pneu
        # F_ypi = self.F_Lateral_Pneu
        
        sinal_numerador = np.array([-1, 1, 1, -1])
        sinal_denominador = np.array([1, 1, -1, -1])
        numerador = self.v_Perpe_Pneu + sinal_numerador(roda)*self.distancia_Pneu_CG * ang_guinada_ponto
        denominador = self.v_Longi_Pneu + sinal_denominador(roda)*comprimento_eixo/2 * ang_guinada_ponto

        self.ang_v_pneu = np.arctan(numerador/denominador)
        self.diff_Ang_Pneu_Ang_v_Pneu = delta_pneu - self.ang_v_pneu
        
        self.coef_lambda = (self.Coef_Atrito * self.F_Apoio_Pneu*(1+self.taxa_Desliz_Longi))/(2*np.sqrt(self.C_Rigidez_Longi_Pneu*self.taxa_Desliz_Longi)** + self.Coef_Rigidez_Deriva_Pneu*np.tan(self.diff_Ang_Pneu_Ang_v_Pneu)**2) 
        if self.coef_lambda < 1:
            self.f_lambda = (2-self.coef_lambda)*self.coef_lambda
        else:
            self.f_lambda = 1    
        
        self.F_Longi_Pneu = self.Coef_Rigidez_Deriva_Pneu *(self.taxa_Desliz_Longi/(1-self.taxa_Desliz_Longi))*self.f_lambda
        self.F_Lateral_Pneu = self.C_Rigidez_Longi_Pneu*(np.tan(self.diff_Ang_Pneu_Ang_v_Pneu)/(1-self.taxa_Desliz_Longi))*self.f_lambda

        self.F_Longi_Pneu_Cord_Bike = self.F_Longi_Pneu*np.cos(delta_pneu)-self.F_Lateral_Pneu*np.sin(delta_pneu)
        self.F_Lateral_Pneu_Cord_Bike = self.F_Longi_Pneu*np.sin(delta_pneu)+self.F_Lateral_Pneu*np.cos(delta_pneu)
        return self.F_Longi_Pneu_Cord_Bike, self.F_Lateral_Pneu_Cord_Bike

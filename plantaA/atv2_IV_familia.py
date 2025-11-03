import numpy as np
import matplotlib.pyplot as plt
import control as clt
from control import tf, feedback, bode_plot

# --- 1. Definição do Controlador Fixo (Projeto de 1ª Ordem) ---
# Parâmetros nominais
K_nominal = 0.20
tau_nominal = 20.0
theta_nominal = 5.0
ts_desejado = 60.0
tau_d = ts_desejado / 4.6

# Controlador PI Fixo
Kc = tau_nominal / (K_nominal * (tau_d + theta_nominal))
Ti = tau_nominal
C = Kc * (1 + tf([1], [Ti, 0]))

print("--- Gerando Análise de Frequência da Família (Hipercubo 3D) ---")
print("Controlador Fixo C(s):")
print(C)

# --- 2. Definição das Faixas do Hipercubo (8 Vértices) ---
K_range = [0.15, 0.25]      # [K_min, K_max]
tau_range = [15.0, 25.0]    # [tau_min, tau_max]
theta_range = [3.0, 5.0]    # [theta_min, theta_max]

# --- 3. Geração dos Gráficos de Frequência ---

# Define a faixa de frequência para os plots de Bode
omega = np.logspace(-3, 1, 500)

# Figura 1: Família de Malha Aberta (L = C*G)
plt.figure(figsize=(10, 8))
plt.suptitle('Análise de Frequência da Família (8 Vértices) - MALHA ABERTA', fontsize=16)

# Figura 2: Família de Malha Fechada (Y/R)
plt.figure(figsize=(10, 8))
plt.suptitle('Análise de Frequência da Família (8 Vértices) - MALHA FECHADA', fontsize=16)

# --- CORREÇÃO: Loop triplo para os 8 vértices ---
for K_val in K_range:
    for tau_val in tau_range:
        for theta_val in theta_range:
            
            # Cria a planta "incerta"
            # CORREÇÃO: Pade usa theta_val, G usa K_val e tau_val
            num_pade, den_pade = clt.pade(theta_val, 4) 
            G_incerta = clt.series(tf([K_val], [tau_val, 1]), tf(num_pade, den_pade))

            # --- Plot de Malha Aberta ---
            L_incerta = C * G_incerta
            plt.figure(1) # Seleciona a Figura 1
            # CORREÇÃO: Label inclui todos os 3 parâmetros
            bode_plot(L_incerta, omega=omega, dB=True, label=f'K={K_val}, T={tau_val}, th={theta_val}', alpha=0.7)
            # --- Plot de Malha Fechada ---
            Gmf_incerta = feedback(L_incerta, 1)
            plt.figure(2) # Seleciona a Figura 2
            bode_plot(Gmf_incerta, omega=omega, dB=True, label=f'K={K_val}, T={tau_val}, th={theta_val}', alpha=0.7)
# Adiciona legendas e grids às figuras
plt.figure(1)
plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize='small')
plt.tight_layout(rect=[0, 0.03, 1, 0.95]) # Ajusta para o supertítulo

plt.figure(2)
plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize='small')
plt.tight_layout(rect=[0, 0.03, 1, 0.95]) # Ajusta para o supertítulo

plt.show()
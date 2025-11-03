import numpy as np
import matplotlib.pyplot as plt
import control as clt
from control import tf, feedback, step_response

# item 1

# Parâmetros
t_final = 300
y_final = 150  # Ajuste baseado no novo sistema

# Sistema G(s) = 100e^(-7s) / (14s + 1)(21s + 1)
# Parâmetros da Planta B
K = 100.0
tau1 = 14.0
tau2 = 21.0
theta = 7.0
t_acomodacao= theta + tau2*4.6
t_acomodacao_desejado = 0.7*t_acomodacao
tau_c = (t_acomodacao_desejado) / 4.6

# Define a variável 's'
s = clt.tf('s')

# Parte dinâmica da planta (sem o atraso)
G_dinamica = K / ((tau1*s + 1) * (tau2*s + 1))

# Cria uma aproximação de Padé de 1ª ordem para o atraso e^(-7s)
num_pade, den_pade = clt.pade(theta, 1)
G_atraso_aprox = clt.tf(num_pade, den_pade)

# A função de transferência em malha aberta APROXIMADA é a série das duas
G_ma_aprox = clt.series(G_dinamica, G_atraso_aprox)
G_ma_aprox = clt.minreal(G_ma_aprox) # Simplifica a TF

# controlador de 1a ordem
Kc_1 = tau1 / (K * (tau_c + theta))
C_1 = Kc_1 * (1 + tf([1], [tau1, 0])) * (tau2 * s + 1)

# controlador de 2a ordem
zeta = 0.9
Kc_2 = tau1 / (K * (2*zeta*tau_c + theta))
C_2 = Kc_2 * (1 + tf([1], [tau1, 0]))*(tau2*s + 1)*(1 /((tau_c**2/(2*zeta*tau_c+theta))*s+1))

# malha fechada
Gmf_1 = feedback(C_1 * G_ma_aprox, 1)
Gmf_2 = feedback(C_2 * G_ma_aprox, 1)

# Simulação
t = np.linspace(0, t_final, 1000)
t, y1 = step_response(Gmf_1, t)
t, y2 = step_response(Gmf_2, t)
t, y3 = step_response(G_ma_aprox, t)

# Gráfico Malha Aberta
plt.figure()
plt.plot(t, y3, label="G(s) (Planta) Malha Aberta")
plt.axhline(K+K*0.02, color="black", linestyle="--", label="Erro mínimo 2%")
plt.axhline(K-K*0.02, color="black", linestyle="--", label="Erro máximo 2%")
plt.axvline(t_acomodacao, color='purple', linestyle='--', label='Limite Acomodação')
plt.grid()
plt.xlabel("Tempo (s)")
plt.ylabel("Saída")
plt.legend()
plt.axis([0, t_final, 0, y_final])
plt.show()

# Gráfico Malha Fechada controlador 1a ordem
plt.figure()
plt.plot(t, y1, label="Gmf (Malha Fechada)")
plt.axhline(1.02, color="black", linestyle="--", label="Erro mínimo 2%")
plt.axhline(0.98, color="black", linestyle="--", label="Erro máximo 2%")
plt.axvline(t_acomodacao_desejado, color='purple', linestyle='--', label='Acomodação Desejada')
plt.grid()
plt.xlabel("Tempo (s)")
plt.ylabel("Saída")
plt.legend()
plt.axis([0, t_final, 0, 1.5])
plt.show()

# Gráfico Malha Fechada controlador 2a ordem
plt.figure()
plt.plot(t, y2, label="Gmf (Malha Fechada) - Controlador 2ª Ordem")
plt.axhline(1.02, color="black", linestyle="--", label="Erro mínimo 2%")
plt.axhline(0.98, color="black", linestyle="--", label="Erro máximo 2%")
plt.axvline(t_acomodacao_desejado, color='purple', linestyle='--', label='Acomodação Desejada')
plt.grid()
plt.xlabel("Tempo (s)")
plt.ylabel("Saída")
plt.legend()
plt.axis([0, t_final, 0, 1.5])
plt.show()

# Análise dos resultados
info = clt.step_info(G_ma_aprox, T=t)
    
t_acomodacao_real_ma = info['SettlingTime']
print(f"Tempo de acomodação MA calculado: {t_acomodacao_real_ma:.2f} s")

# Cálculo do tempo de acomodação e overshoot para o sistema com controlador 1a ordem
try:
    info = clt.step_info(Gmf_1, T=t)
    
    t_acomodacao_real = info['SettlingTime']
    overshoot_real = info['Overshoot']
    
    if np.isnan(t_acomodacao_real):
        print(f"O sistema NÃO acomodou dentro dos {t_final}s da simulação.")
    else:
        print(f"Tempo de acomodação 1a ordem (2%) calculado: {t_acomodacao_real:.2f} s")
        
    print(f"Sobressinal 1a ordem (Overshoot) calculado: {overshoot_real:.2f} %")

except Exception as e:
    print(f"Não foi possível calcular o step_info: {e}")
    print("Isso pode acontecer se o sistema for instável.")




try:
    info = clt.step_info(Gmf_2, T=t)
    
    t_acomodacao_real2 = info['SettlingTime']
    overshoot_real2 = info['Overshoot']
    
    if np.isnan(t_acomodacao_real2):
        print(f"O sistema NÃO acomodou dentro dos {t_final}s da simulação.")
    else:
        print(f"Tempo de acomodação 2a ordem calculado: {t_acomodacao_real2:.2f} s")
        
    print(f"Sobressinal 2a ordem (Overshoot) calculado: {overshoot_real2:.2f} %")

except Exception as e2:
    print(f"Não foi possível calcular o step_info: {e2}")
    print("Isso pode acontecer se o sistema for instável.")



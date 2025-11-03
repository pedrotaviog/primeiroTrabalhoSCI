import numpy as np
import matplotlib.pyplot as plt
import control as clt
from control import tf, feedback, step_response, series
import itertools

# item 2 - Validação do controlador projetado

# Intervalos de variação (faixa de incertezas)
K_values = np.linspace(0.15, 0.25, 5)      # ganho
tau_values = np.linspace(15, 25, 5)      # constante de tempo
theta_values = np.linspace(3, 5, 5)      # atraso

# Tempo de simulação
t_final = 200
t = np.linspace(0, t_final, 200)


plt.figure()
plt.title("Respostas da Família de Sistemas com Mesmo Controlador")
plt.grid(True)

K = 0.20
tau = 20
theta = 5  # Atraso de 0.5s

# Aproximação de Padé para o atraso
num_pade, den_pade = clt.pade(theta, 1)

# Definição de G2
G2 = tf([K], [tau, 1])
G2a = clt.series(clt.tf(num_pade, den_pade), G2)

# Especificação de malha fechada Gd = 1 / (tau_d s + 1)
tau_d = 20/4.6  # 60s de acomodação e 20s para tornar mais agressivo
Gd = tf([1], [tau_d, 1])
Gda = series(clt.tf(num_pade, den_pade), Gd)

# Controlador especificado
Kc = tau / (K * (tau_d + theta))
Ti = tau
C = Kc* (1 + tf([1], [Ti, 0]))

# Loop de variação
for K, tau, theta in itertools.product(K_values, tau_values, theta_values):
    # Planta com atraso (Pade 1ª ordem)
    num_pade, den_pade = clt.pade(theta, 1)
    G = series(tf([K], [tau, 1]), tf(num_pade, den_pade))
    
    # Malha fechada
    Gmf = feedback(C * G, 1)
    
    # Simula resposta
    t_out, y = step_response(Gmf, t)
    
    # Cor depende do ganho K (só pra diferenciar um pouco)
    color = plt.cm.viridis((K - min(K_values)) / (max(K_values) - min(K_values)))
    plt.plot(t_out, y, color=color, alpha=0.6)

# ============================
# Resposta desejada (especificação)
# ============================
num_pade, den_pade = clt.pade(theta, 1)
Gd = tf([1], [tau_d, 1])
Gda = series(tf(num_pade, den_pade), Gd)
t_out, y_ref = step_response(Gda, t)
plt.plot(t_out, y_ref, 'k--', linewidth=2, label="Gd (Especificação)")

# Linhas de erro máximo/mínimo
plt.axhline(1.02, color="green", linestyle="--", label="Erro máximo (2%)")
plt.axhline(0.98, color="blue", linestyle="--", label="Erro mínimo (2%)")
plt.axhline(1.05, color='red', linestyle='--', label='Limite Sobressinal (5%)')
plt.axvline(60, color='purple', linestyle='--', label='Limite Acomodação (60s)')
plt.xlabel("Tempo (s)")
plt.ylabel("Saída")
plt.legend()
plt.axis([0, t_final, 0, 1.5])
plt.show()
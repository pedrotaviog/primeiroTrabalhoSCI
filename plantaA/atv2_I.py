import numpy as np
import matplotlib.pyplot as plt
import control as clt
from control import tf, feedback, step_response, series
import os

# item 1

# Parâmetros
t_final = 300 
y_final = 1.5  # Ajuste baseado no novo sistema

# Sistema G(s) = 0.20e^(-5s) / (20s + 1)
K = 0.20
tau = 20 #
theta = 5  # Atraso de 0.5s

# Aproximação de Padé para o atraso
num_pade, den_pade = clt.pade(theta, 1)

# Definição de G2
G2 = tf([K], [tau, 1])
G2a = clt.series(clt.tf(num_pade, den_pade), G2)

# Especificação de malha fechada Gd = 1 / (tau_d s + 1)
tau_d = 60/4.6  # 60s de tempo de acomodação para 2% (4.6*tau_d)
Gd = tf([1], [tau_d, 1])
Gda = series(clt.tf(num_pade, den_pade), Gd)

# Controlador especificado
Kc = tau / (K * (tau_d + theta))
Ti = tau
C = Kc * (1 + tf([1], [Ti, 0]))

# Malha fechada
Gmf = feedback(C * G2a, 1)

# Simulação
t = np.linspace(0, t_final, 1000)
t, y1 = step_response(Gmf, t)
t, y2 = step_response(Gda, t)
t, y3 = step_response(G2a, t)
# Gráficos
plt.figure()
plt.plot(t, y1, label="Gmf (Malha Fechada)")
plt.plot(t, y2, label="Gd (Especificação)")

plt.axhline(0.98, color="black", linestyle="--", label="Erro mínimo")
plt.axhline(1.02, color="black", linestyle="--", label="Erro máximo")
plt.axhline(1.05, color='red', linestyle='--', label='Limite Sobressinal (5%)')
plt.axvline(60, color='purple', linestyle='--', label='Limite Acomodação (60s)')
plt.grid()
plt.legend()
plt.axis([0, t_final, 0, y_final])
plt.title("Sistema com G(s)")
plt.show()

#---------------------------------------------------------------------------------------------------




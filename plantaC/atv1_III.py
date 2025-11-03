import control as ct
import matplotlib.pyplot as plt
import numpy as np

s = ct.tf('s')
G = -8/(s**2 + 1.5*s - 1)

Kp = 5.0
Ti = 1.2
Td = 0.2
N = 10    # filtro derivativo

# PID com filtro derivativo
C = -(Kp * (1 + 1/(Ti*s) + (Td*s)/(1 + (Td/N)*s)))

L = C * G
T = ct.feedback(L, 1)

t = np.linspace(0, 5, 2000)
t, y = ct.step_response(T, T=t)

plt.figure()
plt.plot(t, y)
plt.grid(True)
plt.xlabel('Tempo [s]')
plt.ylabel('Saída y(t)')
plt.title(f'Resposta ao degrau – PID filtrado: Kp={Kp}, Ti={Ti}, Td={Td}, N={N}')
plt.show()

# Sinal de controle u(t)
U_R = ct.minreal(C / (1 + C*G))
t, u = ct.step_response(U_R, T=t)

plt.figure()
plt.plot(t, u)
plt.grid(True)
plt.xlabel('Tempo [s]')
plt.ylabel('Sinal de controle u(t)')
plt.title('Sinal de controle u(t)')
plt.show()

print(ct.poles(T))
y_final = y[-1]
print("Valor final da saída:", y_final)
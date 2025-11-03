import control as ct
import matplotlib.pyplot as plt
import numpy as np

# --- Planta C ---
s = ct.tf('s')
G = -8/(s**2 + 1.5*s - 1)

# --- Controlador PD (Item I) ---
Td = 0.2
Kpd = 10.390625   # valor do fechamento do arco
C_PD = -(Kpd*(1 + Td*s))

# --- Controlador PID filtrado (Item III) ---
Kp, Ti, Td_pid, N = 5.0, 1.2, 0.2, 10
C_PID = -(Kp*(1 + 1/(Ti*s) + (Td_pid*s)/(1 + (Td_pid/N)*s)))

# --- Malhas fechadas ---
T_PD  = ct.feedback(C_PD*G, 1)
T_PID = ct.feedback(C_PID*G, 1)

# --- Tempo e entradas ---
t = np.linspace(0, 1000, 3000)
r_step = np.ones_like(t)
r_ramp = t

# --- Respostas: degrau ---
t, y_step_PD  = ct.step_response(T_PD,  T=t)
t, y_step_PID = ct.step_response(T_PID, T=t)

# --- Respostas: rampa (forçada) ---
t, y_ramp_PD  = ct.forced_response(T_PD,  T=t, U=r_ramp)
t, y_ramp_PID = ct.forced_response(T_PID, T=t, U=r_ramp)

# --- Plot 1: degrau (PD x PID) ---
plt.figure()
plt.plot(t, y_step_PD,  label='PD (Item I)')
plt.plot(t, y_step_PID, label='PID (Item III)')
plt.grid(True); plt.legend()
plt.xlabel('Tempo [s]'); plt.ylabel('Saída y(t)')
plt.title('Entrada degrau – comparação PD x PID')

# --- Plot 2: rampa (PD x PID) ---
plt.figure()
plt.plot(t, r_ramp, '--', color='black', alpha=0.6, label='Referência rampa')
plt.plot(t, y_ramp_PD,  label='PD (Item I)')
plt.plot(t, y_ramp_PID, label='PID (Item III)')
plt.grid(True); plt.legend()
plt.xlabel('Tempo [s]'); plt.ylabel('Saída y(t)')
plt.title('Entrada rampa – comparação PD x PID')
plt.show()

# --- Checagens numéricas de erro estacionário ---
# Degrau:
e_step_PD_final  = 1 - y_step_PD[-1]
e_step_PID_final = 1 - y_step_PID[-1]
print("e_ss (degrau) PD  aprox.", e_step_PD_final)
print("e_ss (degrau) PID aprox.", e_step_PID_final)

# Rampa: estima erro "em regime" com média nos 20% finais
idx = int(0.8*len(t))
e_ramp_PD  = r_ramp - y_ramp_PD
e_ramp_PID = r_ramp - y_ramp_PID
print("e_ss (rampa) PD  ~ cresce (tipo 0)  => media ultimos 20%:", e_ramp_PD[idx:].mean())
print("e_ss (rampa) PID aprox. constante (tipo 1) => media ultimos  20%:", e_ramp_PID[idx:].mean())


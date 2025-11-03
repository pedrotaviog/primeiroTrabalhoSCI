import control as ct
import matplotlib.pyplot as plt
import numpy as np

# --- Planta C ---
s = ct.tf('s')
G = -8 / (s**2 + 1.5*s - 1)

# --- Controlador PD (Item I) ---
Td = 0.2
Kpd = 10.390625   # ganho correspondente ao fechamento do arco
C_PD = -(Kpd * (1 + Td*s))

# --- Controlador PID filtrado (Item III) ---
Kp, Ti, Td_pid, N = 5.0, 1.2, 0.2, 10
C_PID = -(Kp * (1 + 1/(Ti*s) + (Td_pid*s)/(1 + (Td_pid/N)*s)))

# --- Malhas abertas e fechadas ---
L_PD  = C_PD * G
L_PID = C_PID * G
T_PD  = ct.feedback(L_PD, 1)
T_PID = ct.feedback(L_PID, 1)

# --- Frequência logarítmica ---
omega = np.logspace(-1, 2, 600)  # 0.1 a 100 rad/s

# --- Resposta em frequência da malha fechada ---
mag_PD, phase_PD, omega = ct.frequency_response(T_PD, omega)
mag_PID, phase_PID, _   = ct.frequency_response(T_PID, omega)

# --- Plot Bode comparativo ---
plt.figure(figsize=(8,6))

# Magnitude
plt.subplot(2,1,1)
plt.semilogx(omega, 20*np.log10(mag_PD), label='PD (Item I)')
plt.semilogx(omega, 20*np.log10(mag_PID), label='PID (Item III)')
plt.ylabel('Magnitude [dB]')
plt.grid(True, which='both')
plt.legend()

# Fase
plt.subplot(2,1,2)
plt.semilogx(omega, np.degrees(phase_PD), label='PD (Item I)')
plt.semilogx(omega, np.degrees(phase_PID), label='PID (Item III)')
plt.xlabel('Frequência [rad/s]')
plt.ylabel('Fase [graus]')
plt.grid(True, which='both')
plt.legend()

plt.suptitle('Comparação em frequência – T(s) = Y(s)/R(s)')
plt.tight_layout()
plt.show()

# --- Margens de estabilidade ---
gm_PD, pm_PD, wg_PD, wp_PD = ct.margin(L_PD)
gm_PID, pm_PID, wg_PID, wp_PID = ct.margin(L_PID)

# Exibição dos resultados
print("\n=== Margens de Estabilidade ===")
print(f"PD (Item I):  Margem de fase = {pm_PD:.2f}°,  Margem de ganho = {20*np.log10(gm_PD):.2f} dB")
print(f"PID (Item III): Margem de fase = {pm_PID:.2f}°,  Margem de ganho = {20*np.log10(gm_PID):.2f} dB")


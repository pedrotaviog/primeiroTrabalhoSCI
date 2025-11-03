import matplotlib.pyplot as plt
import control as clt
from control import tf, feedback
import os

# Diretório deste script (garante que arquivos gerados fiquem no mesmo local)
here = os.path.dirname(os.path.abspath(__file__))
# --- Parâmetros do sistema ---
K = 0.20
tau = 20.0
theta = 5.0  # atraso em segundos

# Aproximação de Padé 1ª ordem
num_pade, den_pade = clt.pade(theta, 1)
G2 = tf([K], [tau, 1])
G2a = clt.series(tf(num_pade, den_pade), G2)

# --- Controlador PI ---
tau_d = 60/4.6  # TS desejado de 60s (±2%)
Kc = tau / (K * (tau_d + theta))
Ti = tau
C = Kc * (1 + tf([1], [Ti, 0]))

# --- Malha aberta ---
L = C * G2a

# Bode plot da malha aberta
plt.figure(figsize=(10,6))
_ = clt.bode_plot(L, dB=True, Hz=False)
plt.suptitle("Bode da Malha Aberta (L = C(s)*G(s))", fontsize=16)
plt.show()

# Margens de estabilidade
try:
    gm, pm, Wcg, Wcp = clt.margin(L)
    print(f"Margem de Ganho (GM): {gm:.2f}")
    print(f"Margem de Fase (PM): {pm:.2f} graus")
    print(f"Freq. Cruzamento de Ganho (Wcg): {Wcg:.3f} rad/s")
    print(f"Freq. Cruzamento de Fase (Wcp): {Wcp:.3f} rad/s")
except:
    print("Não foi possível calcular margens. Tente Padé de 1ª ordem e minreal(L).")

# --- Malha fechada ---
Gmf = feedback(L, 1)

plt.figure(figsize=(10,6))
_ = clt.bode_plot(Gmf, dB=True, Hz=False)
plt.suptitle("Bode da Malha Fechada (Y/R)", fontsize=16)
plt.show()



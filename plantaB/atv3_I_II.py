import numpy as np
import matplotlib.pyplot as plt
import control as clt
from control import tf, feedback, step_response

# --- 0. Definições e Controladores (Seu código original) ---
K = 100.0
tau1 = 14.0
tau2 = 21.0
theta = 7.0
t_acomodacao_ma = theta + tau2*4.6
t_acomodacao_desejado = 0.7 * t_acomodacao_ma
tau_c = (t_acomodacao_desejado) / 4.6
t_final = 300
t = np.linspace(0, t_final, 1000)
dt = t[1] - t[0] # Intervalo de tempo para integração

s = clt.tf('s')

# Planta (G_ma_aprox)
G_dinamica = K / ((tau1*s + 1) * (tau2*s + 1))
num_pade, den_pade = clt.pade(theta, 1)
G_atraso_aprox = clt.tf(num_pade, den_pade)
G_ma_aprox = clt.series(G_dinamica, G_atraso_aprox)
G_ma_aprox = clt.minreal(G_ma_aprox)

# Controlador 1
Kc_1 = tau1 / (K * (tau_c + theta))
C_1 = Kc_1 * (1 + tf([1], [tau1, 0])) * (tau2 * s + 1)
C_1.name = "C1 (Alvo 1a Ordem)"

# Controlador 2
zeta = 0.9
Kc_2 = tau1 / (K * (2*zeta*tau_c + theta))
filtro_C2 = (1 /((tau_c**2/(2*zeta*tau_c+theta))*s+1))
C_2 = Kc_2 * (1 + tf([1], [tau1, 0]))*(tau2*s + 1)*filtro_C2
C_2.name = "C2 (Alvo 2a Ordem)"

# --- 1. Conclusão do Item I: Sinal de Controle e Índices ---

print("--- Análise do Item I (Sinal de Controle e Índices) ---")

# Simulação da Saída Y(t)
Gmf1 = feedback(C_1 * G_ma_aprox, 1)
Gmf2 = feedback(C_2 * G_ma_aprox, 1)
_, y1 = step_response(Gmf1, t)
_, y2 = step_response(Gmf2, t)

# Sinal de Erro
e1 = 1 - y1
e2 = 1 - y2


# --- Sistema 1: C1 = Kc1 * [PI] * [D] ---
# v1(t) é a saída do bloco PI
v1 = e1 + (1/tau1) * np.cumsum(e1) * dt
# u1(t) é a saída do bloco D, que age sobre v1(t)
u1 = Kc_1 * ( (tau2 * np.gradient(v1, t)) + v1 )


# --- Sistema 2: C2 = Kc2 * [PI] * [D] * [Filtro] ---
# v2(t) é a saída do bloco PI
v2 = e2 + (1/tau1) * np.cumsum(e2) * dt
# w2(t) é a saída do bloco D, que age sobre v2(t)
w2 = Kc_2 * ( (tau2 * np.gradient(v2, t)) + v2 )
# u2(t) é a saída do Filtro, que age sobre w2(t)
Tf = (tau_c**2/(2*zeta*tau_c+theta))
G_filtro = tf([1], [Tf, 1])
T_out, u2 = clt.forced_response(G_filtro, T=t, U=w2)


# Plot do Sinal de Controle U(t)
plt.figure(figsize=(10, 6))
plt.plot(t, u1, label="Sinal de Controle U(t) - Projeto 1")
plt.plot(t, u2, label="Sinal de Controle U(t) - Projeto 2", linestyle='--')
plt.title("Comparação do Sinal de Controle U(t)")
plt.xlabel("Tempo (s)")
plt.ylabel("Sinal de Controle")
plt.legend()
plt.grid(True)

# Limita o eixo Y para melhor visualização (o pico inicial de u1 é grande)
plt.ylim(np.min(u2)*1.2, np.max(u2)*2) 
plt.show()

# Cálculo dos Índices IAE e IVU
iae1 = np.trapz(np.abs(e1), t)
iae2 = np.trapz(np.abs(e2), t)

ivu1 = np.trapz(np.abs(u1), t)
ivu2 = np.trapz(np.abs(u2), t)

print(f"Projeto 1 (Alvo 1ª Ordem):")
print(f"  IAE (Integral do Erro Absoluto) = {iae1:.3f}")
print(f"  IVU (Integral do Sinal de Controle) = {ivu1:.3f}")

print(f"\nProjeto 2 (Alvo 2ª Ordem):")
print(f"  IAE (Integral do Erro Absoluto) = {iae2:.3f}")
print(f"  IVU (Integral do Sinal de Controle) = {ivu2:.3f}")

# --- 2. Análise do Item II: Atraso Máximo Tolerado (Δ_max) ---

print("\n--- Análise do Item II (Atraso Máximo do Sensor) ---")
print("Calculando Margens de Fase para determinar o atraso máximo...")

L_1 = C_1 * G_ma_aprox
L_2 = C_2 * G_ma_aprox

gm1, pm1, Wcg1, Wcp1 = clt.margin(L_1)
gm2, pm2, Wcg2, Wcp2 = clt.margin(L_2)

delta_max_1 = (pm1 * np.pi / 180) / Wcp1
delta_max_2 = (pm2 * np.pi / 180) / Wcp2

print(f"\nProjeto 1 (Alvo 1ª Ordem):")
print(f"  Margem de Fase (PM) = {pm1:.2f} graus")
print(f"  Freq. Cruzamento de Ganho (Wcp) = {Wcp1:.3f} rad/s")
print(f"  ==> Atraso máximo adicional (delta_max) = {delta_max_1:.2f} segundos")

print(f"\nProjeto 2 (Alvo 2ª Ordem):")
print(f"  Margem de Fase (PM) = {pm2:.2f} graus")
print(f"  Freq. Cruzamento de Ganho (Wcp) = {Wcp2:.3f} rad/s")
print(f"  ==> Atraso máximo adicional (delta_max) = {delta_max_2:.2f} segundos")
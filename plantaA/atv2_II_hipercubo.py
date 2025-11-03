import numpy as np
import matplotlib.pyplot as plt
import control as clt
from control import tf, feedback

# --- 1. Controlador Fixo (Baseado no Projeto de 1ª Ordem) ---
# Parâmetros nominais que definiram o controlador
K_nominal = 0.20
tau_nominal = 20.0
theta_nominal = 5.0
ts_desejado = 60.0
tau_d = ts_desejado / 4.6

# Controlador PI Fixo
Kc = tau_nominal / (K_nominal * (tau_d + theta_nominal))
Ti = tau_nominal
C = Kc * (1 + tf([1], [Ti, 0]))
print("--- Gerando Hipercubo 3D de Polos para o Controlador PI ---")
print(C)


# --- 2. Definir o Espaço de Parâmetros (Hipercubo 3D) ---
# Amostrando os parâmetros com menos pontos para um cálculo mais rápido
num_pontos_k_tau = 10  # Grade de 10x10
num_pontos_theta = 10  # 10 pontos na vertical

K_vals = np.linspace(0.15, 0.25, num_pontos_k_tau)
tau_vals = np.linspace(15.0, 25.0, num_pontos_k_tau)
theta_vals = np.linspace(3.0, 5.0, num_pontos_theta)


# --- 3. Calcular os Polos para cada Ponto do Hipercubo 3D ---
all_reals = []
all_imags = []
all_thetas_for_plot = []
all_ks_for_plot = []

print("\nCalculando polos para o Hipercubo 3D... Isso pode demorar.")
# Loop por todas as 3 dimensões de incerteza
for k_param in K_vals:
    for tau_param in tau_vals:
        for theta_param in theta_vals:
            # Cria a planta "incerta"
            num_pade, den_pade = clt.pade(theta_param, 4) 
            G_incerta = clt.series(tf([k_param], [tau_param, 1]), tf(num_pade, den_pade))

            # Malha fechada com o controlador fixo C
            Gmf_incerta = feedback(C * G_incerta, 1)

            # Calcula e armazena os polos dominantes
            poles = clt.poles(Gmf_incerta)
            dominant_poles = sorted(poles, key=lambda p: abs(p.real), reverse=False)[:2]

            for p in dominant_poles:
                all_reals.append(p.real)
                all_imags.append(p.imag)
                all_thetas_for_plot.append(theta_param)
                all_ks_for_plot.append(k_param)

print("Cálculo finalizado. Gerando gráfico 3D.")


# --- 4. Plotagem 3D ---
fig = plt.figure(figsize=(12, 10))
ax = fig.add_subplot(111, projection='3d')

# Gráfico de dispersão 3D
# X = Parte Real, Y = Parte Imaginária, Z = Atraso (theta)
# Cor = Ganho (K)
# O efeito de Tau (τ) está "misturado" na nuvem de pontos
sc = ax.scatter(all_reals, all_imags, all_thetas_for_plot, c=all_ks_for_plot, cmap='coolwarm', s=15, alpha=0.6)

# Adiciona o plano de instabilidade (Real = 0)
y_plane, z_plane = np.meshgrid(np.linspace(min(all_imags), max(all_imags), 2), np.linspace(min(all_thetas_for_plot), max(all_thetas_for_plot), 2))
x_plane = np.zeros_like(y_plane)
ax.plot_surface(x_plane, y_plane, z_plane, alpha=0.2, color='r')

ax.set_xlabel('Eixo Real (Estabilidade)')
ax.set_ylabel('Eixo Imaginário (Oscilação)')
ax.set_zlabel('Atraso de Tempo, θ (s)')
ax.set_title('Movimento 3D dos Polos no Hipercubo (K, $\\tau$, $\\theta$)')
ax.view_init(elev=20, azim=-60)

# Barra de cores para a variável K
cbar = plt.colorbar(sc)
cbar.set_label('Ganho, K')

plt.show()
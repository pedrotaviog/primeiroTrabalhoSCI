import control as ct
import matplotlib.pyplot as plt

s = ct.tf('s')
G = -8/(s**2 + 1.5*s - 1)

Td = 0.2

C = -(1 + Td*s)            
L = C * G

# calcula os polos da planta
poles = ct.poles(G)

# formata os polos para texto 
poles_str = ", ".join([f"{p.real:.2f}" if abs(p.imag) < 1e-6 else f"{p.real:.2f}{p.imag:+.2f}j" for p in poles])

ct.root_locus(L, plot=True)
plt.axvline(-10, color='g', linestyle='--', label='Re = -10')
plt.axvline(0, color='r', linestyle='--', label='Eixo imaginário')
plt.title(f'LGR – Com PD (zero em s = {-1/Td:.2f}) | Polos: {poles_str}')
plt.grid(True); plt.legend()
plt.show()

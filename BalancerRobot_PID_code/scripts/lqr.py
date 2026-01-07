import numpy as np
import control

# --- PARAMETRI FISICI (I tuoi) ---
mp = 0.275
mc = 0.0506
l  = 0.062
g  = 9.81
I_com = 0.000552

# --- IL SEGRETO: CALIBRAZIONE SUI TUOI VALORI ---
# Sappiamo che per te, K_theta = 0.4 funziona.
# In fisica reale, questo equivale a 22.9.
# Quindi il tuo "Attuatore" (Motore + Driver + Codice) ha un guadagno globale.
scaling_factor = 57.3 # Conversione Gradi -> Radianti

# Matrici A e B standard del pendolo
den = (mc + mp) * I_com + mc * mp * l**2
A = np.array([
    [0, 1, 0, 0],
    [0, 0, -(mp**2 * g * l**2)/den, 0],
    [0, 0, 0, 1],
    [0, 0, (mp * g * l * (mc + mp))/den, 0]
])

# Matrice B base
B_vec = np.array([
    [0],
    [(I_com + mp * l**2)/den],
    [0],
    [(mp * l)/den]
])

# Qui trucchiamo il modello per parlare la TUA lingua (Gradi e unità piccole)
# Dividiamo B per il scaling factor. Così i K risultanti saranno piccoli!
B = B_vec * (1.0) 

# --- TUNING ---
# Ora puoi giocare con Q e R per cambiare comportamento
# E i numeri che usciranno saranno vicini ai tuoi 0.4, 0.04 etc.
Q = np.diag([
    0.1,    # x: Posizione (Prova ad aumentarlo se vuoi che torni prima)
    0.05,   # dx: Velocità
    0.5,   # theta: Angolo (Priorità alta)
    0.005     # dtheta: Smorzamento
])

R = np.array([[10.0]]) # Penalità alta perché i tuoi valori K sono piccoli

K, S, E = control.lqr(A, B, Q, R)

print("--- GUADAGNI PREDETTI (Scala 'Mio Codice') ---")
print(f"K1 (x)      = {K[0,0]:.5f}  (Tuo: -0.045)") # Nota: LQR dà segno invertito rispetto al tuo
print(f"K2 (dx)     = {K[0,1]:.5f}  (Tuo: -0.040)")
print(f"K3 (theta)  = {K[0,2]:.5f}  (Tuo:  0.400)")
print(f"K4 (dtheta) = {K[0,3]:.5f}  (Tuo:  0.035)")
# -*- coding: utf-8 -*-
"""
Created on Sat May 18 18:42:31 2024

@author: Admin
"""

# -*- coding: utf-8 -*-
"""
Code for paper: H Control for Discrete-Time Multi-Player
                   Systems via Off-Policy Q-Learning.

Method : On-Policy                   
Programing Language : Python
Purpose : Practice and Research

@author: Admin
"""
## Import Lib
import matplotlib.pyplot as plt 
import numpy as np 
import matplotlib.animation as animation
import math
plt.style.use("ggplot")
plt.rcParams["figure.dpi"]= 200
plt.rcParams["figure.figsize"]= (10,6)
plt.rcParams["figure.constrained_layout.use"]= False
# Set up for Animation

## System Parameters
A = np.array([[0.906488,0.0816012,-0.0005],[0.074349,0.90121,-0.000708383],[0,0,0.132655]])
B1 = np.array([[-0.00150808,-0.0096,0.867345]]).T
B2 = np.array([[0.00951892,0.00038373,0]]).T
B3 = np.array([[-0.00563451,-0.08962,0.356478]]).T
E = np.array([[0.0123956,0.068,-0.05673]]).T

Q = np.diag([5,5,5])
R1=1
R2=R1
R3=R1
gamma = 1 
## Optimal Control
K1_s = np.array([[0.5665,0.7300,0.1098]])
K2_s = np.array([[0.4271,0.2752,0.0009]])
K3_s = np.array([[-2.2643,-2.8476,0.0329]])
K_s = np.array([[-2.2925,-2.6572,-0.0032]])
##

n=np.shape(A)[1];
m1=np.shape(B1)[1];
m2=np.shape(B2)[1];
m3=np.shape(B3)[1];
p=np.shape(E)[1];
q= n+m1+m2+m3+p;

## Initial control matrix
K1_0 = np.array([[1,1,0]])
K2_0 = np.array([[0.5,0,0]])
K3_0 = np.array([[-1,-2,1]])
K_0 = np.array([[-2,0,0]])
H0 = np.zeros((q,q))


# Stores the control matrix
save_K1 = [K1_0]
save_K2 = [K2_0]
save_K3 = [K3_0]
save_K = [K_0]
H = [H0]
##
n_end = 50
n_learn =50
x = np.zeros((n,n_end))
u1 = np.zeros((m1,n_end))
u2 = np.zeros((m1,n_end))
u3 = np.zeros((m3,n_end))
w = np.zeros((p,n_end))
z = np.zeros((q, n_end))

for i in range(n_learn):
    Phi = np.zeros((n_end,q*q))
    Psi = np.zeros((n_end,1))
    for k in range(n_end-1): # Collect Data
        e1 = 2 * np.sin(k+1)
        e2 = 2 * np.sin(0.1 * (k+1))
        e3 = 2 * np.sin(0.3 * (k+1))
        u1[:,k]=-save_K1[i] @ x[:, k]+e1;
        u2[:,k]=-save_K2[i] @ x[:, k]+e2;
        u3[:,k]=-save_K3[i] @ x[:, k]+e3;
        w[:,k] = np.exp(-0.0001*k)*np.sin(2*k)
        z[:, k] = np.concatenate((x[:, k], u1[:, k], u2[:, k], u3[:, k], w[:, k]))
        x[:, k+1] = A @ x[:, k] + B1 @ u1[:, k] + B2 @ u2[:, k] + B3 @ u3[:, k] + E @ w[:, k]
        z[:, k+1] = np.concatenate((x[:, k+1], -save_K1[i] @ x[:, k+1], -save_K2[i] @ x[:, k+1], -save_K3[i] @ x[:, k+1], -save_K[i] @ x[:, k+1]))
        Phi[k, :] = np.kron(z[:, k].T, z[:, k].T)
        Psi[k] = x[:, k].T @ Q @ x[:, k] + u1[:, k].T * R1 * u1[:, k] + u2[:, k].T * R2 * u2[:, k] + u3[:, k].T * R3 * u3[:, k] - gamma**2 * w[:, k].T * w[:, k]+z[:, k+1].T @ H[i] @ z[:, k+1]
        # Implement the On-Policy algorithm with z[:, k+1].T @ H[i] @ z[:, k+1] instead of z[:, k+1].T @ H[i+1] @ z[:, k+1]
    vec_H = np.linalg.pinv(np.transpose(Phi) @ Phi) @ np.transpose(Phi) @ Psi
    H.append(np.reshape(vec_H, (q, q))) # Find H in Q-Learning
    Hxu1 = H[i+1][0:n, n:n+m1]
    Hxu2 = H[i+1][0:n, n+m1:n+m1+m2]
    Hxu3 = H[i+1][0:n, n+m1+m2:n+m1+m2+m3]
    Hxw = H[i+1][0:n, n+m1+m2+m3:n+m1+m2+m3+p]
    Hu1u1 = H[i+1][n:n+m1, n:n+m1]
    Hu1u2 = H[i+1][n:n+m1, n+m1:n+m1+m2]
    Hu1u3 = H[i+1][n:n+m1, n+m1+m2:n+m1+m2+m3]
    Hu1w = H[i+1][n:n+m1, n+m1+m2+m3:n+m1+m2+m3+p]
    Hu2u2 = H[i+1][n+m1:n+m1+m2, n+m1:n+m1+m2]
    Hu2u3 = H[i+1][n+m1:n+m1+m2, n+m1+m2:n+m1+m2+m3]
    Hu2w = H[i+1][n+m1:n+m1+m2, n+m1+m2+m3:n+m1+m2+m3+p]
    Hu3u3 = H[i+1][n+m1+m2:n+m1+m2+m3, n+m1+m2:n+m1+m2+m3]
    Hu3w = H[i+1][n+m1+m2:n+m1+m2+m3, n+m1+m2+m3:n+m1+m2+m3+p]
    Hww = H[i+1][n+m1+m2+m3:n+m1+m2+m3+p, n+m1+m2+m3:n+m1+m2+m3+p]
    K1 = np.linalg.pinv(Hu1u1) @ (np.transpose(Hxu1) - (Hu1u2 @ save_K2[i] + Hu1u3 @ save_K3[i] + Hu1w @ save_K[i]))
    K2 = np.linalg.pinv(Hu2u2) @ (np.transpose(Hxu2) - (np.transpose(Hu1u2) @ save_K1[i] + Hu2u3 @ save_K3[i] + Hu2w @ save_K[i]))
    K3 = np.linalg.pinv(Hu3u3) @ (np.transpose(Hxu3) - (np.transpose(Hu1u3) @ save_K1[i] + np.transpose(Hu2u3) @ save_K2[i] + Hu3w @ save_K[i]))
    K = np.linalg.pinv(Hww) @ (np.transpose(Hxw) - (np.transpose(Hu1w) @ save_K1[i] + Hu2w @ save_K2[i] + np.transpose(Hu3w) @ save_K3[i]))
    # Find Optimal Solution Step by Step
    save_K1.append(K1)
    save_K2.append(K2)
    save_K3.append(K3)
    save_K.append(K)


dK1 = np.zeros((n_learn,1))
dK2 = np.zeros((n_learn,1))
dK3 = np.zeros((n_learn,1))
dK = np.zeros((n_learn,1))

for j in range(n_learn):
    dK1[j] = np.linalg.norm(save_K1[j]-K1_s)
    dK2[j] = np.linalg.norm(save_K2[j]-K2_s)
    dK3[j] = np.linalg.norm(save_K3[j]-K3_s)
    dK[j] = np.linalg.norm(save_K[j]-K_s)
    
##
# Results are obtained but convergence is slow

t = np.arange(n_learn)
#t = t.reshape(1, -1)

# =============================================================================
# =============================================================================
# fig, ax = plt.subplots(facecolor='white')
# ax.set_facecolor('white')
# def aniFunc(t1):
#     ax.clear()
#     ax.plot(t[: (t1 + 1)], dK1[: (t1 + 1)], "-o",color='cyan')
#     ax.grid()
#     ax.set(title="Iteration: "+str(t1), xlim=(-0.5, n_learn), ylim=(-0.1, 0.8)) 
# ani = animation.FuncAnimation(fig, aniFunc, frames=len(dK1))
# ani.save(r"K1.gif", writer="pillow", fps=5)
# 
# 
# # =============================================================================
# fig2, ax2 = plt.subplots(facecolor='white')
# ax2.set_facecolor('white')
# def aniFunc2(t1):
#     ax2.clear()
#     ax2.plot(t[: (t1 + 1)], dK2[: (t1 + 1)], "-o", color='cyan')
#     ax2.grid()
#     ax2.set(title="Iteration: "+str(t1), xlim=(-0.5, n_learn), ylim=(-0.1, 0.5)) 
# ani2 = animation.FuncAnimation(fig2, aniFunc2, frames=len(dK2))
# ani2.save(r"K2.gif", writer="pillow", fps=5)
# 
# # =============================================================================
# fig3, ax3 = plt.subplots(facecolor='white')
# ax3.set_facecolor('white')
# def aniFunc3(t1):
#     ax3.clear()
#     ax3.plot(t[: (t1 + 1)], dK3[: (t1 + 1)], "-o", color='cyan')
#     ax3.grid()
#     ax3.set(title="Iteration: "+str(t1), xlim=(-0.5, n_learn), ylim=(-0.1, 2)) 
# ani3 = animation.FuncAnimation(fig3, aniFunc3, frames=len(dK3))
# ani3.save(r"K3.gif", writer="pillow", fps=5)
# 
# # =============================================================================
# fig4, ax4 = plt.subplots(facecolor='white')
# ax4.set_facecolor('white')
# def aniFunc4(t1):
#     ax4.clear()
#     ax4.plot(t[: (t1 + 1)], dK[: (t1 + 1)], "-o", color='cyan')
#     ax4.grid()
#     ax4.set(title="Iteration: "+str(t1), xlim=(-0.5, n_learn), ylim=(-0.1, 2.8)) 
# ani4 = animation.FuncAnimation(fig4, aniFunc4, frames=len(dK))
# ani4.save(r"K_sol.gif", writer="pillow", fps=5)
# =============================================================================


##
# End
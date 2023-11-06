# -*- coding: utf-8 -*-
"""
Created on Fri Sep 27 16:25:07 2019

@author: oscar
"""
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def viterbi(obs, Modelo1):
    mat_A, mat_B = Modelo1.A, Modelo1.B
    delta = np.zeros((len(obs)+1, len(mat_A)))
    phi = np.zeros((len(obs)+1, len(mat_A)))+666
    path = np.zeros(len(obs)+1)
    
    delta[0, :] = Modelo1.PI * mat_B[:, obs[0]]
    phi[0, :] = 666
    for t in range(len(obs)):
        for j in range(delta.shape[1]):
            delta[t+1, j] = np.max(delta[t] * mat_A[:, j]) * mat_B[j, obs[t]]
            phi[t+1, j] = np.argmax(delta[t] * mat_A[:, j])

    path[len(obs)] = int(np.argmax(delta[len(obs), :]))
    for i in np.arange(len(obs)-1, 0, -1):
        path[i] = phi[i+1, int(path[i+1])]

    return (path)


def forw_alg(o_k, Modelo):
    # MATRIX NOTATION
    PI = Modelo.PI
    K = len(o_k)  # Secuencia Observaciones
    N = len(Modelo.A)  # número de estados
    alpha = np.zeros((N, K))
    alpha[:, 0] = PI
    for k in range(1, K):
        alpha_k = alpha[:, k-1]
        a = Modelo.A[:, :]
        b = Modelo.B[:, o_k[k]]
        alpha[:, k] = (b*np.dot(alpha_k, a))
    return alpha


def backw_alg(o_k, Modelo):
    # MATRIX NOTATION
    K = len(o_k)  # Secuencia Observaciones
    N = len(Modelo.A)  # número de estados
    beta = np.zeros((N, K))
    beta[:, -1] = 1
    for t in range(K-2, -1, -1):
        beta_t1 = beta[:, t+1]
        # beta_t1   #TODO: remove
        a = Modelo.A[:, :]
        b = Modelo.B[:, o_k[t]]
        beta[:, t] = b*np.dot(a, beta_t1)
    return beta


class HMM(object):
    def __init__(self, A, B, PI):
        self.A = A
        self.B = B
        self.PI = PI


def cuantizar_xy(xy, cc):
    xycuant = cc
    out = np.power(xycuant-xy, 2).sum(axis=1).argmin()
    return out


def path_to_xy(path, ccxy):
    estimated = pd.DataFrame(path.astype(int).T)
    estimated.columns = ['Path_vit']
    estimated['xcuant'] = estimated['Path_vit'].apply(lambda x: ccxy[x, 0])
    estimated['ycuant'] = estimated['Path_vit'].apply(lambda x: ccxy[x, 1])
    return (estimated)


def quantized(xyth, ccxyth):
    xythcuant = np.argmin(np.linalg.norm(xyth-ccxyth, axis=1))
    x, y = ccxyth[xythcuant, :2]
    return ((x, y), (xythcuant))


def Markov_A_2_grafo(A, ccxyth):
    dists = np.zeros(A.shape)
    for i in range(A.shape[0]):
        for j in range(A.shape[1]):
            if A[i, j] != 0:
                dists[i, j] = np.linalg.norm(ccxyth[i]-ccxyth[j])

    con = np.where(dists == 0, np.inf, dists)
    graphe2 = grafo(ccxyth, con)
    return graphe2


class node(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y


class grafo (object):
    def __init__(self, nodos, conec):
        self.nodos = nodos
        self.conec = conec


def dijkstra(nodoinicial, nodofinal, graphe):
    numnodos = len(graphe.nodos)
    con = graphe.conec
    D = np.ones(numnodos)*np.inf
    Prv = np.ones(numnodos)*np.inf
    V = np.zeros(numnodos)
    a = nodoinicial
    D[a] = 0
    Prv[a] = 0
    Prv[np.where(con[a] != np.inf)] = a
    V[a] = 1
    Dacc = D[a]
    ########
    D = np.minimum(D, con[a]+D[a])
    cont = 0
    sucess = False
    while (sucess == False):
        a = np.argmin(D+np.where(V == 1, np.inf, V))
        Dacc = D[a]
        Prv[np.where(D > (con[a]+Dacc))] = a
        V[a] = 1
        D = np.minimum(D, con[a]+Dacc)
        if (a == nodofinal):
            print("RUTA CALCULADA ")
            sucess = True
    rutainv = []
    rutainv.append(nodofinal)
    while (rutainv[-1] != nodoinicial):
        prv = Prv[int(rutainv[-1])]
        rutainv.append(prv)

    ruta = []
    for n in reversed(rutainv):
        ruta.append(n)
    return (ruta)


def plot_cc_arrow(cc):
    fig = plt.figure(figsize=(10, 10))
    ax1 = fig.add_subplot(111)

    for c in cc:
        ang = c[2]*180/math.pi
        plt.plot(c[0], c[1], marker=(3, 1, ang),
                 markersize=10, c='green', linestyle='None')
        plt.plot(c[0]-(.01*np.cos(c[2]+math.pi/2)), c[1]-(.01*np.sin(c[2]+math.pi/2)),
                 marker=(1, 1, ang), markersize=25, c='green', linestyle='--')

    plt.show()


def accuracy(yhat, ytrue):
    weights = [.5, .3, .1, .05, .05]
    if len(yhat) >= 5:
        yhat = yhat[-5:]
        ytrue = ytrue[-5:]
        accuracy = np.dot((yhat == ytrue), weights)
        return accuracy


def baum_welch(obs, est, save=False):

    # BAUM WELCH APPROX###############???!?!?!

    N = est.max()+1
    K_corpus = 318  # obs.max()+1
    TRANS = np.zeros((N, N))
    for i in range(N):
        fromTO = []

        indices = np.asarray(np.where(est == i)).ravel()

        for indice in indices:

            if (int(indice) == len(est)-1):

                indice = indice-1

            fromTO.append(est[indice+1])
        fromTO.append(0)
        fromTO.append(N-1)
        aux = np.asarray(np.bincount(fromTO))
        aux[0] = aux[0]-1
        aux[N-1] = aux[N-1]-1

        TRANS[i, :] = aux
    TRANS = TRANS/(TRANS.sum(axis=1)+.00001)
    # 3
    EMIS = np.zeros((N, K_corpus))
    for Vk in range(K_corpus):

        indices = np.asarray(np.where(obs == Vk)).ravel()
        estconvk = est[indices]

        numestenvk = np.bincount(estconvk)
        if (N-len(numestenvk) > 0):
            numestenvk = np.append(numestenvk, np.zeros(N-len(numestenvk)))
        EMIS[:, Vk] = (numestenvk/(np.bincount(est)+.00001))

    A, B = TRANS, EMIS
    PI = np.ones(N)/N
    Modelo1 = HMM(A, B, PI)
    if save:
        np.save('A.npy', Modelo1.A)
        np.save('B.npy', Modelo1.B)
        np.save('PI.npy', Modelo1.PI)
        print("MODEL SAVED")
        return (A, B, PI)
    else:
        # A,B,PI = np.load('A.npy'),np.load('B.npy'),np.load('PI.npy')
        # Modelo1=HMM(A,B,PI)
        return (A, B, PI)

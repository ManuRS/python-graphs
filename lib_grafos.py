import numpy as np
import random as rd
import matplotlib.pyplot as plt
import sys
import queue

def grafo_aleatorio(num_nodos, prob):
    res = np.zeros(shape = (num_nodos, num_nodos))
    for i in range(num_nodos):
        for j in range(num_nodos):
            if i>j and i!=j and rd.random()<=prob:
                    res[i][j]=1
                    res[j][i]=1
    return res

def tam_grafo(grafo):
    return np.shape(grafo)[0]

def aleatorio(num_nodos, prob):
    grafo_aleatorio(num_nodos, prob)

def indiceCluster_caminoCar(grafo):
    c, grados, N = calcula_C(grafo)   
    l = calcula_L(grafo)
    return c, l, grados

def aleatiorio2(num_nodos, prob):
    c, l, grados = indiceCluster_caminoCar(grafo_aleatorio(num_nodos, prob))
    print ("Indice de clusterización (C) = " + str(c))
    print ("Camino característico (L) = " + str(l))
    print ("Distribución de los grados de los nodos:")
    plt.figure()
    dist = get_distro(grados, num_nodos)
    f = open('dist.txt', 'w')
    f.write(str(dist))
    #print(str(dist))
    plt.plot(dist)
    plt.show()
    return c, l, grados

def calcula_C(grafo):
    nodos=int(tam_grafo(grafo))
    grados = calcula_grados_rapido(grafo, nodos)
    N=[0 for i in range(nodos)]
    c = [0.0 for x in range(nodos)]
    suma = 0.0
    
    for i in range(nodos):
        for j in range(nodos):
            if grafo[i][j] == 1:
                for k in range(j, nodos):
                    if grafo[j][k] == 1 and grafo[i][k] == 1:
                        N[i] = N[i] + 1
                        
    for i in range(nodos):
        if grados[i] > 1:
            c[i] = (2*N[i]) / (grados[i] * (grados[i] - 1))
        suma = suma + c[i]
    
    return suma/nodos, grados, N

def calcula_L(grafo):
    L=0
    for i in range (tam_grafo(grafo)):
        distancias, padres = busqueda_anchura(grafo, i)
        L = L + ( sum(distancias) / (tam_grafo(grafo)-1) )
    return L / tam_grafo(grafo)

def calcula_grados(grafo):
    grados=[0 for i in range(tam_grafo(grafo))]
    all_vecinos=[]
    for i in range (tam_grafo(grafo)):
        vecinos=[]
        for j in range (tam_grafo(grafo)):
            if grafo[i][j]==1:
                grados[i] = grados[i] + 1
                vecinos.append(j)   
        all_vecinos.append(vecinos)
    return grados, all_vecinos

def adyacentes(grafo, vertice):
    vecinos=[]
    for j in range (tam_grafo(grafo)):
        if grafo[vertice][j]==1:
            vecinos.append(j) 
    return vecinos

def matrix_to_adyacent(grafo):
    g, v = calcula_grados(grafo)
    return v

def calcula_grados_rapido(grafo, limit):
    grados=[0 for i in range(limit)]
    for i in range (limit):
        grados[i]=sum(grafo[i])
    return grados

def get_distro(grados, tam_grafo):
    res=[0 for i in range(tam_grafo)]
    for g in grados:
        res[int(g)]+=1
    return res
    
def busqueda_anchura(grafo, nodo_ini=0):
    visitado=[False for i in range(tam_grafo(grafo))]
    distancias=[float('inf') for i in range(tam_grafo(grafo))]
    padre=[None for i in range(tam_grafo(grafo))]

    visitado[nodo_ini]=True
    distancias[nodo_ini]=0
    
    cola = queue.Queue()
    cola.put(nodo_ini)
    
    while cola.empty()==False:
        u = cola.get()
        for v in adyacentes(grafo, u):
            if visitado[v]==False:
                distancias[v]=distancias[u]+1
                padre[v]=u
                cola.put(v)
                visitado[v]=True
    return distancias, padre

def componentes_conexas(grafo):
    res=[]
    cola = queue.Queue()
    
    #Primera componente
    aux, cola = componentes_conexas_nodo(grafo, 0, cola)
    res.append(aux)
        
    #Resto, vamos vaciando la cola
    while cola.empty()==False:
        u = cola.get()
        #La cola contiene candidatos a ser una nueva componente
        #Aqui se comprueba si su componente ya existe o es nueva
        flag=False
        for lis in res:
            if u in lis:
                #Su componente ya existe
                flag=True
        #Este elemento forma una nueva componente
        if flag!=True:
            aux, cola = componentes_conexas_nodo(grafo, u, cola)  
            res.append(aux)
    return res, len(res)


def componentes_conexas_nodo(grafo, nodo, cola):
    distancias, padres = busqueda_anchura(grafo, nodo)
    aux=[nodo]
    for i in range(len(padres)):
        if padres[i]!=None:
            #Nuevo elemento de la componente
            aux.append(i)
        elif i>0:
            #Este es de otra componente, a la cola de candidatos
            cola.put(i)
    return aux, cola

def aleatorio3(N, p):
    text=""
    res, tam = componentes_conexas(grafo_aleatorio(N, p))
    for i in range(tam):
        text = text + "Componente " + str(i) + " tiene " + str(len(res[i])) + " nodos\n"
    return text, tam
  
def create_mundo_pequeno (num_nodos, k, probabilidad):
    num_vecinos=2*k
    res = np.zeros(shape = (num_nodos, num_nodos))
    for i in range(num_nodos):
        for j in range(k):
            res[i][(i+j+1)%num_nodos]=1
            res[i][(i-j-1)%num_nodos]=1
    for i in range(num_nodos):
        vecinos = adyacentes(res, i)
        for vecino in vecinos:
            if vecino>i:
                if rd.random()<=probabilidad:
                    nr = rd.randrange(0, num_nodos-1)
                    count_break=0
                    posible=True
                    while res[i][nr]==1 and i==nr:
                        nr = rd.randrange(0, num_nodos-1)
                        count_break = count_break + 1
                        if count_break == num_nodos*2: 
                            posible=False #Asumimos que este nodo tiene todas las posibles ramas
                            break
                    if posible != False and len(adyacentes(res,i))>1 and len(adyacentes(res,vecino))>1:
                        res[i][vecino]=0
                        res[vecino][i]=0
                        res[i][nr]=1
                        res[nr][i]=1
    return res

def mundo_pequeno (num_nodos, k, probabilidad):
    res = create_mundo_pequeno (num_nodos, k, probabilidad)
    c, grafos, N = calcula_C(res)
    l = calcula_L(res)
    return res, c, l

def mundo_pequeno_print (num_nodos, k, probabilidad):
    res, c, l = mundo_pequeno (num_nodos, k, probabilidad)
    print("=================\nGenerado grafo:\nC = "+str(c)+"\nL = "+str(l))
    print (res)
    
def libre_escala (num_inicial, num_final, num_ramas):
    
    #Hacemos la matriz del tamaño maximo y hacemos un grafo aleatorio para el tam inicial
    res = np.zeros(shape = (num_final, num_final))
    prob=.5
    for i in range(num_inicial):
        for j in range(num_inicial):
            if i>j and i!=j and rd.random()<=prob:
                    res[i][j]=1
                    res[j][i]=1
                    
    #Crecimiento
    for i in range(num_inicial, num_final):
        #Calculo de probabilidades
        grados = calcula_grados_rapido(res, i)
        suma = sum(grados)
        grados = [x / suma for x in grados]
        #Numero de ramas
        for j in range(num_ramas):
            #Evitar añadir dos veces la misma
            encontrado=False
            while encontrado==False:
                #Calculo de la seleccionada
                aleat = rd.random()
                suma = grados[0]
                index=0
                while suma<aleat:
                    index=index+1
                    suma=suma+grados[index]
                #Actualizar matriz
                if res[i][index]==0:
                    res[i][index]=1
                    res[index][i]=1
                    encontrado=True
    return res

def libre_escala_print (num_inicial, num_final, num_ramas, l=True):
    print("!!!Calculando grafo libre de escala")
    res = libre_escala (num_inicial, num_final, num_ramas)
    
    print("!!!Calculando C")
    c, gg, N = calcula_C(res)
    
    print("!!!Calculando grados")
    grados = calcula_grados_rapido(res, num_final)
    
    if l==True:
        print("!!!Calculando L")
        L = calcula_L(res)
    
    print("!!!Calculando distribucion")
    dist = get_distro(grados, num_final)
    
    print("!!!Ploteando")
    print("==================\nTamaño grafo = "+str(num_final))
    if l==True:
        print("C = "+str(c)+"\nL = "+str(L))
    else:
        print("C = "+str(c))
        
    
    x=[]
    y=[]
    for i in range(len(dist)):
        if dist[i]!=0:
            x.append(i)
            y.append(dist[i])
        

    X = np.log(x)
    Y = np.log(y)   
    plt.plot(X, Y, 'o')
    m,b = np.polyfit(X[0:len(x)/2], Y[0:len(x)/2], 1) 
    plt.plot(X, m*X+b)
    print("Pendiente recta regresion = "+str(m))
    plt.show()

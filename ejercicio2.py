#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import numpy as np
import time
import math
import plot_robot
from Robot import Robot

EJERCICIO2 = False
EJERCICIO3 = True

circle = []
robot = Robot()

def hom(xt):
    x = xt[0]
    y = xt[1]
    alpha = xt[2]

    cos = math.cos(alpha)
    sin = math.sin(alpha)

    f1 = [cos, -sin, x]
    f2 = [sin, cos, y]
    f3 = [0, 0, 1]
    T = np.matrix([f1, f2, f3])

    return T

def loc(T):
    x = T[0, 2]
    y = T[1, 2]
    alpha = math.atan2(T[1, 0], T[0, 0])

    return [x, y, alpha]

# calcular coordenadas del objetivo en el círculo dependiendo del tiempo que ha pasado
def obtenerCoordenadasCirculo(timems):
    VELOCIDAD = 3/10   # velocidad en radianes/s
    RADIO = 10      # radio del circulo
    mod = (2 * math.pi * 1000) / VELOCIDAD      # tiempo en ms para dar una vuelta a 3 rad/s
    timems = timems % mod

    angulo = ( timems * 2 * math.pi ) / ( 2000 * math.pi / VELOCIDAD )
    x = RADIO * math.sin(angulo)
    y = RADIO * math.cos(angulo)

    circle.append([x,y,angulo])
    return x,y,(angulo + (math.pi / 2))



def seguirCirculo(umbral, K):
    startms = int(round(time.time() * 1000))
    x,y,trad = robot.readOdometry()
    xg,yg,tg = obtenerCoordenadasCirculo(int(round(time.time() * 1000)) - startms)
    print("posicion actual x",x,"y",y,"th",math.degrees(trad))
    distancia = math.sqrt( math.pow((xg-x),2) + math.pow((yg-y), 2) )
    kalfa,kbeta,kro = K[0],K[1],K[2]

    count = 0
    while distancia > umbral and count < 5000:
        
        xrobot_rd_mundo = np.array([x,y,trad])
        xgoal_rd_mundo = np.array([xg,yg,math.radians(tg)])
        Trobot_rd_mundo = hom(xrobot_rd_mundo)
        Tgoal_rd_mundo = hom(xgoal_rd_mundo)
        Tmundo_rd_goal = np.linalg.inv(Tgoal_rd_mundo)
        Trobot_rd_goal = Tmundo_rd_goal @ Trobot_rd_mundo
        xrobot_rd_goal = loc(Trobot_rd_goal)

        ro = math.sqrt( pow(xrobot_rd_goal[0],2) + pow(xrobot_rd_goal[1],2) )
        betaaux = math.atan2(xrobot_rd_goal[1],xrobot_rd_goal[0]) + math.pi
        beta = math.atan2(math.sin(betaaux), math.cos(betaaux))
        alfa = beta - xrobot_rd_goal[2]

        v = kro * ro
        w = kalfa * alfa + kbeta * beta

        robot.setSpeed(v,w)

        robot.waitOdometryUpdated()
        x,y,trad = robot.readOdometry()
        xg,yg,tg = obtenerCoordenadasCirculo(int(round(time.time() * 1000)) - startms)
        distancia = math.sqrt( pow((xg-x),2) + pow((yg-y),2) )

        count = count+1
    print(count)

def trayectoria(objetivo, umbral, K):
    print("objetivo x",objetivo[0],"y",objetivo[1],"th",objetivo[2])
    x,y,trad = robot.readOdometry()
    print("posicion actual x",x,"y",y,"th",math.degrees(trad))
    distancia = math.sqrt( math.pow((objetivo[0]-x),2) + math.pow((objetivo[1]-y), 2) )
    kalfa,kbeta,kro = K[0],K[1],K[2]
    count = 0
    while distancia > umbral:
        
        xrobot_rd_mundo = np.array([x,y,trad])
        xgoal_rd_mundo = np.array([objetivo[0],objetivo[1],math.radians(objetivo[2])])
        Trobot_rd_mundo = hom(xrobot_rd_mundo)
        Tgoal_rd_mundo = hom(xgoal_rd_mundo)
        Tmundo_rd_goal = np.linalg.inv(Tgoal_rd_mundo)
        Trobot_rd_goal = Tmundo_rd_goal @ Trobot_rd_mundo
        xrobot_rd_goal = loc(Trobot_rd_goal)

        ro = math.sqrt( pow(xrobot_rd_goal[0],2) + pow(xrobot_rd_goal[1],2) )
        betaaux = math.atan2(xrobot_rd_goal[1],xrobot_rd_goal[0]) + math.pi
        beta = math.atan2(math.sin(betaaux), math.cos(betaaux))
        alfa = beta - xrobot_rd_goal[2]

        v = kro * ro
        w = kalfa * alfa + kbeta * beta

        robot.setSpeed(v,w)

        robot.waitOdometryUpdated()
        x,y,trad = robot.readOdometry()
        distancia = math.sqrt( pow((objetivo[0]-x),2) + pow((objetivo[1]-y),2) )

        count = count+1

def main():
    try:
        robot.startOdometry()

        K = [1.5,1,0.75]
        
        if EJERCICIO2:
            trayectoria([2.0,3.0,90], 0.1, K)
            trayectoria([6.0,4.0,45], 0.1, K)
            trayectoria([10.0,5.0,-45], 0.1, K)
            trayectoria([7.0,-3.0,180], 0.1, K)
            trayectoria([2.0,3.0,90], 0.1, K)
        
        if EJERCICIO3:
            seguirCirculo( 1, K)
            


        robot.lock_odometry.acquire()
        print("Odom values at main at the END: %.2f, %.2f, %.2f " % (robot.x.value, robot.y.value, robot.th.value))
        robot.lock_odometry.release()

        robot.stopOdometry()

        if EJERCICIO3:
            plot_robot.plotLogCircle(circle)


    except KeyboardInterrupt:
    # except the program gets interrupted by Ctrl+C on the keyboard.
    # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
        robot.stopOdometry()

if __name__ == "__main__":
    main()

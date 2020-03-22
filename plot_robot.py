import argparse
import math

import numpy as np
import datetime
import matplotlib
import matplotlib.pyplot as plt
import csv

#matplotlib.use('GTK')  # Or any other X11 back-end


# Dibuja robot en location_eje con color (c) y tamano (p/g)
def dibrobot(loc_eje,c,tamano):
    if tamano=='p':
        largo=0.1
        corto=0.05
        descentre=0.01
    else:
        largo=0.5
        corto=0.25
        descentre=0.05

    trasera_dcha=np.array([-largo,-corto,1])
    trasera_izda=np.array([-largo,corto,1])
    delantera_dcha=np.array([largo,-corto,1])
    delantera_izda=np.array([largo,corto,1])
    frontal_robot=np.array([largo,0,1])
    tita=loc_eje[2]
    Hwe=np.array([[np.cos(tita), -np.sin(tita), loc_eje[0]],
                [np.sin(tita), np.cos(tita), loc_eje[1]],
                [0,        0 ,        1]])
    Hec=np.array([[1,0,descentre],
                [0,1,0],
                [0,0,1]])
    extremos=np.array([trasera_izda, delantera_izda, delantera_dcha, trasera_dcha, trasera_izda, frontal_robot, trasera_dcha])
    robot=np.dot(Hwe,np.dot(Hec,np.transpose(extremos)))

    plt.plot(robot[0,:], robot[1,:], color = c)

def normalizeLogFile(data):
    # Open csv file
    logFileName = args.outputlog
    f = open(logFileName, 'w')

    # Write logs to CSV file
    with f:
        print("Printing logs to CSV file : " + logFileName + "...")
        writer = csv.writer(f)
        writer.writerow(['x', 'y', 'theta', 'time'])
        for i, line in enumerate(data):
            row = [float(line[0]), float(line[1]), float(line[2]), line[3]]
            row[2] = math.atan2(math.sin(row[2]), math.cos(row[2]))  # normalize# Print odometry log file
            writer.writerow(row)

    # Close file
    f.close()


def plotLog(data):
    num = len(data)
    for i, line in enumerate(data):
        if i%8 == 0:
            progress = float(i) / float(num)
            pos = [float(line[0]), float(line[1]), float(line[2])]
            dibrobot(pos, (1.0-progress, progress, 0.0), 'p')
    plt.show()

def plotLogCircle(data):
    for i, line in enumerate(data):
        if i%8 == 0:
            pos = [float(line[0]), float(line[1]), float(line[2])]
            dibrobot(pos, (0.5, 0.5, 1.0), 'p')
    plt.show()

def plotLogFiledesc(f):
    data = list(csv.reader(f))
    data.pop(0)
    if args.outputlog:
        normalizeLogFile(data)

    plotLog(data)

def plotLogFile(fileName):
    with open(fileName, newline='') as f:
        plotLogFiledesc(f)

def draw():
    plt.draw()
    plt.pause(0.001)

def save():
    print("Saving plot to file: ")
    plt.savefig('trajectory'+ str(datetime.datetime.now()) + '.png')
    plt.show()

def main(args):
    if args.file:
        plotLogFiledesc(args.file)


if __name__ == "__main__":
    # get and parse arguments passed to main
    # Add as many args as you need ...
    parser = argparse.ArgumentParser()
    parser.add_argument("-f", "--file", help="Log file to plot",
                        type=argparse.FileType('r'))
    parser.add_argument("-o", "--output", help="File to save the plot to")
    parser.add_argument("-ol", "--outputlog", help="File to save the log file")

    args = parser.parse_args()

    main(args)
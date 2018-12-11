import matplotlib.pyplot as plt
from pathPlanning import *
from model import *
import time
import numpy as np
import cv2
from vehicleState import VehicleState
from math import *

p = os.path.abspath(os.path.dirname(__file__))
lib_path = os.path.abspath(os.path.join(p, '..', '..', 'truck_map', 'scripts'))
sys.path.append(lib_path)
import map_func
import ref_path_gui



#GET optimal path
map_obj = map_func.Map()
ref_obj = ref_path_gui.RefPath()
mapo, scaleo = map_obj.getMapAndScale()
start = ref_path_gui.VehicleState(115, 600, -pi/2, -pi/2)
#start = ref_path.VehicleState(3700, 7000, 90, 90)
#start = ref_path.VehicleState(3300, 6000, -90, -90)

optimalPath = ref_obj.getRefPath(start)
"""
output = []
for x in optimalPath:
    if x not in output:
        output.append(x)
optimalPath = output"""

#optimalPath = map(lambda a : (a[0], a[1]), rp)

#optimalPath = [(115, 510), (115, 415), (165, 415), (260, 415), (273, 444), (294, 467), (323, 483), (330, 497), (325, 670)]

mapName = 'map.png'
mapp = np.asarray(cv2.imread(mapName, 0), dtype=np.bool).tolist()
#optimalFile = "optimal_path_rondell3.txt"
#mapName = 'rondell_4.png'

matr = map_func.readImgToMatrix('/'+mapName)
pf = PathPlanner(mapo)
#pf.setOptimalpath(optimalPath)

#startPoint = Point(100, 530) # for rondell
#endPoint = Point(523, 60) #for rondell
startPoint = Point(115, 600) #for map3
#startPoint = Point(370, 700) #for map3
#startPoint = Point(330, 600) #for map3
endPoint = optimalPath[-1] #for map3

start_time = time.time()
vehicleState = VehicleState(startPoint.x, startPoint.y, radians(-90), radians(-90))
#vehicleState = VehicleState(startPoint.x, startPoint.y, radians(-90), radians(-90))
#path = pf.creategraph(startPoint, endPoint, 30, radians(0), radians(0))

pf.setOptimalpath([(327.6, 585.0), (327.2, 605.0), (326.7, 625.0), (326.2, 645.0), (325.8, 665.0), (325.3, 685.0), (324.8, 705.0), (324.4, 725.0), (323.9, 745.0), (323.4, 765.0), (323.0, 785.0), (321.1, 799.8), (318.3, 809.7), (314.5, 819.3), (309.9, 828.5), (304.4, 837.2), (298.1, 845.4), (291.1, 852.9), (283.4, 859.8), (275.1, 865.8), (266.3, 871.1), (257.0, 875.0), (245.9, 878.4), (234.7, 880.6), (223.2, 881.8), (211.7, 881.8), (200.3, 880.8), (189.0, 878.6), (178.0, 875.0), (166.4, 870.5), (155.6, 864.4), (145.7, 856.9), (137.0, 848.0), (129.6, 838.1), (123.7, 827.1), (119.4, 815.5), (116.8, 803.3), (116.0, 797.0), (116.0, 776.3), (116.0, 755.7), (116.1, 735.1), (116.1, 714.5), (116.1, 693.9), (116.2, 673.3), (116.2, 652.7), (116.2, 632.1), (116.3, 611.5), (116.3, 590.9), (116.3, 570.3), (116.4, 549.7), (116.4, 529.1), (116.5, 508.5), (116.5, 487.8), (116.5, 467.2), (116.6, 446.6), (116.6, 426.0), (116.6, 405.4), (116.7, 384.8), (116.7, 364.2), (116.7, 343.6), (116.8, 323.0), (116.8, 302.4), (116.8, 281.8), (116.9, 261.2), (116.9, 240.6)])

path = pf.getPath(VehicleState(116.152891805, 693.191225702, 4.71239524926, 4.588817296), (116.9, 240.6), (116.9, 261.2))
#path = pf.getPath(vehicleState, endPoint, optimalPath[-2])
#37 does many laps

run_time = ("--- %s seconds ---" % (time.time() - start_time))
model = truck()

lx= []
ly=[]
lt1= []
lt2=[]


#for ((xa,ya),ta1, ta2) in path:
    #print "angle", ta1
    #print xa,ya
#    lx.append(xa)
#    ly.append(ya)

for vs in path:
    print "x, y: ", vs.x, vs.y
    lx.append(vs.x)
    ly.append(vs.y)
    lt1.append(vs.theta1)
    lt2.append(vs.theta2)

li= []
#for i in range(len(lx)-1):
#    li = li + [(model.calculateCorners(Point(lx[i],ly[i]), lt1[i],lt2[i]))]

headerFrontLeftx = []
headerFrontLefty = []

headerFrontRightx = []
headerFrontRighty = []

headerBacktLeftx = []
headerBacktLefty = []

headerBackRightx = []
headerBackRighty = []

trailerBackLeftx = []
trailerBackLefty = []

trailerBackRightx = []
trailerBackRighty = []
count =0

for ((x1,y1),(x2,y2),(x3,y3),(x4,y4),(x5,y5),(x6,y6)) in reversed(li):
    #if count >4:
    #    break
    #count= count+1
    headerFrontLeftx.append(x1)
    headerFrontLefty.append(y1)
    headerFrontRightx.append(x2)
    headerFrontRighty.append(y2)
    headerBacktLeftx.append(x3)
    headerBacktLefty.append(y3)
    headerBackRightx.append(x4)
    headerBackRighty.append(y4)
    trailerBackLeftx.append(x5)
    trailerBackLefty.append(y5)
    trailerBackRightx.append(x6)
    trailerBackRighty.append(y6)
#matrix = np.asarray(cv2.imread(mapName, 0), dtype=np.bool).tolist()
plt.imshow(mapo)
#wall,
#wallx1 = [500,80,80,80,500,500]
#wally1 = [842,842,842,540,540,0]

#wallx2 = [0,0,420,420,0]
#wally2 = [1000,460,460,166,166]

plt.plot(headerFrontLeftx, headerFrontLefty, 'blue')
plt.plot(headerFrontRightx, headerFrontRighty, 'blue')

plt.plot(headerBacktLeftx, headerBacktLefty, 'red')
plt.plot(headerBackRightx, headerBackRighty, 'red')

plt.plot(trailerBackRightx, trailerBackRighty, 'green')
plt.plot(trailerBackLeftx, trailerBackLefty, 'green')

#plt.plot(wallx1, wally1, 'yellow')
#plt.plot(wallx2, wally2, 'yellow')

plt.plot([startPoint.x, endPoint[0]], [startPoint.y, endPoint[1]], 'ro')

plt.plot(lx, ly, 'go')

optimal_x =  []
optimal_y =  []
for l in optimalPath:
    optimal_x.append(l[0])
    optimal_y.append(l[1])

plt.plot(optimal_x, optimal_y, 'yellow')

print run_time
print start
print "opt", optimalPath

plt.axis([-10, 1000, 1000, -10])
plt.show()

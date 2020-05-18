#!/usr/bin/python
import numpy as np
from polysimplify import VWSimplifier
from rdp import rdp

from utm import utmconv
import matplotlib
import matplotlib.pyplot as plt
from math import sqrt
import json

class Convert_to_utm:
    def __init__(self):
        self.loss_time = 0
        self.altitude = 0
        self.longitude = 0
        self.latitude = 0

    def loadData(self,name,deli):
        text = np.loadtxt(name,dtype=str,delimiter=deli)
        self.loss_time = text[:,0]
        self.latitude = text[:,1]
        self.longitude = text[:,2]
        self.altitude = text[:,3]
        self.altitude = np.array([float(self.altitude[i].split("m")[0]) for i in range(len(self.altitude))])
#        self.altitude = self.altitude.astype(float)

        self.loss_time = self.loss_time.astype(float)
        #print type(self.loss_time[0])
    
class Remove_outliers:
    def __init__(self):
        pass 
    def getVelocity(self,e1,n1, time): #altitude
        clean_e = np.array(e1)
        clean_n = np.array(n1)
        
        distance = np.zeros(len(clean_e)-1)
        for i in range(len(clean_e)-1):
            distance[i] = self.calcDistance(clean_e[i],clean_n[i],clean_e[i+1],clean_n[i+1])

        #print distance.shape
        #print time.shape
        velocity = np.divide(np.divide(distance,time[1:]),1000)

        return velocity
        
    def calcDistance(self,e1,n1,e2,n2):
        return sqrt((e1-e2)*(e1-e2)+(n1-n2)*(n1-n2))
        

    def outlierRemoval(self, velocity,e1,n1,time):#, altitude):
        vel = np.array([velocity[0]])
        ind = []
        e = np.array([e1[0],e1[1]])
        n = np.array([n1[0],n1[1]])
        t = np.array([time[0],time[1]])
        #a = np.array([altitude[0],altitude[1]])
        
        for i in range(1,len(velocity)):
            if velocity[i] < np.mean(velocity)+np.std(velocity) or velocity[i-1] == 0:
                e = np.append(e,e1[i+1])
                n = np.append(n,n1[i+1])
                t = np.append(t,time[i+1])
                #a = np.append(a,altitude[i])
                vel = np.append(vel,velocity[i])
        return vel, e, n, t#, a


    def whyatt(self,e1,n1,waypoints):
        
        n = len(e1)
        
        pts = np.array([[e1[x],n1[x]] for x in range(n)])

        simplifier = VWSimplifier(pts)
        VWpts = simplifier.from_number(waypoints)
        #print "Visvalingam: reduced to %s points" %(len(VWpts))
        #50 points in .131 seconds on my computer
        
        return VWpts


    def ramerDouglas(self,e1,n1,eps):
        n = len(e1)
        pts = np.array([[e1[x],n1[x]] for x in range(n)])
        #start=time()
        RDPpts = rdp(pts,epsilon=eps) #found by trail and error
        #end = time()
        print ("Ramer-Douglas-Peucker: to %s points" %(len(RDPpts)))
        #40 points in 1.35 seconds on my computer
        return RDPpts

class Json_groundcontrol:
    def __init__(self):
        pass
    def saveToFile(self,lat2,lon2,alt2):

        plan = {}
        geoFence = {}
        plan['fileType'] = 'Plan'

        geoFence['polygon'] = [] 
        geoFence['version'] = 1 
        plan['geoFence'] = geoFence

        plan['groundStation'] = 'QGroundControl'
        print(len(lat2))
        print(len(lon2))
        print(len(alt2))

        items = []

        item = {}
        item['autoContinue'] = True
        item['command'] = 22
        item['doJumpId'] = 1
        item['frame'] = 3
        item['params'] = [0,0,0,0,lat2[0],lon2[0],alt2[0]]
        item['type'] = 'SimpleItem'
        items.append (item)
        for i in range(1,len(lat2)):
            item = {}
            item['autoContinue'] = True
            item['command'] = 16
            item['doJumpId'] = i+1
            item['frame'] = 3
            item['params'] = [0,0,0,0,lat2[i],lon2[i],alt2[i]]
            item['type'] = 'SimpleItem'
            items.append (item)

        mission = {}
        mission['cruiseSpeed'] = 15
        mission['firmwareType'] = 3
        mission['hoverSpeed'] = 5
        mission['items'] = items
        mission['plannedHomePosition'] = [lat2[0], lon2[0], alt2[0]]
        mission['vehicleType'] = 2
        mission['version'] = 2
        plan['mission'] = mission

        rallyPoints = {}
        rallyPoints['points'] = [] 
        rallyPoints['version'] = 1 
        plan['rallyPoints'] = rallyPoints

        plan['version'] = 1

        plan_json = json.dumps(plan, indent=4, sort_keys=True)

        file = open('../diverse/mission.plan','w') 
        file.write (plan_json)
        file.close() 


if __name__ == "__main__":
    con = Convert_to_utm()
    con.loadData("../diverse/data.txt"," ")
    uc = utmconv()
    hemisphere = []
    zone = []
    letter = []
    e1 = []
    n1 = []
    for i in range(len(con.latitude)):
        (h, z, l, e, n) = uc.geodetic_to_utm(float(con.latitude[i]),float(con.longitude[i]))
        hemisphere.append(h)
        zone.append(z)
        letter.append(l)
        e1.append(e)
        n1.append(n)
    #print e1
    e_original = e1
    n_original = n1
    plt.scatter(e1,n1)
    plt.axis('equal')
    plt.title('Received track')
    plt.xlabel('east')
    plt.ylabel('north')
    plt.legend(['# of pts: ' + str(len(e1))])
    plt.show()
    altitude = con.altitude
    ro = Remove_outliers()
    velocity = ro.getVelocity(e1,n1,con.loss_time) #con.altitude
    velocity, e1, n1, time = ro.outlierRemoval(velocity,e1,n1,con.loss_time)
    #print("--------------------")
    #print velocity
    plt.scatter(e1,n1)
    plt.axis('equal')
    plt.title('Outlier removal')
    plt.xlabel('east')
    plt.ylabel('north')
    plt.legend(['# of pts: ' + str(len(e1))])
    plt.show()
    reduced_points = ro.whyatt(e1,n1,10) #number of points
    #print reduced_points
    #print reduced_points.T
    fig, axs = plt.subplots(2,1)
    fig.suptitle('Reduced pts, Whyatt - 10 waypoints',fontsize=16)
    fig.tight_layout(pad=5.0)
    axs[0].scatter(* reduced_points.T)
    axs[0].set_title('Scatter pts')
    axs[0].set_xlabel('east')
    axs[0].set_ylabel('north')
    axs[0].legend(['# of scatter pts: 10 (set)'])
    axs[1].plot(* reduced_points.T)
    axs[1].set_title('Connected path')
    axs[1].set_xlabel('east')
    axs[1].set_ylabel('north')
    axs[1].legend(['10 pts'])
    plt.show()
    reduced_points_r = ro.ramerDouglas(e1,n1,1.35) #based on epsilon
    fig, axs = plt.subplots(2,1)
    fig.suptitle('Reduced pts, RamerDouglas - e = 1.35m ',fontsize=16)
    fig.tight_layout(pad=5.0)
    axs[0].scatter(* reduced_points_r.T)
    axs[0].set_title('Scatter pts')
    axs[0].set_xlabel('east')
    axs[0].set_ylabel('north')
    axs[0].legend(['# of scatter pts: ' + str(len(reduced_points_r.T[0]))])
    axs[1].plot(* reduced_points_r.T)
    axs[1].set_title('Connected path')
    axs[1].set_xlabel('east')
    axs[1].set_ylabel('north')
    axs[1].legend([str(len(reduced_points_r.T[0])) + ' pts'])   
    plt.show()
    lat2 = []
    lon2 = []
    east, north = reduced_points.T
    for i in range(len(east)):
        (la2,lo2) = uc.utm_to_geodetic(hemisphere[i],zone[i],east[i],north[i])
        lat2.append(la2)
        lon2.append(lo2)
    
    print(lat2)
    print(lon2)
    
    
    alt2 = []
    last_index = 0
    for j in range(len(east)):
        for i in range(len(e_original)):
            if e_original[i] == east[j] and n_original[i] == north[j]:
                alt2.append(altitude[i])
                #last_index = i+1
                break
    print(alt2)
    jdog = Json_groundcontrol()

    jdog.saveToFile(lat2,lon2,alt2)


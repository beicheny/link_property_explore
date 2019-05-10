#!/usr/bin/env python
# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
# Copyright (C) 2009-2018 German Aerospace Center (DLR) and others.
# This program and the accompanying materials
# are made available under the terms of the Eclipse Public License v2.0
# which accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v20.html
# SPDX-License-Identifier: EPL-2.0

# @file    runner.py
# @author  Lena Kalleske
# @author  Daniel Krajzewicz
# @author  Michael Behrisch
# @author  Jakob Erdmann
# @date    2009-03-26
# @version $Id$

from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import math

Detectors = ["e1Detector_E12_0_0","e1Detector_E12_1_1","e1Detector_E32_1_2","e1Detector_E32_0_3",
             "e1Detector_E23_0_4","e1Detector_E23_1_5","e1Detector_E43_1_6","e1Detector_E43_0_7",
			 "e1Detector_E34_0_8","e1Detector_E34_1_9","e1Detector_E54_1_10","e1Detector_E54_0_11",
			 "e1Detector_E45_0_12","e1Detector_E45_1_13","e1Detector_E65_1_14","e1Detector_E65_0_15",
			 "e1Detector_E56_0_16","e1Detector_E56_1_17","e1Detector_E76_1_18","e1Detector_E76_0_19",
			 "e1Detector_E67_0_20","e1Detector_E67_1_21","e1Detector_E87_1_22","e1Detector_E87_0_23",
			 "e1Detector_E78_0_24","e1Detector_E78_1_25","e1Detector_E98_1_26","e1Detector_E98_0_27",
			 "e1Detector_E89_0_28","e1Detector_E89_1_29","e1Detector_E09_1_30","e1Detector_E09_0_31",
             "out_e1Detector_E21_0_0","out_e1Detector_E21_1_1","out_e1Detector_E32_1_2","out_e1Detector_E32_0_3",
             "out_e1Detector_E23_0_4","out_e1Detector_E23_1_5","out_e1Detector_E43_1_6","out_e1Detector_E43_0_7",
			 "out_e1Detector_E34_0_8","out_e1Detector_E34_1_9","out_e1Detector_E54_1_10","out_e1Detector_E54_0_11",
			 "out_e1Detector_E45_0_12","out_e1Detector_E45_1_13","out_e1Detector_E65_1_14","out_e1Detector_E65_0_15",
			 "out_e1Detector_E56_0_16","out_e1Detector_E56_1_17","out_e1Detector_E76_1_18","out_e1Detector_E76_0_19",
			 "out_e1Detector_E67_0_20","out_e1Detector_E67_1_21","out_e1Detector_E87_1_22","out_e1Detector_E87_0_23",
			 "out_e1Detector_E78_0_24","out_e1Detector_E78_1_25","out_e1Detector_E98_1_26","out_e1Detector_E98_0_27",
			 "out_e1Detector_E89_0_28","out_e1Detector_E89_1_29","out_e1Detector_E90_1_30","out_e1Detector_E90_0_31"]

# Intersection Positions
IntersectionNames = ['I2','I3','I4','I5','I6','I7','I8','I9'];
IntersectionPos = [[3185.72,13874.64],
                   [3215.89,13406.34],
                   [3415.16,12945.08],
                   [3651.59,12380.20],
                   [3391.99,10980.01],
                   [3173.08,10411.72],
                   [3055.19,10043.67],
                   [2957.41,9467.68]]
transmissionRange = 100

# ID for Edges Enter Intersection
edgeList = ['E12','E32','E23','E43','E34','E54','E45','E65','E56','E76','E67','E87','E78','E98','E89','E09']
edgeListOut = ['E21','E23','E32','E34','E43','E45','E54','E56','E65','E67','E76','E78','E87','E89','E98','E90']

# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import traci  # noqa

# The program looks like this
#    <tlLogic id="0" type="static" programID="0" offset="0">
# the locations of the tls are      NESW
#        <phase duration="31" state="GrGr"/>
#        <phase duration="6"  state="yryr"/>
#        <phase duration="31" state="rGrG"/>
#        <phase duration="6"  state="ryry"/>
#    </tlLogic>


def run():
    """execute the TraCI control loop"""
    step = 0
    # we start with phase 2 where EW has green
    # traci.trafficlight.setPhase("0", 2)
    traceIDFile = open("intervalIDBaseEdge.txt", "w")
    traceNumFile = open("intervalNumBasedEdge.txt", "w")
	
    firstLine = 'timestamp'
    trafficLightIDList = traci.trafficlight.getIDList()    
    for detector in Detectors:
        firstLine = firstLine + ';' + detector     
    for trafficLightID in trafficLightIDList:
        firstLine = firstLine + ';' + trafficLightID 
    for edge in edgeList:
        firstLine = firstLine + ';' + edge
    for edge in edgeListOut:
        firstLine = firstLine + ';' + 'out_' + edge
    traceIDFile.write(firstLine+'\n')
    traceNumFile.write(firstLine+'\n')
	
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()			
        currentLineID = '' + str(step)
        currentLineNum = '' + str(step)
        vehicleIDList = traci.vehicle.getIDList()            
        EdgeIntersectionsVehNums = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        EdgeOutIntersectionsVehNums = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
		
        for (index, edgeID) in enumerate(edgeList):
            vehicleIDList=traci.edge.getLastStepVehicleIDs(edgeID)
            for vehicleID in vehicleIDList:
                (x,y) = traci.vehicle.getPosition(vehicleID)
                j = int(edgeID[2]) - 2
                if math.pow(x - IntersectionPos[j][0], 2) + math.pow(y - IntersectionPos[j][1], 2) <= math.pow(transmissionRange,2):
                    EdgeIntersectionsVehNums[index] = EdgeIntersectionsVehNums[index] + 1

        for (index, edgeID) in enumerate(edgeListOut):
            vehicleIDList=traci.edge.getLastStepVehicleIDs(edgeID)
            for vehicleID in vehicleIDList:
                (x,y) = traci.vehicle.getPosition(vehicleID)
                j = int(edgeID[1]) - 2
                if math.pow(x - IntersectionPos[j][0], 2) + math.pow(y - IntersectionPos[j][1], 2) <= math.pow(transmissionRange,2):
                    EdgeOutIntersectionsVehNums[index] = EdgeOutIntersectionsVehNums[index] + 1
        
        print("Current Step: " + str(step))
        for detector in Detectors:
            vehID = traci.inductionloop.getLastStepVehicleIDs(detector)
            vehNum = traci.inductionloop.getLastStepVehicleNumber(detector)
            currentLineID = currentLineID + ';' + str(vehID)
            currentLineNum = currentLineNum + ';' + str(vehNum)
        for trafficLightID in trafficLightIDList:
            phase = traci.trafficlight.getPhase(trafficLightID)
            currentLineID = currentLineID + ';' + str(phase)
            currentLineNum = currentLineNum + ';' + str(phase)
        for EdgeIntersectionsVehNum in EdgeIntersectionsVehNums:
            currentLineID = currentLineID + ';' + str(EdgeIntersectionsVehNum)
            currentLineNum = currentLineNum + ';' + str(EdgeIntersectionsVehNum)
        for EdgeOutIntersectionsVehNum in EdgeOutIntersectionsVehNums:
            currentLineID = currentLineID + ';' + str(EdgeOutIntersectionsVehNum)
            currentLineNum = currentLineNum + ';' + str(EdgeOutIntersectionsVehNum)			
        traceIDFile.write(currentLineID + '\n')
        traceNumFile.write(currentLineNum + '\n')
        step += 1
    
    traceIDFile.close()
    traceNumFile.close()
    traci.close()
    sys.stdout.flush()


def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options


# this is the main entry point of this script
if __name__ == "__main__":
    options = get_options()

    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    #if options.nogui:
    #    sumoBinary = checkBinary('sumo')
    #else:
    #    sumoBinary = checkBinary('sumo-gui')

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    traci.start(['sumo', "-c", "AL69_withTraciVeh.sumocfg"])
    run()
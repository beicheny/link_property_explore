#!/usr/bin/env python
# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
# Copyright (C) 2009-2018 German Aerospace Center (DLR) and others.
# This program and the accompanying materials
# are made available under the terms of the Eclipse Public License v2.0
# which accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v20.html
# SPDX-License-Identifier: EPL-2.0

# @file    Route_Gen.py
# @author  Lena Kalleske
# @author  Daniel Krajzewicz
# @author  Michael Behrisch
# @author  Jakob Erdmann
# @date    2009-03-26
# @version $Id$

from __future__ import absolute_import
from __future__ import print_function

import optparse
import random
import math

def generate_routefile():
    random.seed(42)  # make tests reproducible
    N = 3600  # number of time steps
    pi = 3.14159

    with open("AL69_14intersections.rou.xml", "w") as routes:
        print("""<routes>
        <vType id="car" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="22.352" \
guiShape="passenger"/>

        <route id="down" edges="Eab Ebc Ec1 E12 E23 E34 E45 E56 E67 E78 E89 E90 E0d Ede Eef" />
        <route id="up" edges="Efe Eed Ed0 E09 E98 E87 E76 E65 E54 E43 E32 E21 E1c Ecb Eba" />
        """, file=routes)
        vehNr = 0
        for i in range(N):
            # demand per second from different directions
            pDown = 0.34694
            pUp = pDown    
            if random.uniform(0, 1) < pDown:
                print('    <vehicle id="down_%i" type="car" route="down" depart="%i" color="0,1,0"/>' % (
                    vehNr, i), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < pUp:
                print('    <vehicle id="up_%i" type="car" route="up" depart="%i" color="0,0,1"/>' % (
                    vehNr, i), file=routes)
                vehNr += 1
        print("</routes>", file=routes)


# this is the main entry point of this script
if __name__ == "__main__":
    # first, generate the route file for this simulation
    generate_routefile()

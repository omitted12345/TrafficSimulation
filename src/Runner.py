import math
import os
import sys
import optparse
import subprocess
import random

# taken from SUMO tutorials
# import python modules from the $SUMO_HOME/tools directory
try:
    sys.path.append(os.path.join(os.path.dirname(
        __file__), '..', '..', '..', '..', "tools"))  # tutorial in tests
    sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(
        os.path.dirname(__file__), "..", "..", "..")), "tools"))  # tutorial in docs
    sys.path.append("/usr/local/Cellar/sumo/1.2.0/share/sumo/tools")
    from sumolib import checkBinary  # noqa
except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")

import traci


# returns the hex part of a node id
def getNodeId(node):
    return str(hex(node)[2:])


# returns ids of neighbouring nodes
def getNodeNeighbours(node):
    n = []
    if node // JUNC_LENGTH == 0:
        n.append("1" + getNodeId(node))
    else:
        n.append("0" + getNodeId(node - JUNC_LENGTH))
    if node % JUNC_LENGTH == JUNC_LENGTH - 1:
        n.append("1" + getNodeId(JUNC_LENGTH + node // JUNC_LENGTH))
    else:
        n.append("0" + getNodeId(node + 1))
    if node // JUNC_LENGTH == JUNC_LENGTH - 1:
        n.append("1" + getNodeId(3 * JUNC_LENGTH - 1 - node % JUNC_LENGTH))
    else:
        n.append("0" + getNodeId(node + JUNC_LENGTH))
    if node % JUNC_LENGTH == 0:
        n.append("1" + getNodeId(4 * JUNC_LENGTH - 1 - node // JUNC_LENGTH))
    else:
        n.append("0" + getNodeId(node - 1))
    return n


def toXml(fileType, *args):
    args = args[0]
    if fileType == ".nod":
        s = "\t<node id=\"" + args[0] + "\" x=\"" + args[1] \
            + "\" y=\"" + args[2] + "\" type=\"" + args[3] + "\"/>\n"
    elif fileType == ".edg":
        s = "\t<edge id=\"" + args[0] + "to" + args[1] \
            + "\" from=\"" + args[0] + "\" to=\"" + args[1] \
            + "\" type=\"" + args[2] + "\"/>\n"
    elif fileType == ".det":
        if args[0][0] == '1' or args[1][0] == '1':
            length = args[2] - 12
        else:
            length = args[2] - 22
        s = "\t<e2Detector id=\"det" + args[0] + "to" + args[1] + "-" + str(args[3]) \
            + "\" lane=\"" + args[0] + "to" + args[1] \
            + "_" + str(args[3]) + "\" pos=\"0\" length=\"" + str(length) \
            + "\" freq=\"" + str(PHASE_LENGTH) + "\" file=\"sim.out\" friendlyPos=\"x\"/>\n"
    else:
        raise TypeError("fileType must be .nod, .edg or .det")

    return s


# returns the relative coordinates of a given node, (0,0) is top-left
def toCoords(nodeId):
    nodeNum = int(nodeId[1:], 16)
    # inner nodes
    if nodeId[0] == '0':
        # column of int value of node plus 1 to accomodate for outer nodes
        x = nodeNum % JUNC_LENGTH + 1
        # same but for rows
        y = nodeNum % JUNC_LENGTH + 1
    # outer nodes
    else:
        # top
        if nodeNum // JUNC_LENGTH == 0:
            x = nodeNum + 1
            y = 0
        # right
        elif nodeNum // JUNC_LENGTH == 1:
            x = JUNC_LENGTH + 1
            y = nodeNum - JUNC_LENGTH + 1
        # bottom
        elif nodeNum // JUNC_LENGTH == 2:
            x = 3 * JUNC_LENGTH - nodeNum
            y = JUNC_LENGTH + 1
        # left
        else:
            x = 0
            y = 4 * JUNC_LENGTH - nodeNum
    return (x, y)


# returns the node corresponding to the given coords
def fromCoords(coords):
    x = coords[0]
    y = coords[1]
    # outer top
    if y == 0:
        node = "1" + getNodeId(x - 1)
    # outer right       
    elif x == JUNC_LENGTH + 1:
        node = "1" + getNodeId(y + JUNC_LENGTH - 1)
    # outer bottom
    elif y == JUNC_LENGTH + 1:
        node = "1" + getNodeId(3 * JUNC_LENGTH - x)
    # outer left       
    elif x == 0:
        node = "1" + getNodeId(4 * JUNC_LENGTH - y)
    # inner
    else:
        node = "0" + getNodeId((x - 1) + (y - 1) * JUNC_LENGTH)
    return node


# returns the total pressure release of a phase given in SUMO format on a node with given pressures
def pressureRelease(phase, pressures):
    total = 0
    for i in range(0, len(phase)):
        if phase[i] == "G":
            total += pressures[i]
    return total


# generate node file
def genNodes():
    # length of junction block
    l = int(math.sqrt(NUM_JUNCTIONS))
    # perimeter of junction block
    p = l * 4
    # scalar transform for keeping coordinates around 0
    coordNorm = (l - 1) // 2
    nodFile = open("data/sim.nod.xml", "w")
    entriesX = [0] * p
    # define x positions for outer nodes
    for i in range(0, l):
        # top
        entriesX[i] = "\"" + str(EDGE_LENGTH * (i - coordNorm)) + "\""
        # right
        entriesX[i + l] = "\"" + str(EDGE_LENGTH * (l - coordNorm)) + "\""
        # bottom
        entriesX[i + 2 * l] = "\"" + str(-EDGE_LENGTH * (i - l // 2)) + "\""
        # left
        entriesX[i + 3 * l] = "\"" + str(-EDGE_LENGTH * (l - l // 2)) + "\""
    # y positions are reverse x * -1
    # essentially iterate through entriesX backwards and multiply by -1, then add to entriesY
    entriesY = ["\"" + str(int(x[1:-1]) * -1) + "\"" for x in entriesX[::-1]]

    nodFile.write("<nodes>\n")
    # create junctions
    for i in range(0, NUM_JUNCTIONS):
        nodFile.write("\t<node id=\"0" + getNodeId(i) + "\" x=\"" + str(EDGE_LENGTH * ((i % l) - coordNorm))
                      + "\" y=\"" + str(-EDGE_LENGTH * ((i // l) - coordNorm)) + "\" type=\"traffic_light\"/>\n")
    nodFile.write("\n")
    # create outer nodes
    for i in range(0, p):
        nodFile.write("\t<node id=\"1" + getNodeId(i) + "\" x=" + entriesX[i]
                      + " y=" + entriesY[i] + " type=\"priority\"/>\n")
    nodFile.write("</nodes>")
    nodFile.close()


# generate edge file and det file
def betterGenEdges():
    edgFile = open("data/sim.edg.xml", "w")
    detFile = open("data/sim.det.xml", "w")
    edgFile.write("<edges>\n")
    detFile.write("<additional>\n")
    links = set()
    for node in range(0, NUM_JUNCTIONS):
        for neighbour in getNodeNeighbours(node):
            links.update([(neighbour, "0" + getNodeId(node)), ("0" + getNodeId(node), neighbour)])
    for link in links:
        edgFile.write(toXml(".edg", [link[0], link[1], "junction_out"]))
        for i in range(0, 2):
            detFile.write(toXml(".det", [link[0], link[1], EDGE_LENGTH, i]))
    edgFile.write("</edges>")
    detFile.write("</additional>")
    edgFile.close()
    detFile.close()


# generate links to be removed from left-turn lanes to other lanes
def genConnections():
    edgFile = open("data/sim.edg.xml", "r")
    conFile = open("data/sim.con.xml", "w")
    conFile.write("<connections>\n")
    for node in range(0, NUM_JUNCTIONS):
        neighbours = getNodeNeighbours(node)
        for i in range(0, 4):
            conFile.write("\t<delete from=\"" + neighbours[i] + "to0" + getNodeId(node) + "\" to=\"0" \
                          + getNodeId(node) + "to" + neighbours[(i + 2) % 4] + "\" fromLane=\"1\" toLane=\"0\"/>\n")
            conFile.write("\t<delete from=\"" + neighbours[i] + "to0" + getNodeId(node) + "\" to=\"0" \
                          + getNodeId(node) + "to" + neighbours[(i + 2) % 4] + "\" fromLane=\"1\" toLane=\"1\"/>\n")

    conFile.write("</connections>")
    conFile.close()
    edgFile.close()


# generate traffic light phases
def genTlLogic():
    tllFile = open("data/sim.add.xml", "w")
    tllFile.write("<additional>\n")
    for node in range(0, NUM_JUNCTIONS):
        tllFile.write("\t<tlLogic id=\"0" + getNodeId(node) + "\" type=\"static\" programID=\"runner\" offset=\"0\">\n")
        for i in range(0, len(PHASES)):
            if i % 2 == 0:
                dur = PHASE_LENGTH - 3
            else:
                dur = 3
            tllFile.write("\t\t<phase duration=\"" + str(dur) + "\" state=\"" + PHASES[i] + "\"/>\n")
        tllFile.write("</tlLogic>\n")
    tllFile.write("</additional>")
    tllFile.close()


# randomly generate routes for vehicles to take across the network
def genRoutes():
    # proportion of vehicles to be buses
    busProp = 0.0
    rouFile = open("data/sim.rou.xml", "w")
    rouFile.write("<routes>\n")
    rouFile.write(
        "\t<vType id=\"typeCar\" accel=\"0.8\" decel=\"4.5\" sigma=\"0.5\" length=\"5\" minGap=\"2.5\" maxSpeed=\"16.67\" guiShape=\"passenger\"/>\n"
        + "\t<vType id=\"typeBus\" accel=\"0.8\" decel=\"4.5\" sigma=\"0.5\" length=\"7\" minGap=\"3\" maxSpeed=\"25\" guiShape=\"bus\"/>\n\n")
    # list of routes vehicles can take, routes are a list of edges
    routes = []
    # list of pairs of inputs and outputs that already form a route
    inUse = []
    for i in range(0, NUM_ROUTES):
        # randomly select entry and exit
        inpNode = random.randint(0, JUNC_PERIMETER - 1)
        outNode = inpNode
        while outNode == inpNode or (inpNode, outNode) in inUse:
            outNode = random.randint(0, JUNC_PERIMETER - 1)
        inUse.append((inpNode, outNode))
        inpNode = "1" + getNodeId(inpNode)
        outNode = "1" + getNodeId(outNode)
        routes.append([inpNode])

        # find route to get from entry to exit
        current = list(toCoords(inpNode))
        # one phase to enter the network
        if min(current) == 0:
            current[current.index(min(current))] += 1
        else:
            current[current.index(max(current))] -= 1
        routes[i].append(fromCoords(current))

        target = toCoords(outNode)
        # navigate across x and then y until same coords as target
        while current != list(target):
            # only move if not leaving network unless reaching target
            if current[0] < target[0] and (
                    current[0] < JUNC_LENGTH or (current[0] + 1 == target[0] and current[1] == target[1])):
                current[0] += 1
                routes[i].append(fromCoords(current))
                continue
            if current[0] > target[0] and (current[0] > 1 or (current[0] - 1 == target[0] and current[1] == target[1])):
                current[0] -= 1
                routes[i].append(fromCoords(current))
                continue
            if current[1] < target[1] and (
                    current[1] < JUNC_LENGTH or (current[1] + 1 == target[1] and current[0] == target[0])):
                current[1] += 1
                routes[i].append(fromCoords(current))
                continue
            if current[1] > target[1] and (current[1] > 1 or (current[1] - 1 == target[1] and current[0] == target[0])):
                current[1] -= 1
                routes[i].append(fromCoords(current))
                continue

    for route in routes:
        edges = []
        for i in range(0, len(route) - 1):
            edges.append(route[i] + "to" + route[i + 1])
        rouFile.write("\t<route id=\"" + route[0] + "to" + route[-1] \
                      + "\" edges=\"" + " ".join(edges) + "\"/>\n")

    rouFile.write("\n\n")
    for i in range(0, NUM_VEHICLES):
        vehRand = random.randint(1, 10)
        rouRand = random.randint(0, NUM_ROUTES - 1)
        timeRand = random.randint(-1, 1)
        if vehRand / 10 <= busProp:
            veh = "typeBus"
        else:
            veh = "typeCar"
        rouFile.write("\t<vehicle id=\"" + str(i) + "\" type=\"" + veh + "\" route=\"" \
                      + routes[rouRand][0] + "to" + routes[rouRand][-1] + "\" depart=\"" \
                      + str((i + 1) * 2 + timeRand) + "\"/>\n")

    rouFile.write("</routes>")
    rouFile.close()


# generate files and run simulation
def genFiles():
    # validation
    if NUM_JUNCTIONS < 0 or math.sqrt(NUM_JUNCTIONS) % 1 != 0:
        raise ValueError("NUM_JUNCTIONS must be a square number")
    if EDGE_LENGTH < 1:
        raise ValueError("EDGE_LENGTH must be at least 1")

    # generate files
    genNodes()
    betterGenEdges()
    genConnections()
    genTlLogic()
    genRoutes()


def backPressure(node):
    # list of queue lengths ordered clockwise from 12:00, input lanes followed by output lanes
    queues = []
    # list of pressures ordered in triplets of pressures for input lanes ordered clockwise
    pressures = []
    neighbours = getNodeNeighbours(node)
    # find all queue lengths (pressures)
    for n in neighbours:
        queues += [(traci.lanearea.getLastStepVehicleNumber("det" + n + "to0" + getNodeId(node) + "-0"))] * 2
        queues += [(traci.lanearea.getLastStepVehicleNumber("det" + n + "to0" + getNodeId(node) + "-1"))]
        queues += [(traci.lanearea.getLastStepVehicleNumber("det0" + getNodeId(node) + "to" + n + "-0")) + (
            traci.lanearea.getLastStepVehicleNumber("det0" + getNodeId(node) + "to" + n + "-1"))]

    # calculate all pressure differences
    for i in range(0, 8, 2):
        # even queues are input lanes
        for j in range(0, 3):
            # odd queues are output lanes
            # calculate differences between input lane and its respective output lane
            pressures.append(max(queues[i * 2 + j] - queues[((i * 2 + 3) + 4 * (3 - j)) % 16], 0))

    # find phase with greatest pressure release
    r = 0
    p = 0
    for i in range(0, len(PHASES)):
        pressure = pressureRelease(PHASES[i], pressures)
        if pressure > r:
            r = pressure
            p = i
    return p


# adapted from SUMO tutorials        
def run():
    """execute the TraCI control loop"""
    step = 0
    waits = []
    for i in range(0, NUM_JUNCTIONS):
        waits.append([0] * len(PHASES))
    while traci.simulation.getMinExpectedNumber() > 0: #step <= 900:
        traci.simulationStep()
        if step % PHASE_LENGTH == 0:
            for i in range(0, NUM_JUNCTIONS):
                if MAX_WAIT in waits[i]:
                    phase = waits[i].index(MAX_WAIT)
                else:
                    phase = backPressure(i)
                traci.trafficlight.setPhase("0" + getNodeId(i), phase)
                for j in range(0, len(PHASES), 2):
                    if j == phase:
                        waits[i][j] = 0
                    else:
                        if waits[i][j] < MAX_WAIT:
                            waits[i][j] += 1
        step += 1
    traci.close()
    sys.stdout.flush()


# taken from SUMO tutorials
def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options


# adapted from SUMO tutorials
# main entry point
if __name__ == "__main__":
    options = get_options()

    # constants
    NUM_JUNCTIONS = 9  # int(sys.argv[1])
    JUNC_LENGTH = int(math.sqrt(NUM_JUNCTIONS))
    JUNC_PERIMETER = JUNC_LENGTH * 4
    EDGE_LENGTH = 200  # int(sys.argv[2])
    PHASES = ["GGrrrrGGrrrr", "yyrrrryyrrrr", "rrrGGrrrrGGr", "rrryyrrrryyr",
              "rrGrrrrrGrrr", "rryrrrrryrrr", "rrrrrGrrrrrG", "rrrrryrrrrry"]
    NUM_ROUTES = 90  # int(sys.argv[3])
    NUM_VEHICLES = 1000  # int(sys.argv[4])
    PHASE_LENGTH = 50
    MAX_WAIT = 6

    # set seed for reproducable results
    random.seed(100)

    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    netconvertBinary = checkBinary('netconvert')

    genFiles()

    p = subprocess.Popen([netconvertBinary, "-c", "sim.netccfg", "--no-turnarounds.tls", "true"],
                         stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                         cwd=os.getcwd() + "/data")

    p.wait()

    traci.start(
        [sumoBinary, "-c", "data/sim.sumocfg", "--tripinfo-output", "tripinfo.xml", "--summary", "data/sim.sum"])
    run()

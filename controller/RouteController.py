from abc import ABC, abstractmethod
import math
import random
import os
import sys
from core.Util import *
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("No environment variable SUMO_HOME!")
import traci
import sumolib

STRAIGHT = "s"
TURN_AROUND = "t"
LEFT = "l"
RIGHT = "r"
SLIGHT_LEFT = "L"
SLIGHT_RIGHT = "R"




class RouteController(ABC):
    """
    Base class for routing policy

    To implement a scheduling algorithm, implement the make_decisions() method.
    Please use the boilerplate code from the example, and implement your algorithm between
    the 'Your algo...' comments.

    make_decisions takes in a list of vehicles and network information (connection_info).
        Using this data, it should return a dictionary of {vehicle_id: decision}, where "decision"
        is one of the directions defined by SUMO (see constants above). Any scheduling algorithm
        may be injected into the simulation, as long as it is wrapped by the RouteController class
        and implements the make_decisions method.

    :param connection_info: object containing network information, including:
                            - out_going_edges_dict {edge_id: {direction: out_edge}}
                            - edge_length_dict {edge_id: edge_length}
                            - edge_index_dict {edge_index_dict} keep track of edge ids by an index
                            - edge_vehicle_count {edge_id: number of vehicles at edge}
                            - edge_list [edge_id]

    """
    def __init__(self, connection_info: ConnectionInfo):
        self.connection_info = connection_info
        self.direction_choices = [STRAIGHT, TURN_AROUND,  SLIGHT_RIGHT, RIGHT, SLIGHT_LEFT, LEFT]

    def compute_local_target(self, decision_list, vehicle):
        current_target_edge = vehicle.current_edge
        try:
            path_length = 0
            i = 0

            #the while is used to make sure the vehicle will not assume it arrives the destination beacuse the target edge is too short.
            while path_length <= max(vehicle.current_speed, 20):
                if current_target_edge == vehicle.destination:
                    break
                if i >= len(decision_list):
                    raise UserWarning(
                        "Not enough decisions provided to compute valid local target. TRACI will remove vehicle."
                    )

                choice = decision_list[i]
                if choice not in self.connection_info.outgoing_edges_dict[current_target_edge]:
                    raise UserWarning(
                            "Invalid direction. TRACI will remove vehicle."
                        )
                current_target_edge = self.connection_info.outgoing_edges_dict[current_target_edge][choice]
                path_length += self.connection_info.edge_length_dict[current_target_edge]

                if i > 0:
                    if decision_list[i - 1] == decision_list[i] and decision_list[i] == 't':
                        # stuck in a turnaround loop, let TRACI remove vehicle
                        return current_target_edge

                i += 1

        except UserWarning as warning:
            print(warning)

        return current_target_edge


    @abstractmethod
    def make_decisions(self, vehicles, connection_info):
        pass

def midpoint(points):
        """
        Finds the midpoint between multiple points.

        Args:
            points: A list of points.

        Returns:
            The midpoint of the points.
        """

        x = sum([x for x, y in points]) / len(points)
        y = sum([y for x, y in points]) / len(points)

        return (x, y)
    
"""def findDistanceOfEdge(connection_info, edge):
    network = sumolib.net.readNet(connection_info.net_filename, withInternal=True)
    midFrom = midpoint(network.getEdge(edge).getFromNode().getShape())
    midTo = midpoint(network.getEdge(edge).getToNode().getShape())
    distance = math.hypot(midFrom[0] - midTo[0], midFrom[1] - midTo[1])
    return distance"""

def findDistanceBetweenEdges(connection_info, edge1, edge2):
    network = sumolib.net.readNet(connection_info.net_filename, withInternal=True)
    midTo = midpoint(network.getEdge(edge1).getToNode().getShape())
    midFrom = midpoint(network.getEdge(edge2).getFromNode().getShape())
    distance = math.hypot(midTo[0] - midFrom[0], midTo[1] - midFrom[1])
    return distance
    
class RandomPolicy(RouteController):
    """
    Example class for a custom scheduling algorithm.
    Utilizes a random decision policy until vehicle destination is within reach,
    then targets the vehicle destination.
    """
    def __init__(self, connection_info):
        super().__init__(connection_info)
        
    
    
    def make_decisions(self, vehicles, connection_info):
        """
        A custom scheduling algorithm can be written in between the 'Your algo...' comments.
        -For each car in the vehicle batch, your algorithm should provide a list of future decisions.
        -Sometimes short paths result in the vehicle reaching its local TRACI destination before reaching its
         true global destination. In order to counteract this, ask for a list of decisions rather than just one.
        -This list of decisions is sent to a function that returns the 'closest viable target' edge
          reachable by the decisions - it is not the case that all decisions will always be consumed.
          As soon as there is enough distance between the current edge and the target edge, the compute_target_edge
          function will return.
        -The 'closest viable edge' is a local target that is used by TRACI to control vehicles
        -The closest viable edge should always be far enough away to ensure that the vehicle is not removed
          from the simulation by TRACI before the vehicle reaches its true destination

        :param vehicles: list of vehicles to make routing decisions for
        :param connection_info: object containing network information
        :return: local_targets: {vehicle_id, target_edge}, where target_edge is a local target to send to TRACI
        """

        local_targets = {}
        #print('PRINTING MY PRINTS')
        x = 0
        for vehicle in vehicles:
            start_edge = vehicle.current_edge
            print(vehicle)
            print("STARTING edge: ", start_edge)
            print("DISTANCE OF EDGE: ", sumolib.net.readNet(connection_info.net_filename, withInternal=True).getEdge(start_edge).getLength())
            print("DESTINATION edge: ", vehicle.destination)
            print("DISTANCE BETWEEN EDGE AND DESTINATION: ", findDistanceBetweenEdges(connection_info, start_edge, vehicle.destination))
            print("From Node of Destination Edge: ", sumolib.net.readNet(connection_info.net_filename, withInternal=True).getEdge(vehicle.destination).getFromNode().getShape())

            '''
            Your algo starts here
            '''
            decision_list = []
            #create a decision list for each car based on them going through the map in the fastest scenario possible
            current_edge = start_edge
            i = 0
            visitedEdges = []
            visitedEdges.append(start_edge)
            blackListedEdges = []
            while i < 20:  # choose the number of decisions to make in advanced; depends on the algorithm and network
                skip = False
                # dead end
                if len(self.connection_info.outgoing_edges_dict[current_edge].keys()) == 0:
                    print("DED END")
                    break

                # make sure to check if it's a valid edge
                if len(self.connection_info.outgoing_edges_dict[current_edge].keys()) != 0:
                    print("Making Choice")
                    possibleChoices = list(self.connection_info.outgoing_edges_dict[current_edge].keys())
                    print(possibleChoices)
                    bestPossibleChoice = None
                    for x, choice in enumerate(possibleChoices):
                        probableEdge = self.connection_info.outgoing_edges_dict[current_edge][choice]
                        print(probableEdge, len(str(probableEdge)))
                        print(vehicle.destination, len(str(vehicle.destination)))
                        print("CHOICE: ", choice)
                        if (len((self.connection_info.outgoing_edges_dict[probableEdge].keys())) != 0) or (self.connection_info.outgoing_edges_dict[current_edge][choice] == vehicle.destination): #might remove probable edge # and probableEdge not in visitedEdges
                            firstTime = False
                            if (bestPossibleChoice is None):
                                bestPossibleChoice = choice
                                firstTime = True
                            if (self.connection_info.outgoing_edges_dict[current_edge][choice] == vehicle.destination):
                                bestPossibleChoice = choice
                                print("destination reached - breaking")
                                break
                            if ((probableEdge in blackListedEdges) and (self.connection_info.outgoing_edges_dict[current_edge][bestPossibleChoice] not in blackListedEdges) and (probableEdge != self.connection_info.outgoing_edges_dict[current_edge][bestPossibleChoice])):
                                print("Probable Edge is Blacklisted so will not be considered")
                                continue
                            elif ((probableEdge not in blackListedEdges) and (self.connection_info.outgoing_edges_dict[current_edge][bestPossibleChoice] in blackListedEdges) and (probableEdge != self.connection_info.outgoing_edges_dict[current_edge][bestPossibleChoice])):
                                print('Old Edge Black Listed, so forcible choosing the other')
                                bestPossibleChoice = choice
                                continue
                            elif ((probableEdge in blackListedEdges) and (self.connection_info.outgoing_edges_dict[current_edge][bestPossibleChoice] in blackListedEdges) and (probableEdge != self.connection_info.outgoing_edges_dict[current_edge][bestPossibleChoice])):
                                print("both blacklisted, look for new route by taking worse option")
                                if(findDistanceBetweenEdges(connection_info, probableEdge, vehicle.destination) > findDistanceBetweenEdges(connection_info, self.connection_info.outgoing_edges_dict[current_edge][bestPossibleChoice], vehicle.destination)):
                                    bestPossibleChoice = choice
                                continue
                            elif((str(probableEdge)[-4:] in str(vehicle.destination)) and (len(str(probableEdge)) != len(str(vehicle.destination))) and (probableEdge not in blackListedEdges)):
                                print('Black Listing Edge: ', probableEdge)
                                blackListedEdges.append(probableEdge)
                                if(len(decision_list) > 0):
                                    print("Black Listing Edge and Back Tracking Once")
                                    latestDecision = decision_list.pop()
                                    blackListedEdges.append(latestDecision)#remove the last decision
                                    current_edge = start_edge
                                    for decision in decision_list:
                                        current_edge = self.connection_info.outgoing_edges_dict[current_edge][decision]
                                    skip = True
                                    while True:
                                        if (len(list(self.connection_info.outgoing_edges_dict[current_edge].keys())) == 1 and len(decision_list) > 0):
                                            print("Back Tracking More Than Once")
                                            latestDecision = decision_list.pop()
                                            blackListedEdges.append(latestDecision)#remove the last decision
                                            current_edge = start_edge
                                            for decision in decision_list:
                                                current_edge = self.connection_info.outgoing_edges_dict[current_edge][decision]
                                        else:
                                            break
                                    break
                                #go back to the previous edge
                                skip = True
                            elif (findDistanceBetweenEdges(connection_info, probableEdge, vehicle.destination) < findDistanceBetweenEdges(connection_info, self.connection_info.outgoing_edges_dict[current_edge][bestPossibleChoice], vehicle.destination)): #finds best choice
                                if(probableEdge in blackListedEdges):
                                    print('BLACK LISTED')
                                    continue
                                else:    
                                    print("Passed black listing proccess")
                                    bestPossibleChoice = choice
                    if(not skip):
                        print("BEST POSSIBLE CHOICE: ", bestPossibleChoice)
                        decision_list.append(bestPossibleChoice)
                        current_edge = self.connection_info.outgoing_edges_dict[current_edge][bestPossibleChoice]
                    
                    if i > 0 and i<len(decision_list):
                        if decision_list[i-1] == decision_list[i] and decision_list[i] == 't':
                            # stuck in a turnaround loop, let TRACI remove vehicle
                            print("turn around loop?")
                            break  
                    i += 1
                if current_edge==vehicle.destination:
                    print("FOUND DESTINATION")
                    break
                print("DISTANCE BETWEEN EDGE AND DESTINATION: ", findDistanceBetweenEdges(connection_info, current_edge, vehicle.destination))
                print("------------------------------------")
                if(i == 20):
                    print("PROBLEM AREA")

            '''
            Your algo ends here
            '''
            local_targets[vehicle.vehicle_id] = self.compute_local_target(decision_list, vehicle)

        return local_targets

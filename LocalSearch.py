from Model_Emm_Emm import *
# from SolutionDrawer import *

class Solution:
    def __init__(self):
        self.cost = 0.0
        self.routes = []

# Addition of classes for Local Research
class RelocationMove(object):
    def __init__(self):
        self.originRoutePosition = None
        self.targetRoutePosition = None
        self.originNodePosition = None
        self.targetNodePosition = None
        self.costChangeOriginRt = None
        self.costChangeTargetRt = None
        self.moveCost = None

    def Initialize(self):
        self.originRoutePosition = None
        self.targetRoutePosition = None
        self.originNodePosition = None
        self.targetNodePosition = None
        self.costChangeOriginRt = None
        self.costChangeTargetRt = None
        
        self.moveCost = - 10 ** 9


class SwapMove(object):
    def __init__(self):
        self.positionOfFirstRoute = None
        self.positionOfSecondRoute = None
        self.positionOfFirstNode = None
        self.positionOfSecondNode = None
        self.costChangeFirstRt = None
        self.costChangeSecondRt = None
        self.moveCost = None
    def Initialize(self):
        self.positionOfFirstRoute = None
        self.positionOfSecondRoute = None
        self.positionOfFirstNode = None
        self.positionOfSecondNode = None
        self.costChangeFirstRt = None
        self.costChangeSecondRt = None
        self.moveCost = 10 ** 9


class CustomerInsertion(object):
    def __init__(self):
        self.customer = None
        self.route = None
        self.cost = 10 ** 9

class CustomerInsertionAllPositions(object):
    def __init__(self):
        self.customer = None
        self.route = None
        self.insertionPosition = None
        self.cost = 10 ** 9

class TwoOptMove(object):
    def __init__(self):
        self.positionOfFirstRoute = None
        self.positionOfSecondRoute = None
        self.positionOfFirstNode = None
        self.positionOfSecondNode = None
        self.moveCost = None
    def Initialize(self):
        self.positionOfFirstRoute = None
        self.positionOfSecondRoute = None
        self.positionOfFirstNode = None
        self.positionOfSecondNode = None
        self.moveCost = -10 ** 9

# Local Search 
class LocalSolver():
    def __init__(self, m):
        self.allNodes = m.allNodes
        self.customers = m.customers
        self.depot = m.allNodes[0]
        self.distanceMatrix = m.dist_matrix
        self.capacity = m.capacity
        self.empty_vehicle_weight = m.empty_vehicle_weight
        self.sol = None
        self.bestSolution = None

    def LocalSolve(self, sol):
        self.sol = sol
        self.ReportSolution(sol)
        self.LocalSearch()
        self.ReportSolution(sol)
        return self.sol
    
    def LocalSearch(self):
        self.bestSolution = self.cloneSolution(self.sol)
        terminationCondition = False
        localSearchIterator = 0
        operator = 2
        rm = RelocationMove()
        sm = SwapMove()
        top = TwoOptMove()

        while terminationCondition is False:

            self.InitializeOperators(rm, sm, top)
            #SolDrawer.draw(localSearchIterator, self.sol, self.allNodes)

            # Relocations
            if operator == 0:
                self.FindBestRelocationMove(rm)
                if rm.moveCost > 0:
                    self.ApplyRelocationMove(rm)
                    operator = 0
                else:
                    terminationCondition = True
            # Swaps
            elif operator == 1:
                self.FindBestSwapMove(sm)
                if sm.positionOfFirstRoute is not None:
                    if sm.moveCost < 0:
                        self.ApplySwapMove(sm)
                    else:
                        terminationCondition = True
            elif operator == 2:
                self.FindBestTwoOptMove(top)
                if top.positionOfFirstRoute is not None:
                    if top.moveCost > 0:
                        self.ApplyTwoOptMove(top)
                        operator = 2
                    else:
                        operator = 0

            self.TestSolution()

            if (self.sol.cost < self.bestSolution.cost):
                self.bestSolution = self.cloneSolution(self.sol)

            localSearchIterator = localSearchIterator + 1
            print(localSearchIterator, self.sol.cost)

        self.sol = self.bestSolution

    def cloneRoute(self, rt:Route):
        cloned = Route(self.depot, self.capacity)
        cloned.cost = rt.cost
        cloned.load = rt.load
        cloned.sequenceOfNodes = rt.sequenceOfNodes.copy()
        return cloned

    def cloneSolution(self, sol: Solution):
        cloned = Solution()
        for i in range (0, len(sol.routes)):
            rt = sol.routes[i]
            clonedRoute = self.cloneRoute(rt)
            cloned.routes.append(clonedRoute)
        cloned.cost = self.sol.cost
        return cloned

    def FindBestRelocationMove(self, rm):
        diff = []
        for originRouteIndex in range(0, len(self.sol.routes)):
            rt1:Route = self.sol.routes[originRouteIndex]
            # for originNodeIndex in range(1, len(rt1.sequenceOfNodes) - 1):
            for originNodeIndex in range(1, len(rt1.sequenceOfNodes)):
                for targetRouteIndex in range (0, len(self.sol.routes)):
                    rt2:Route = self.sol.routes[targetRouteIndex]
                    # for targetNodeIndex in range (0, len(rt2.sequenceOfNodes) - 1):
                    for targetNodeIndex in range (0, len(rt2.sequenceOfNodes)):

                        if originRouteIndex == targetRouteIndex and (targetNodeIndex == originNodeIndex or targetNodeIndex == originNodeIndex - 1):
                            continue

                        A = rt1.sequenceOfNodes[originNodeIndex - 1]
                        B = rt1.sequenceOfNodes[originNodeIndex]
                        if originNodeIndex + 1 < len(rt1.sequenceOfNodes):
                            C = rt1.sequenceOfNodes[originNodeIndex + 1]
                        else:
                            # Εναλλακτική διαχείριση ή συνέχεια χωρίς ενέργεια
                            continue

                        F = rt2.sequenceOfNodes[targetNodeIndex]
                        if targetNodeIndex + 1 < len(rt2.sequenceOfNodes):
                            G = rt2.sequenceOfNodes[targetNodeIndex + 1]
                        else:
                            continue
                        if rt1 != rt2:
                            if rt2.load + B.demand > rt2.capacity:
                                continue
                        # costAdded = self.distanceMatrix[A.ID][C.ID] + self.distanceMatrix[F.ID][B.ID] + self.distanceMatrix[B.ID][G.ID]
                        # costRemoved = self.distanceMatrix[A.ID][B.ID] + self.distanceMatrix[B.ID][C.ID] + self.distanceMatrix[F.ID][G.ID]
                        if originRouteIndex == targetRouteIndex :
                            oldcost = rt1.cost
                            rtcopy = rt1.copy()
                            bcopy = rtcopy.sequenceOfNodes.pop(originNodeIndex)
                            if (originNodeIndex < targetNodeIndex): 
                                rtcopy.sequenceOfNodes.insert(targetNodeIndex, bcopy)
                            else:
                                rtcopy.sequenceOfNodes.insert(targetNodeIndex + 1, bcopy) 
                            rtcopy.cost, rtcopy.load = self.calculate_route_details(rtcopy.sequenceOfNodes)
                            newcost = rtcopy.cost
                            originRtCostChange = oldcost - newcost
                            targetRtCostChange = originRtCostChange 
                        else:   
                            # rt1oldcost = rt1.cost
                            # rt2oldcost = rt2.cost
                            # oldcost = rt1oldcost + rt2oldcost
                            # rt1copy = rt1.copy()
                            # rt2copy = rt2.copy()
                            # bcopy = rt1copy.sequenceOfNodes.pop(originNodeIndex) 
                            # rt2copy.sequenceOfNodes.insert(targetNodeIndex + 1, bcopy)
                            # rt1copy.cost, rt1copy.load = self.calculate_route_details(rt1copy.sequenceOfNodes)
                            # rt1newcost = rt1copy.cost
                            # rt2copy.cost, rt2copy.load = self.calculate_route_details(rt2copy.sequenceOfNodes)
                            # rt2newcost = rt2copy.cost
                            # newcost = rt1newcost + rt2newcost
                            originRtCostChange, targetRtCostChange = self.calculate_relocation_cost(rt1, rt2, originNodeIndex, targetNodeIndex)
                            # altcost = originRtCostChange + targetRtCostChange
                            # difference = oldcost - newcost - altcost
                            # difference = round(difference, 10)
                            # diff.append(difference)
                            # print('Relocating Node ' + str(rt1.sequenceOfNodes[originNodeIndex].ID) + ' at ' + str (rt2.sequenceOfNodes[targetNodeIndex].ID))
                            # print('For Origin Route: Actual Cost = ' + str(rt1oldcost - rt1newcost) + ' Alt Cost = ' + str(originRtCostChange))
                            # print('For Target Route: Actual Cost = ' + str(rt2oldcost - rt2newcost) + ' Alt Cost = ' + str(targetRtCostChange))
                            # print('Actual Cost = ' + str(oldcost - newcost) + ' Alt Cost = ' + str(altcost))
                            # print('Difference = ' + str(oldcost - newcost - altcost))
                            # originRtCostChange = self.distanceMatrix[A.ID][C.ID] - self.distanceMatrix[A.ID][B.ID] - self.distanceMatrix[B.ID][C.ID]
                            # targetRtCostChange = self.distanceMatrix[F.ID][B.ID] + self.distanceMatrix[B.ID][G.ID] - self.distanceMatrix[F.ID][G.ID]
                        if originRouteIndex != targetRouteIndex:                            
                            moveCost = originRtCostChange + targetRtCostChange
                        else:
                            moveCost = originRtCostChange
                        
                        if (moveCost > rm.moveCost):
                            self.StoreBestRelocationMove(originRouteIndex, originNodeIndex, targetRouteIndex, 
                                    targetNodeIndex, originRtCostChange, targetRtCostChange, moveCost, rm)
                                
    def calculate_route_details(self, nodes_sequence):
        tot_dem = sum(n.demand for n in nodes_sequence)
        tot_load = self.empty_vehicle_weight + tot_dem
        tn_km = 0
        for i in range(len(nodes_sequence) - 1):
            from_node = nodes_sequence[i]
            to_node = nodes_sequence[i+1]
            tn_km += self.distanceMatrix[from_node.ID][to_node.ID] * tot_load
            tot_load -= to_node.demand
        return tn_km , tot_dem
    
    def calculate_relocation_cost(self, originRoute, targetRoute, originNodeIndex, targetNodeIndex):
        originRtCostChange = 0
        targetRtCostChange = 0
        originNode = originRoute.sequenceOfNodes[originNodeIndex]
        targetNode = targetRoute.sequenceOfNodes[targetNodeIndex]
        #OriginRtCostChange    
        #Formula first part
        sum_routes = 0
        for i in range(0, originNodeIndex):
            n1 = originRoute.sequenceOfNodes[i]
            n2 = originRoute.sequenceOfNodes[i+1]
            sum_routes += self.distanceMatrix[n1.ID][n2.ID]
        originRtCostChange += sum_routes * originNode.demand

        #if origin is not last
        if originNodeIndex != len(originRoute.sequenceOfNodes) - 1:
            A = originRoute.sequenceOfNodes[originNodeIndex - 1]
            B = originRoute.sequenceOfNodes[originNodeIndex]
            C = originRoute.sequenceOfNodes[originNodeIndex + 1]
            A_B = self.distanceMatrix[A.ID][B.ID]
            B_C = self.distanceMatrix[B.ID][C.ID]
            A_C = self.distanceMatrix[A.ID][C.ID]
            sum_demands = self.empty_vehicle_weight
            for i in range (originNodeIndex + 1, len(originRoute.sequenceOfNodes)):
                n1 = originRoute.sequenceOfNodes[i]
                sum_demands += n1.demand
            originRtCostChange += (A_B + B_C - A_C) * sum_demands

        #TargetRtCostChange
        sum_routes = 0
        for i in range (0, targetNodeIndex):
            n1 = targetRoute.sequenceOfNodes[i]
            n2 = targetRoute.sequenceOfNodes[i+1]
            sum_routes += self.distanceMatrix[n1.ID][n2.ID]
        sum_routes += self.distanceMatrix[targetNode.ID][originNode.ID]
        targetRtCostChange += -1*sum_routes*originNode.demand

        #if target is not last
        if targetNodeIndex != len(targetRoute.sequenceOfNodes) - 1:
            F = targetRoute.sequenceOfNodes[targetNodeIndex]
            B = originRoute.sequenceOfNodes[originNodeIndex]
            G = targetRoute.sequenceOfNodes[targetNodeIndex + 1]
            F_B = self.distanceMatrix[F.ID][B.ID]
            B_G = self.distanceMatrix[B.ID][G.ID]
            F_G = self.distanceMatrix[F.ID][G.ID]
            sum_demands = self.empty_vehicle_weight
            for i in range(targetNodeIndex + 1, len(targetRoute.sequenceOfNodes)):
                sum_demands += targetRoute.sequenceOfNodes[i].demand
            targetRtCostChange += (F_G - F_B - B_G)*sum_demands   
        return originRtCostChange, targetRtCostChange                                     

    def FindBestSwapMove(self, sm):
        for firstRouteIndex in range(0, len(self.sol.routes)):
            rt1:Route = self.sol.routes[firstRouteIndex]
            for secondRouteIndex in range (firstRouteIndex, len(self.sol.routes)):
                rt2:Route = self.sol.routes[secondRouteIndex]
                # for firstNodeIndex in range (1, len(rt1.sequenceOfNodes) - 1):
                for firstNodeIndex in range (1, len(rt1.sequenceOfNodes)):
                    startOfSecondNodeIndex = 1
                    if rt1 == rt2:
                        startOfSecondNodeIndex = firstNodeIndex + 1
                    # for secondNodeIndex in range (startOfSecondNodeIndex, len(rt2.sequenceOfNodes) - 1):
                    for secondNodeIndex in range (startOfSecondNodeIndex, len(rt2.sequenceOfNodes)):

                        a1 = rt1.sequenceOfNodes[firstNodeIndex - 1]
                        b1 = rt1.sequenceOfNodes[firstNodeIndex]
                        if firstNodeIndex + 1 < len(rt1.sequenceOfNodes):
                            c1 = rt1.sequenceOfNodes[firstNodeIndex + 1]
                        else:
                            continue
                        

                        a2 = rt2.sequenceOfNodes[secondNodeIndex - 1]
                        b2 = rt2.sequenceOfNodes[secondNodeIndex]
                        if secondNodeIndex + 1 < len(rt2.sequenceOfNodes):
                            c2 = rt2.sequenceOfNodes[secondNodeIndex + 1]
                        else:
                            continue
                        

                        moveCost = None
                        costChangeFirstRoute = None
                        costChangeSecondRoute = None

                        if rt1 == rt2:
                            oldcost = rt1.cost
                            rtcopy = rt1.copy()
                            first = rtcopy.sequenceOfNodes[firstNodeIndex]
                            second = rtcopy.sequenceOfNodes[secondNodeIndex]
                            rtcopy.insert(firstNodeIndex, first)
                            rtcopy.pop(firstNodeIndex + 1)
                            rtcopy.insert(secondNodeIndex, second)
                            rtcopy.pop(secondNodeIndex + 1)
                            rtcopy.cost, rtcopy.load = self.calculate_route_details(rtcopy.sequenceOfNodes)
                            newcost = rtcopy.cost
                            originRtCostChange = oldcost - newcost
                            targetRtCostChange = originRtCostChange
                        else:
                            if rt1.load - b1.demand + b2.demand > self.capacity:
                                continue
                            if rt2.load - b2.demand + b1.demand > self.capacity:
                                continue

                            originRtCostChange, targetRtCostChange = self.calculate_swap_cost(firstRouteIndex, secondRouteIndex, firstNodeIndex, secondNodeIndex)

                            

                        if moveCost < sm.moveCost:
                            self.StoreBestSwapMove(firstRouteIndex, secondRouteIndex, firstNodeIndex, secondNodeIndex,
                                                   moveCost, costChangeFirstRoute, costChangeSecondRoute, sm)

    def calculate_swap_cost(self, firstRouteIndex, secondRouteIndex, firstNodeIndex, secondNodeIndex):
        #Calculating move cost for origin route
        originRtCostChange = 0

    def ApplyRelocationMove(self, rm: RelocationMove):

        oldCost = self.CalculateTotalCost(self.sol)
        # print('Relocating node ' + str(rm.originNodePosition) + 
        #       ' from Route ' + str(rm.originRoutePosition) + ' at ' + 
        #       str(rm.targetNodePosition) + ' in route ' + str(rm.targetRoutePosition))
        originRt = self.sol.routes[rm.originRoutePosition]
        targetRt = self.sol.routes[rm.targetRoutePosition]
        B = originRt.sequenceOfNodes[rm.originNodePosition]

        if originRt == targetRt:
            # print('Same Route Relocation')
            del originRt.sequenceOfNodes[rm.originNodePosition]
            if (rm.originNodePosition < rm.targetNodePosition):
                targetRt.sequenceOfNodes.insert(rm.targetNodePosition, B)
            else:
                targetRt.sequenceOfNodes.insert(rm.targetNodePosition + 1, B)

            originRt.cost -= rm.moveCost
        else:
            #print('Different Route Relocation')
            del originRt.sequenceOfNodes[rm.originNodePosition]
            targetRt.sequenceOfNodes.insert(rm.targetNodePosition + 1, B)
            originRt.cost -= rm.costChangeOriginRt
            targetRt.cost -= rm.costChangeTargetRt
            originRt.load -= B.demand
            targetRt.load += B.demand

        self.sol.cost -= rm.moveCost

        newCost = self.CalculateTotalCost(self.sol)
        
        #debuggingOnly
        check = abs((newCost - oldCost)) - rm.moveCost
        
        if check > 0.1:
            print('Cost Issue')

    def ApplySwapMove(self, sm):
       oldCost = self.CalculateTotalCost(self.sol)
       rt1 = self.sol.routes[sm.positionOfFirstRoute]
       rt2 = self.sol.routes[sm.positionOfSecondRoute]
       b1 = rt1.sequenceOfNodes[sm.positionOfFirstNode]
       b2 = rt2.sequenceOfNodes[sm.positionOfSecondNode]
       rt1.sequenceOfNodes[sm.positionOfFirstNode] = b2
       rt2.sequenceOfNodes[sm.positionOfSecondNode] = b1

       if (rt1 == rt2):
           rt1.cost += sm.moveCost
       else:
           rt1.cost += sm.costChangeFirstRt
           rt2.cost += sm.costChangeSecondRt
           rt1.load = rt1.load - b1.demand + b2.demand
           rt2.load = rt2.load + b1.demand - b2.demand

       self.sol.cost += sm.moveCost

       newCost = self.CalculateTotalCost(self.sol)
       # debuggingOnly
       if abs((newCost - oldCost) - sm.moveCost) > 0.0001:
           print('Cost Issue')

    def ReportSolution(self, sol):
        for i in range(0, len(sol.routes)):
            rt = sol.routes[i]
            for j in range (0, len(rt.sequenceOfNodes)):
                print(rt.sequenceOfNodes[j].ID, end=' ')
            print(rt.cost)
        print (self.sol.cost)

    def StoreBestRelocationMove(self, originRouteIndex, originNodeIndex, targetRouteIndex, targetNodeIndex, originRtCostChange, targetRtCostChange, moveCost, rm:RelocationMove):
        rm.originRoutePosition = originRouteIndex
        rm.originNodePosition = originNodeIndex
        rm.targetRoutePosition = targetRouteIndex
        rm.targetNodePosition = targetNodeIndex
        rm.costChangeOriginRt = originRtCostChange
        rm.costChangeTargetRt = targetRtCostChange
        rm.moveCost = moveCost

    def StoreBestSwapMove(self, firstRouteIndex, secondRouteIndex, firstNodeIndex, secondNodeIndex, moveCost, costChangeFirstRoute, costChangeSecondRoute, sm):
        sm.positionOfFirstRoute = firstRouteIndex
        sm.positionOfSecondRoute = secondRouteIndex
        sm.positionOfFirstNode = firstNodeIndex
        sm.positionOfSecondNode = secondNodeIndex
        sm.costChangeFirstRt = costChangeFirstRoute
        sm.costChangeSecondRt = costChangeSecondRoute
        sm.moveCost = moveCost

    def CalculateTotalCost(self, sol):
        c = 0
        for i in range (0, len(sol.routes)):
            rt = sol.routes[i]
            tn_km = self.calculate_route_details(rt.sequenceOfNodes)[0]
            c += tn_km
        return c

    def InitializeOperators(self, rm, sm, top):
        rm.Initialize()
        sm.Initialize()
        top.Initialize()

    def FindBestTwoOptMove(self, top):
        for rtInd1 in range(0, len(self.sol.routes)):
            rt1:Route = self.sol.routes[rtInd1]
            for rtInd2 in range(rtInd1, len(self.sol.routes)):
                rt2:Route = self.sol.routes[rtInd2]
                for nodeInd1 in range(0, len(rt1.sequenceOfNodes)):
                    start2 = 0
                    if (rt1 == rt2):
                        start2 = nodeInd1 + 2
                        # για να μην κανει ανταλλαγή δύο διαδοοχικών 
                        # κόμβων που δεν θα έχει νόημα 
                    # for nodeInd2 in range(start2, len(rt2.sequenceOfNodes) - 1):
                    for nodeInd2 in range(start2, len(rt2.sequenceOfNodes)):
                        moveCost = 10 ** 9

                        # A = rt1.sequenceOfNodes[nodeInd1]
                        # B = rt1.sequenceOfNodes[nodeInd1 + 1]
                        # K = rt2.sequenceOfNodes[nodeInd2]
                        # L = rt2.sequenceOfNodes[nodeInd2 + 1]

                        if rt1 == rt2:
                            oldcost = rt1.cost
                            rtcopy = rt1.copy()
                            # reverses the nodes in the segment [positionOfFirstNode + 1,  top.positionOfSecondNode]
                            reversedSegment = reversed(rt1.sequenceOfNodes[nodeInd1 + 1: nodeInd2 + 1])
                            rtcopy.sequenceOfNodes[nodeInd1 + 1 : nodeInd2 + 1] = reversedSegment
                            newcost, load = self.calculate_route_details(rtcopy.sequenceOfNodes)
                            
                            moveCost = oldcost - newcost
                        else:
                            if nodeInd1 == 0 and nodeInd2 == 0:
                                continue
            
                            if nodeInd1 == len(rt1.sequenceOfNodes) - 1 and  nodeInd2 == len(rt2.sequenceOfNodes) - 1:
                                continue

                            if self.CapacityIsViolated(rt1, nodeInd1, rt2, nodeInd2):
                                continue
                            moveCost = self.calculate_two_opt_cost(rtInd1, rtInd2, nodeInd1, nodeInd2)
                        if moveCost > top.moveCost:
                            self.StoreBestTwoOptMove(rtInd1, rtInd2, nodeInd1, nodeInd2, moveCost, top)

    def CapacityIsViolated(self, rt1, nodeInd1, rt2, nodeInd2):

        rt1FirstSegmentLoad = 0
        for i in range(0, nodeInd1 + 1):
            n = rt1.sequenceOfNodes[i]
            rt1FirstSegmentLoad += n.demand
        rt1SecondSegmentLoad = rt1.load - rt1FirstSegmentLoad

        rt2FirstSegmentLoad = 0
        for i in range(0, nodeInd2 + 1):
            n = rt2.sequenceOfNodes[i]
            rt2FirstSegmentLoad += n.demand
        rt2SecondSegmentLoad = rt2.load - rt2FirstSegmentLoad

        if (rt1FirstSegmentLoad + rt2SecondSegmentLoad > rt1.capacity):
            return True
        if (rt2FirstSegmentLoad + rt1SecondSegmentLoad > rt2.capacity):
            return True

        return False

    def calculate_two_opt_cost(self, rtInd1, rtInd2, nodeInd1, nodeInd2):
        moveCost = 0
        rt1 = self.sol.routes[rtInd1]
        rt2 = self.sol.routes[rtInd2]
        sum_routes_1 = 0
        sum_routes_2 = 0
        sum_demands_1 = 0
        sum_demands_2 = 0
        for i in range(0, nodeInd1):
            n1 = rt1.sequenceOfNodes[i]
            n2 = rt1.sequenceOfNodes[i+1]
            sum_routes_1 += self.distanceMatrix[n1.ID][n2.ID]

        for i in range(nodeInd1 + 1, len(rt1.sequenceOfNodes)):
            n = rt1.sequenceOfNodes[i]
            sum_demands_1 += n.demand

        for i in range(0, nodeInd2):
            n1 = rt2.sequenceOfNodes[i]
            n2 = rt2.sequenceOfNodes[i+1]
            sum_routes_2 += self.distanceMatrix[n1.ID][n2.ID] 

        for i in range(nodeInd2 + 1, len(rt2.sequenceOfNodes)):
            n = rt2.sequenceOfNodes[i]
            sum_demands_2 += n.demand

        #could include if statement to make sure that these are not calculated if nodeIndx = 0
        moveCost += sum_routes_1 * (sum_demands_1 - sum_demands_2)
        moveCost += sum_routes_2 * (sum_demands_2 - sum_demands_1)

        if nodeInd1 != len(rt1.sequenceOfNodes) - 1:
            B = rt1.sequenceOfNodes[nodeInd1]
            C = rt1.sequenceOfNodes[nodeInd1 + 1]
            G = rt2.sequenceOfNodes[nodeInd2]
            old_route_1 = self.distanceMatrix[B.ID][C.ID]
            new_route_1 = self.distanceMatrix[G.ID][C.ID]
            w = self.empty_vehicle_weight
            moveCost += (old_route_1 - new_route_1)*(sum_demands_1 + w)

        if nodeInd2 != len(rt2.sequenceOfNodes) - 1:
            B = rt1.sequenceOfNodes[nodeInd1]
            G = rt2.sequenceOfNodes[nodeInd2]
            H = rt2.sequenceOfNodes[nodeInd2 + 1]
            old_route_2 = self.distanceMatrix[G.ID][H.ID]
            new_route_2 = self.distanceMatrix[B.ID][H.ID]
            w = self.empty_vehicle_weight
            moveCost += (old_route_2 - new_route_2)*(sum_demands_2 + w)

        return moveCost

    def StoreBestTwoOptMove(self, rtInd1, rtInd2, nodeInd1, nodeInd2, moveCost, top):
        top.positionOfFirstRoute = rtInd1
        top.positionOfSecondRoute = rtInd2
        top.positionOfFirstNode = nodeInd1
        top.positionOfSecondNode = nodeInd2
        top.moveCost = moveCost 

    def ApplyTwoOptMove(self, top):
        rt1:Route = self.sol.routes[top.positionOfFirstRoute]
        rt2:Route = self.sol.routes[top.positionOfSecondRoute]

        if rt1 == rt2:
            # reverses the nodes in the segment [positionOfFirstNode + 1,  top.positionOfSecondNode]
            reversedSegment = reversed(rt1.sequenceOfNodes[top.positionOfFirstNode + 1: top.positionOfSecondNode + 1])
            #lst = list(reversedSegment)
            #lst2 = list(reversedSegment)
            rt1.sequenceOfNodes[top.positionOfFirstNode + 1 : top.positionOfSecondNode + 1] = reversedSegment

            #reversedSegmentList = list(reversed(rt1.sequenceOfNodes[top.positionOfFirstNode + 1: top.positionOfSecondNode + 1]))
            #rt1.sequenceOfNodes[top.positionOfFirstNode + 1: top.positionOfSecondNode + 1] = reversedSegmentList

            rt1.cost -= top.moveCost

        else:
            #slice with the nodes from position top.positionOfFirstNode + 1 onwards
            relocatedSegmentOfRt1 = rt1.sequenceOfNodes[top.positionOfFirstNode + 1 :]

            #slice with the nodes from position top.positionOfFirstNode + 1 onwards
            relocatedSegmentOfRt2 = rt2.sequenceOfNodes[top.positionOfSecondNode + 1 :]

            del rt1.sequenceOfNodes[top.positionOfFirstNode + 1 :]
            del rt2.sequenceOfNodes[top.positionOfSecondNode + 1 :]

            rt1.sequenceOfNodes.extend(relocatedSegmentOfRt2)
            rt2.sequenceOfNodes.extend(relocatedSegmentOfRt1)

            # self.UpdateRouteCostAndLoad(rt1)
            # self.UpdateRouteCostAndLoad(rt2)
            rt1.cost, rt1.load = self.calculate_route_details(rt1.sequenceOfNodes)
            rt2.cost, rt2.load = self.calculate_route_details(rt2.sequenceOfNodes)

        self.sol.cost -= top.moveCost

    def UpdateRouteCostAndLoad(self, rt: Route):
        tc = 0
        tl = 0
        for i in range(0, len(rt.sequenceOfNodes) - 1):
            A = rt.sequenceOfNodes[i]
            B = rt.sequenceOfNodes[i+1]
            tc += self.distanceMatrix[A.ID][B.ID]
            tl += A.demand
        rt.load = tl
        rt.cost = tc

    def TestSolution(self):
        totalSolCost = 0
        for r in range (0, len(self.sol.routes)):
            rt: Route = self.sol.routes[r]
            rtCost = 0
            rtLoad = 0
            rtCost , rtLoad = self.calculate_route_details(rt.sequenceOfNodes)
            if abs(rtCost - rt.cost) > 0.0001:
                print ('Route Cost problem for route ' + str(r))
            if abs(rtLoad - rt.load) > 0.0001:
                print ('Route Load problem')

            totalSolCost += rt.cost

        if abs(totalSolCost - self.sol.cost) > 0.0001:
            print('Solution Cost problem')

    def transport_solution_to_txt(self):
        file = open('emm_emm_solution.txt', 'w')
        file.write('Cost:\n')
        file.write(str(self.sol.cost) + '\n')
        file.write('Routes:\n')
        file.write(str(len(self.sol.routes)) + '\n')
        for rt in self.sol.routes:
            file.write(str(rt.sequenceOfNodes[0].ID))
            for i in range(1, len(rt.sequenceOfNodes)):
                file.write(',' + str(rt.sequenceOfNodes[i].ID))
            file.write('\n')
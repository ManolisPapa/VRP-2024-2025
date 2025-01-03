from Model_Emm_Emm import *
from SolutionDrawer import *
class Solution:
    def __init__(self):
        self.cost = 0.0
        self.routes = []

class Saving:
    def __init__(self, n1, n2, score):
        self.n1 = n1
        self.n2 = n2
        self.score = score

# Addition of classes for Local Research
class RelocationMove(object):
    def __init__(self):
        # self.originRoutePosition = None
        # self.targetRoutePosition = None
        # self.originNodePosition = None
        # self.targetNodePosition = None
        # self.costChangeOriginRt = None
        # self.costChangeTargetRt = None
        self.moveCost = None
        self.oldRoutes = []
        self.newRoutes = []

    def Initialize(self):
        # self.originRoutePosition = None
        # self.targetRoutePosition = None
        # self.originNodePosition = None
        # self.targetNodePosition = None
        # self.costChangeOriginRt = None
        # self.costChangeTargetRt = None
        
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
        self.moveCost = 10 ** 9



class Emm_Emm_Solver:
    def __init__(self, m):
        self.allNodes = m.allNodes
        self.customers = m.customers
        self.depot = m.allNodes[0]
        self.distanceMatrix = m.distanceMatrix
        self.capacity = m.capacity
        self.empty_vehicle_weight = m.empty_vehicle_weight
        self.sol = None
        self.bestSolution = None

    def solve(self):
        self.sol = self.create_initial_routes()
        savings = self.calculate_savings()
        savings.sort(key=lambda s: s.score, reverse=True)

        for sav in savings:
            n1 = sav.n1
            n2 = sav.n2
            rt1 = n1.route
            rt2 = n2.route
            old_cost = rt1.cost + rt2.cost

            if not self.nodes_are_compatible(n1, n2):
                continue
            
            pot_route = self.create_potential_route(n1, n2)
            pot_route.cost, pot_route.load = self.calculate_route_details(pot_route.sequenceOfNodes)
            new_cost = pot_route.cost

            if old_cost > new_cost:
                self.update_routes(rt1, rt2, pot_route)
                pot_route_ID = []
                for node in pot_route.sequenceOfNodes :
                    pot_route_ID.append(node.ID)
                #print('Created route ', pot_route_ID, 'with saving', old_cost - new_cost)
        
        self.ReportSolution(self.sol)
        self.LocalSearch(0)  # Βελτιστοποίηση της αρχικής λύσης
        self.ReportSolution(self.sol)
        self.transport_solution_to_txt()
    
        return self.sol

    def calculate_savings(self):    
        savings = []
        for n1 in self.customers:
            for n2 in self.customers:
                if n1 != n2:
                    rt1 = n1.route
                    rt2 = n2.route
                    old_cost = rt1.cost + rt2.cost

                    pot_route = self.create_potential_route(n1, n2)

                    pot_route.cost, pot_route.load = self.calculate_route_details(pot_route.sequenceOfNodes)
                    new_cost = pot_route.cost
                    if new_cost < old_cost:
                        score = old_cost - new_cost
                        sav = Saving(n1, n2, score)
                        savings.append(sav)
        return savings

    def create_initial_routes(self):
        s = Solution()
        for i in range(0, len(self.customers)):
            n = self.customers[i]
            rt = Route(self.depot, self.capacity)
            n.route = rt
            n.position_in_route = 1
            rt.sequenceOfNodes.insert(1, n)
            rt.cost, rt.load = self.calculate_route_details(rt.sequenceOfNodes)
            s.routes.append(rt)
            s.cost += rt.cost
        return s
    
    def calculate_route_details(self, nodes_sequence):
        tot_dem = sum(n.demand for n in nodes_sequence)
        tot_load = self.empty_vehicle_weight + tot_dem
        tn_km = 0
        for i in range(len(nodes_sequence) - 1):
            from_node = nodes_sequence[i]
            to_node = nodes_sequence[i+1]
            tn_km += self.distanceMatrix[from_node.ID][to_node.ID] * tot_load
            tot_load -= to_node.demand
        return tn_km, tot_dem
    
    def nodes_are_compatible(self, n1, n2):
        if n1.route == n2.route:
           return False
        if n1.position_in_route != 1 and n1.position_in_route != len(n1.route.sequenceOfNodes) - 1:
           return False
        if n2.position_in_route != 1 and n2.position_in_route != len(n2.route.sequenceOfNodes) - 1:
           return False
        if n1.route.load + n2.route.load > self.capacity:
            return False
        return True
    
    def create_potential_route(self, n1, n2):
        pos1 = n1.position_in_route
        pos2 = n2.position_in_route
        rt1 = n1.route.copy()
        rt2 = n2.route.copy()
        pot_route = n1.route.copy()
        if pos1 == 1 and pos2 == 1:
            pot_route.sequenceOfNodes[1:1] = rt2.sequenceOfNodes[len(rt2.sequenceOfNodes) - 1 : 0 : -1]
            return pot_route
        if pos1 == 1 and pos2 == len(rt2.sequenceOfNodes) - 1:
            pot_route.sequenceOfNodes[1:1] = rt2.sequenceOfNodes[1:]
            return pot_route
        if pos1 == len(rt1.sequenceOfNodes) - 1 and pos2 == 1:
            pot_route.sequenceOfNodes[len(pot_route.sequenceOfNodes) - 1:len(pot_route.sequenceOfNodes) - 1] = rt2.sequenceOfNodes[1:]
            return pot_route
        if pos1 == len(rt1.sequenceOfNodes) - 1 and pos2 == len(rt2.sequenceOfNodes) - 1:
            pot_route.sequenceOfNodes[len(pot_route.sequenceOfNodes) - 1:len(pot_route.sequenceOfNodes) - 1] = rt2.sequenceOfNodes[len(rt2.sequenceOfNodes) - 1 : 0 : -1]
            return pot_route
        
    def update_routes(self, rt1, rt2, pot_route):
        self.sol.routes.remove(rt1)
        self.sol.routes.remove(rt2)
        self.sol.routes.append(pot_route)
        self.sol.cost -= (rt1.cost + rt2.cost)
        self.sol.cost += pot_route.cost
        for i in range (1, len(pot_route.sequenceOfNodes)):
            n = pot_route.sequenceOfNodes[i]
            n.route = pot_route
            n.position_in_route = i

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

# Local Search 


    def LocalSearch(self, operator):
        self.bestSolution = self.cloneSolution(self.sol)
        terminationCondition = False
        localSearchIterator = 0

        rm = RelocationMove()
        sm = SwapMove()
        top = TwoOptMove()

        while terminationCondition is False:

            self.InitializeOperators(rm, sm, top)
            SolDrawer.draw(localSearchIterator, self.sol, self.allNodes)

            # Relocations
            if operator == 0:
                self.FindBestRelocationMove(rm)
                if len(rm.oldRoutes) > 0:
                    if rm.moveCost > 0:
                        self.ApplyRelocationMove(rm)
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
                    if top.moveCost < 0:
                        self.ApplyTwoOptMove(top)
                    else:
                        terminationCondition = True

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
                            rtcopy.sequenceOfNodes.insert(targetNodeIndex + 1, bcopy)
                            rtcopy.cost, rtcopy.load = self.calculate_route_details(rtcopy.sequenceOfNodes)
                            newcost = rtcopy.cost
                            # originRtCostChange = oldcost - newcost
                            # targetRtCostChange = originRtCostChange 
                        else:   
                            rt1oldcost = rt1.cost
                            rt2oldcost = rt2.cost
                            oldcost = rt1oldcost + rt2oldcost
                            rt1copy = rt1.copy()
                            rt2copy = rt2.copy()
                            bcopy = rt1copy.sequenceOfNodes.pop(originNodeIndex) 
                            rt2copy.sequenceOfNodes.insert(targetNodeIndex + 1, bcopy)
                            rt1copy.cost, rt1copy.load = self.calculate_route_details(rt1copy.sequenceOfNodes)
                            rt1newcost = rt1copy.cost
                            rt2copy.cost, rt2copy.load = self.calculate_route_details(rt2copy.sequenceOfNodes)
                            rt2newcost = rt2copy.cost
                            newcost = rt1newcost + rt2newcost
                            originRtCostChange = rt1oldcost - rt1newcost
                            targetRtCostChange = rt2oldcost - rt2newcost
                            # originRtCostChange = self.distanceMatrix[A.ID][C.ID] - self.distanceMatrix[A.ID][B.ID] - self.distanceMatrix[B.ID][C.ID]
                            # targetRtCostChange = self.distanceMatrix[F.ID][B.ID] + self.distanceMatrix[B.ID][G.ID] - self.distanceMatrix[F.ID][G.ID]

                        moveCost = oldcost - newcost
                        
                        if (moveCost > rm.moveCost):
                            oldlist= []
                            newlist =[]
                            if originRouteIndex == targetRouteIndex :
                                oldlist.append(rt1)
                                newlist.append(rtcopy)
                            else:
                                oldlist.append(rt1)
                                oldlist.append(rt2)
                                newlist.append(rt1copy)
                                newlist.append(rt2copy)
                                
                            self.StoreBestRelocationMove(oldlist, newlist, moveCost, rm)
                                
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
                            if firstNodeIndex == secondNodeIndex - 1:
                                # case of consecutive nodes swap
                                costRemoved = self.distanceMatrix[a1.ID][b1.ID] + self.distanceMatrix[b1.ID][b2.ID] + \
                                              self.distanceMatrix[b2.ID][c2.ID]
                                costAdded = self.distanceMatrix[a1.ID][b2.ID] + self.distanceMatrix[b2.ID][b1.ID] + \
                                            self.distanceMatrix[b1.ID][c2.ID]
                                moveCost = costAdded - costRemoved
                            else:

                                costRemoved1 = self.distanceMatrix[a1.ID][b1.ID] + self.distanceMatrix[b1.ID][c1.ID]
                                costAdded1 = self.distanceMatrix[a1.ID][b2.ID] + self.distanceMatrix[b2.ID][c1.ID]
                                costRemoved2 = self.distanceMatrix[a2.ID][b2.ID] + self.distanceMatrix[b2.ID][c2.ID]
                                costAdded2 = self.distanceMatrix[a2.ID][b1.ID] + self.distanceMatrix[b1.ID][c2.ID]
                                moveCost = costAdded1 + costAdded2 - (costRemoved1 + costRemoved2)
                        else:
                            if rt1.load - b1.demand + b2.demand > self.capacity:
                                continue
                            if rt2.load - b2.demand + b1.demand > self.capacity:
                                continue

                            costRemoved1 = self.distanceMatrix[a1.ID][b1.ID] + self.distanceMatrix[b1.ID][c1.ID]
                            costAdded1 = self.distanceMatrix[a1.ID][b2.ID] + self.distanceMatrix[b2.ID][c1.ID]
                            costRemoved2 = self.distanceMatrix[a2.ID][b2.ID] + self.distanceMatrix[b2.ID][c2.ID]
                            costAdded2 = self.distanceMatrix[a2.ID][b1.ID] + self.distanceMatrix[b1.ID][c2.ID]

                            costChangeFirstRoute = costAdded1 - costRemoved1
                            costChangeSecondRoute = costAdded2 - costRemoved2

                            moveCost = costAdded1 + costAdded2 - (costRemoved1 + costRemoved2)

                        if moveCost < sm.moveCost:
                            self.StoreBestSwapMove(firstRouteIndex, secondRouteIndex, firstNodeIndex, secondNodeIndex,
                                                   moveCost, costChangeFirstRoute, costChangeSecondRoute, sm)

    def ApplyRelocationMove(self, rm: RelocationMove):

        oldCost = self.sol.cost


        if len(rm.oldRoutes) == 1:
            self.sol.routes.remove(rm.oldRoutes[0])
            self.sol.routes.append(rm.newRoutes[0])
        else:
            self.sol.routes.remove(rm.oldRoutes[0])
            self.sol.routes.remove(rm.oldRoutes[1])  
            self.sol.routes.append(rm.newRoutes[0])
            self.sol.routes.append(rm.newRoutes[1])
        

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

    def GetLastOpenRoute(self):
        if len(self.sol.routes) == 0:
            return None
        else:
            return self.sol.routes[-1]

    def IdentifyBestInsertion(self, bestInsertion, rt):
        for i in range(0, len(self.customers)):
            candidateCust:Node = self.customers[i]
            if candidateCust.isRouted is False:
                if rt.load + candidateCust.demand <= rt.capacity:
                    lastNodePresentInTheRoute = rt.sequenceOfNodes[-2]
                    # lastNodePresentInTheRoute = rt.sequenceOfNodes[-1]
                    trialCost = self.distanceMatrix[lastNodePresentInTheRoute.ID][candidateCust.ID]
                    if trialCost < bestInsertion.cost:
                        bestInsertion.customer = candidateCust
                        bestInsertion.route = rt
                        bestInsertion.cost = trialCost

    def ApplyCustomerInsertion(self, insertion):
        insCustomer = insertion.customer
        rt = insertion.route
        #before the second depot occurrence
        insIndex = len(rt.sequenceOfNodes) - 1
        # now insIndex = len(rt.sequenceOfNodes) 
        rt.sequenceOfNodes.insert(insIndex, insCustomer)

        beforeInserted = rt.sequenceOfNodes[-3]
        # now beforeInserted = rt.sequenceOfNodes[-2]

        costAdded = self.distanceMatrix[beforeInserted.ID][insCustomer.ID] + self.distanceMatrix[insCustomer.ID][self.depot.ID]
        # now costAdded = self.distanceMatrix[beforeInserted.ID][insCustomer.ID] 
        costRemoved = self.distanceMatrix[beforeInserted.ID][self.depot.ID]
        #  now maybe we dont need that

        rt.cost += costAdded - costRemoved
        #  now maybe we dont need that
        self.sol.cost += costAdded - costRemoved
        #  now maybe we dont need that
        rt.load += insCustomer.demand

        insCustomer.isRouted = True

    def StoreBestRelocationMove(self, oldlist, newlist, moveCost, rm:RelocationMove):
        # rm.originRoutePosition = originRouteIndex
        # rm.originNodePosition = originNodeIndex
        # rm.targetRoutePosition = targetRouteIndex
        # rm.targetNodePosition = targetNodeIndex
        # rm.costChangeOriginRt = originRtCostChange
        # rm.costChangeTargetRt = targetRtCostChange
        rm.oldRoutes = oldlist
        rm.newRoutes = newlist
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

                        A = rt1.sequenceOfNodes[nodeInd1]
                        B = rt1.sequenceOfNodes[nodeInd1 + 1]
                        K = rt2.sequenceOfNodes[nodeInd2]
                        L = rt2.sequenceOfNodes[nodeInd2 + 1]

                        if rt1 == rt2:
                            # if nodeInd1 == 0 and nodeInd2 == len(rt1.sequenceOfNodes) - 2:
                            if nodeInd1 == 0 and nodeInd2 == len(rt1.sequenceOfNodes) - 1:
                                continue
                            costAdded = self.distanceMatrix[A.ID][K.ID] + self.distanceMatrix[B.ID][L.ID]
                            costRemoved = self.distanceMatrix[A.ID][B.ID] + self.distanceMatrix[K.ID][L.ID]
                            moveCost = costAdded - costRemoved
                        else:
                            if nodeInd1 == 0 and nodeInd2 == 0:
                                continue
                            # if nodeInd1 == len(rt1.sequenceOfNodes) - 2 and  nodeInd2 == len(rt2.sequenceOfNodes) - 2:
                            if nodeInd1 == len(rt1.sequenceOfNodes) - 1 and  nodeInd2 == len(rt2.sequenceOfNodes) - 1:
                                continue

                            if self.CapacityIsViolated(rt1, nodeInd1, rt2, nodeInd2):
                                continue
                            costAdded = self.distanceMatrix[A.ID][L.ID] + self.distanceMatrix[B.ID][K.ID]
                            costRemoved = self.distanceMatrix[A.ID][B.ID] + self.distanceMatrix[K.ID][L.ID]
                            moveCost = costAdded - costRemoved
                        if moveCost < top.moveCost:
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

            rt1.cost += top.moveCost

        else:
            #slice with the nodes from position top.positionOfFirstNode + 1 onwards
            relocatedSegmentOfRt1 = rt1.sequenceOfNodes[top.positionOfFirstNode + 1 :]

            #slice with the nodes from position top.positionOfFirstNode + 1 onwards
            relocatedSegmentOfRt2 = rt2.sequenceOfNodes[top.positionOfSecondNode + 1 :]

            del rt1.sequenceOfNodes[top.positionOfFirstNode + 1 :]
            del rt2.sequenceOfNodes[top.positionOfSecondNode + 1 :]

            rt1.sequenceOfNodes.extend(relocatedSegmentOfRt2)
            rt2.sequenceOfNodes.extend(relocatedSegmentOfRt1)

            self.UpdateRouteCostAndLoad(rt1)
            self.UpdateRouteCostAndLoad(rt2)

        self.sol.cost += top.moveCost

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
                print ('Route Cost problem')
            if rtLoad != rt.load:
                print ('Route Load problem')

            totalSolCost += rt.cost

        if abs(totalSolCost - self.sol.cost) > 0.0001:
            print('Solution Cost problem')

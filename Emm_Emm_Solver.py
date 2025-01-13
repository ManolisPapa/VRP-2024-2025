from Model_Emm_Emm import *
#from SolutionDrawer import *
class Solution:
    def __init__(self):
        self.cost = 0.0
        self.routes = []

class Saving:
    def __init__(self, n1, n2, score):
        self.n1 = n1
        self.n2 = n2
        self.score = score

class Emm_Emm_Solver:
    def __init__(self, m):
        self.allNodes = m.allNodes
        self.customers = m.customers
        self.depot = m.allNodes[0]
        self.distanceMatrix = m.dist_matrix
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

    def ReportSolution(self, sol):
        for i in range(0, len(sol.routes)):
            rt = sol.routes[i]
            for j in range (0, len(rt.sequenceOfNodes)):
                print(rt.sequenceOfNodes[j].ID, end=' ')
            print(rt.cost)
        print (self.sol.cost)

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

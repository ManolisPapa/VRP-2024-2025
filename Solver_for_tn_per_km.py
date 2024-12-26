from VRP_Model import *
from SolutionDrawer import *
from sol_checker import *

class Solution:
    def __init__(self):
        self.cost = 0.0
        self.routes = []

class Saving:
    def __init__(self, n1, n2, sav):
        self.n1 = n1
        self.n2 = n2
        self.score = sav


class Solver:
    def __init__(self, m):
        self.allNodes = m.allNodes
        self.customers = m.customers
        self.depot = m.allNodes[0]
        self.distanceMatrix = m.dist_matrix
        self.tn_per_km_matrix = m.tn_per_km_matrix
        self.capacity = m.capacity
        self.empty_vehichle_weight = m.empty_vehichle_weight
        self.sol = None
        self.bestSolution = None

    def worstSolution(self):
        self.sol = Solution()
        for node in self.customers:
            rt = Route(self.depot, self.capacity)
            rt.sequenceOfNodes.append(node)
            self.sol.routes.append(rt)
        self.calculate_cost()
        return self.sol
    
    
    def solve(self):
        self.sol = Solution()
        available_nodes = self.customers[:]
        available_nodes.sort(key= lambda n: n.demand)

        while (len(available_nodes) != 0): 
            route = Route(self.depot, self.capacity)
            next_node = self.calculate_best_node(available_nodes, route)
            while (next_node != None):
                available_nodes.remove(next_node) #isRouted implementation
                route.sequenceOfNodes.append(next_node)
                route.load += next_node.demand
                next_node = self.calculate_best_node(available_nodes, route)
            self.sol.routes.append(route)
        
        self.calculate_cost()
        return self.sol

    def calculate_best_node(self, av_nodes, rt):
        current_node = rt.sequenceOfNodes[-1]
        load = rt.load
        index = 0
        if (len(av_nodes) == 0):
            return None
        next_node = av_nodes[index]
        best_node = None
        max = -1
        while (next_node.demand + load <= self.capacity and index < len(av_nodes)):
            next_node = av_nodes[index]
            if (self.tn_per_km_matrix[current_node.ID][next_node.ID] >= max):
                max = self.tn_per_km_matrix[current_node.ID][next_node.ID]
                best_node = next_node
            index += 1
            
        return best_node

    def calculate_cost(self):
        for rt in self.sol.routes:
            rt.cost, rt.load = calculate_route_details(rt.sequenceOfNodes, self.empty_vehichle_weight)
            self.sol.cost += rt.cost
        

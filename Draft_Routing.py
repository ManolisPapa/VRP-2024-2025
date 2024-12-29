from VRP_Model import *

class Solution:
    def __init__(self):
        self.cost = 0.0
        self.routes = []

class Draft_Routing_Solver:
    def __init__(self, m):
        self.allNodes = m.allNodes
        self.customers = m.customers
        self.depot = m.allNodes[0]
        self.distanceMatrix = m.dist_matrix
        self.tn_per_km_matrix = m.tn_per_km_matrix
        self.capacity = m.capacity
        self.empty_vehicle_weight = m.empty_vehicle_weight
        self.sol = None
        self.bestSolution = None

    def solve(self):
        self.sol = Solution()
        tn_per_km_from_start = []
        for i in range(len(self.customers)):
            tn_per_km_from_start.append((self.customers[i], self.tn_per_km_matrix[0][i + 1]))
        tn_per_km_from_start.sort(key=lambda x: x[1], reverse = True)
        no_of_routes = 30
        self.sol.routes = self.create_initial_routes(tn_per_km_from_start, no_of_routes)
        
        available_nodes = []
        for i in range(no_of_routes, len(tn_per_km_from_start)):
            available_nodes.append(tn_per_km_from_start[i][0])
        available_nodes.sort(key= lambda n: n.demand)

        while (len(available_nodes) != 0):  
            for i in range(len(self.sol.routes)):
                route = self.sol.routes[i]
                next_node = self.calculate_best_node(available_nodes, route)
                if (next_node != None):
                    available_nodes.remove(next_node) #isRouted implementation
                    route.sequenceOfNodes.append(next_node)
                    route.load += next_node.demand
        
        self.calculate_cost()
        self.transport_solution_to_txt()
        return self.sol
    
    def create_initial_routes(self, tn_per_km_from_start, no_of_routes):
        routes = []
        for i in range(no_of_routes):
            route = Route(self.depot, self.capacity)
            route.sequenceOfNodes.pop(-1)
            route.sequenceOfNodes.append(tn_per_km_from_start[i][0])
            route.cost, route.load = self.calculate_route_details(route.sequenceOfNodes)
            routes.append(route)
        return routes

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
        if (best_node != None):
            if (best_node.demand + load > self.capacity):    
                best_node = None
        return best_node
    
    def calculate_cost(self):
        for rt in self.sol.routes:
            rt.cost, rt.load = self.calculate_route_details(rt.sequenceOfNodes)
            self.sol.cost += rt.cost

    def transport_solution_to_txt(self):
        file = open('draft_routing_solution.txt', 'w')
        file.write('Cost:\n')
        file.write(str(self.sol.cost) + '\n')
        file.write('Routes:\n')
        file.write(str(len(self.sol.routes)) + '\n')
        for rt in self.sol.routes:
            file.write(str(rt.sequenceOfNodes[0].ID))
            for i in range(1, len(rt.sequenceOfNodes)):
                file.write(',' + str(rt.sequenceOfNodes[i].ID))
            file.write('\n') 
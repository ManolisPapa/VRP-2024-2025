from Solver import *
from sol_checker import *
from Solver_for_tn_per_km import *

class Solver_Clarke_Us:
    def __init__(self, m):
        default_solver = Solver(m)
        self.sol = default_solver.solve()
        self.empty_vehicle_weight = m.empty_vehicle_weight

    def solve(self):
        self.sol.cost = 0

        for route in self.sol.routes :
            route.sequenceOfNodes.pop(-1)
            cost, load = calculate_route_details(route.sequenceOfNodes, self.empty_vehicle_weight)
            self.sol.cost += cost
        self.transport_solution_to_txt()
        return self.sol
    
    def transport_solution_to_txt(self):
        file = open('clarke_wright_solution.txt', 'w')
        file.write('Cost:\n')
        file.write(str(self.sol.cost) + '\n')
        file.write('Routes:\n')
        file.write(str(len(self.sol.routes)) + '\n')
        for rt in self.sol.routes:
            file.write(str(rt.sequenceOfNodes[0].ID))
            for i in range(1, len(rt.sequenceOfNodes)):
                file.write(',' + str(rt.sequenceOfNodes[i].ID))
            file.write('\n')
        


    
    

print(1)
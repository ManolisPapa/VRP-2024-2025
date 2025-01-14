from VRP_Model import *
from Model_Emm_Emm import *
from Emm_Emm_Solver import *
from SolutionDrawer import *
from sol_checker import *
from Solver_for_tn_per_km import *
from Solver_Clarke_Us import *
from Draft_Routing import *
from LocalSearch import *
import time
results = []

# #Draft Routing implementation
# timestart = time.time()
# m = Model()
# m.BuildModel('Instance.txt')
# localSolver = LocalSolver(m)

# solver = Draft_Routing_Solver(m)
# Draft_Routing_Solution = solver.solve()

# localSolver = LocalSolver(m)
# new_solution = localSolver.LocalSolve(Draft_Routing_Solution)
# solver.sol = new_solution
# solver.transport_solution_to_txt()
# SolDrawer.draw('Draft_Routing', Draft_Routing_Solution, m.allNodes)
# print('Testing Draft Routing solution')
# all_nodes, capacity, empty_vehicle_weight = load_model('Instance.txt')
# test_solution('draft_routing_solution.txt', all_nodes, capacity, empty_vehicle_weight)
# timeend = time.time()
# results.append(('Draft Routing', solver.sol.cost, (timeend - timestart)/60))
# print(1)


# Emm & Emm implementation
timestart = time.time()
m = Model_Emm_Emm()
m.BuildModel('Instance.txt')

solver = Emm_Emm_Solver(m)
Emm_Emm_solution = solver.solve()

localSolver = LocalSolver(m)
new_solution = localSolver.LocalSolve(Emm_Emm_solution)
solver.sol = new_solution
solver.transport_solution_to_txt()
print('Testing Emm & Emm Solution')
test_solution('emm_emm_solution.txt', m.allNodes, m.capacity, m.empty_vehicle_weight)
SolDrawer.draw('Emm&Emm with Savings Logic', Emm_Emm_solution, m.allNodes) 
timeend = time.time()
print((timeend - timestart)/60)
results.append(('Draft Routing', solver.sol.cost, (timeend - timestart)/60))
print(1)

#Clarke & Wright implementation
timestart = time.time()
m = Model()
m.BuildModel('Instance.txt')

solver = Solver_Clarke_Us(m)
Clarke_Wright_solution = solver.solve()

localSolver = LocalSolver(m)
new_solution = localSolver.LocalSolve(Clarke_Wright_solution)
solver.sol = new_solution
solver.transport_solution_to_txt()

print('Testing Clarke & Wright solution')
test_solution('clarke_wright_solution.txt', m.allNodes, m.capacity, m.empty_vehicle_weight)
SolDrawer.draw('Clarke&Wright', Clarke_Wright_solution, m.allNodes)
timeend = time.time()
print((timeend - timestart)/60)
results.append(('Draft Routing', solver.sol.cost, (timeend - timestart)/60))
print(1)

#Tn per Km implementation
timestart = time.time()
m = Model()
m.BuildModel('Instance.txt')

solver_tn_per_km = Solver_TN_PER_KM(m)

Tn_Per_Km_solution = solver_tn_per_km.solve()
localSolver = LocalSolver(m)
new_solution = localSolver.LocalSolve(Tn_Per_Km_solution)
solver_tn_per_km.sol = new_solution
solver_tn_per_km.transport_solution_to_txt()

print('Testing Tn per Km solution')
test_solution('tn_per_km_solution.txt', m.allNodes, m.capacity, m.empty_vehicle_weight)
SolDrawer.draw('TN_PER_KM', Tn_Per_Km_solution, m.allNodes)
timeend = time.time()
print((timeend - timestart)/60)
results.append(('Draft Routing', solver.sol.cost, (timeend - timestart)/60))
print(1)

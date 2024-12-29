from VRP_Model import *
from Model_Emm_Emm import *
from Emm_Emm_Solver import *
from SolutionDrawer import *
from sol_checker import *
from Solver_for_tn_per_km import *
from Solver_Clarke_Us import *
from Draft_Routing import *

#Draft Routing implementation
m = Model()
m.BuildModel('Instance.txt')

solver = Draft_Routing_Solver(m)
solution = solver.solve()
SolDrawer.draw('Draft_Routing', solution, m.allNodes)
all_nodes, capacity, empty_vehicle_weight = load_model('Instance.txt')
test_solution('draft_routing_solution.txt', all_nodes, capacity, empty_vehicle_weight)
print(1)


# Emm & Emm implementation
m = Model_Emm_Emm()
m.BuildModel('Instance.txt')

solver = Emm_Emm_Solver(m)
Emm_Emm_solution = solver.solve()
print('Testing Emm & Emm Solution')
test_solution('emm_emm_solution.txt', all_nodes, capacity, empty_vehicle_weight)
SolDrawer.draw('Emm&Emm with Savings Logic', Emm_Emm_solution, m.allNodes)

#Clarke & Wright implementation
m = Model()
m.BuildModel('Instance.txt')

solver = Solver_Clarke_Us(m)
Clarke_Wright_solution = solver.solve()
print('Testing Clarke & Wright solution')
test_solution('clarke_wright_solution.txt', all_nodes, capacity, empty_vehicle_weight)
SolDrawer.draw('Clarke&Wright', Clarke_Wright_solution, m.allNodes)

#Tn per Km implementation
solver_tn_per_km = Solver_TN_PER_KM(m)

Tn_Per_Km_solution = solver_tn_per_km.solve()
print('Testing Tn per Km solution')
test_solution('tn_per_km_solution.txt', all_nodes, capacity, empty_vehicle_weight)
SolDrawer.draw('TN_PER_KM', Tn_Per_Km_solution, m.allNodes)


print(1)

from search.algorithms import State
from search.map import Map
import getopt
import sys
import heapq

def dijkstra(start, goal, gridded_map):
    # open list is heap
    open_list = []
    # closed list is hashmap or dict in python
    closed_list = dict()
    heapq.heappush(open_list, start)
    closed_list[start.state_hash()] = start
    while open_list:
        n = heapq.heappop(open_list)
        if n == goal:
            return n.get_g(), closed_list
        for n_prime in gridded_map.successors(n):
            if n_prime.state_hash() not in closed_list:
                heapq.heappush(open_list, n_prime)
                closed_list[n_prime.state_hash()] = n_prime
            if n_prime.state_hash() in closed_list and n_prime.get_g() < closed_list[n_prime.state_hash()].get_g():
                heapq.heappush(open_list, n_prime)
    return -1, closed_list

def bi_bs(start, goal, gridded_map):
    open_forward = []
    open_back = []
    closed_forward = dict()
    closed_back = dict()
    u = float("inf")

    heapq.heappush(open_forward, start)
    heapq.heappush(open_back, goal)

    closed_forward[start.state_hash()] = start
    closed_back[goal.state_hash()] = goal
    
    while open_forward and open_back:
        if u < open_forward[0].get_g() + open_back[0].get_g():
            return u, {**closed_forward, **closed_back}
        if open_forward[0].get_g() < open_back[0].get_g():
            n = heapq.heappop(open_forward)
            for n_prime in gridded_map.successors(n):
                if n_prime.state_hash() in closed_back:
                    u = min(u, n_prime.get_g() + closed_back[n_prime.state_hash()].get_g())
                if n_prime.state_hash() not in closed_forward:
                    heapq.heappush(open_forward, n_prime)
                    closed_forward[n_prime.state_hash()] = n_prime
                if n_prime.state_hash() in closed_forward and n_prime.get_g() < closed_forward[n_prime.state_hash()].get_g():
                    heapq.heappush(open_forward, n_prime)
                    closed_forward[n_prime.state_hash()] = n_prime
        else:
            n = heapq.heappop(open_back)
            for n_prime in gridded_map.successors(n):
                if n_prime.state_hash() in closed_forward:
                    u = min(u, n_prime.get_g() + closed_forward[n_prime.state_hash()].get_g())
                if n_prime.state_hash() not in closed_back:
                    heapq.heappush(open_back, n_prime)
                    closed_back[n_prime.state_hash()] = n_prime
                if n_prime.state_hash() in closed_back and n_prime.get_g() < closed_back[n_prime.state_hash()].get_g():
                    heapq.heappush(open_back, n_prime)
                    closed_back[n_prime.state_hash()] = n_prime
    return -1, {**closed_forward, **closed_back}

def main():
    """
    Function for testing your A* and Dijkstra's implementation. There is no need to edit this file.
    Run it with a -help option to see the options available. 
    """
    optlist, _ = getopt.getopt(sys.argv[1:], 'h:m:r:', ['testinstances', 'plots', 'help'])

    plots = False
    for o, a in optlist:
        if o in ("-help"):
            print("Examples of Usage:")
            print("Solve set of test instances: main.py --testinstances")
            print("Solve set of test instances and generate plots: main.py --testinstances --plots")
            exit()
        elif o in ("--plots"):
            plots = True
        elif o in ("--testinstances"):
            test_instances = "test-instances/testinstances.txt"
                              
    gridded_map = Map("dao-map/brc000d.map")
    
    nodes_expanded_dijkstra = []    
    nodes_expanded_bibs = []
    
    start_states = []
    goal_states = []
    solution_costs = []
       
    file = open(test_instances, "r")
    for instance_string in file:
        list_instance = instance_string.split(",")
        start_states.append(State(int(list_instance[0]), int(list_instance[1])))
        goal_states.append(State(int(list_instance[2]), int(list_instance[3])))
        
        solution_costs.append(float(list_instance[4]))
    file.close()
        
    for i in range(0, len(start_states)):    
        start = start_states[i]
        goal = goal_states[i]
    
        cost, expanded_diskstra = dijkstra(start, goal, gridded_map) # Implement here the call to your Dijkstra's implementation for start, goal, and gridded_map

        nodes_expanded_dijkstra.append(expanded_diskstra)

        if cost != solution_costs[i]:
            print("There is a mismatch in the solution cost found by Dijkstra and what was expected for the problem:")
            print("Start state: ", start)
            print("Goal state: ", goal)
            print("Solution cost encountered: ", cost)
            print("Solution cost expected: ", solution_costs[i])
            print()

        cost, expanded_astar = bi_bs(start, goal, gridded_map) # Implement here the call to your Bi-BS's implementation for start, goal, and gridded_map

        nodes_expanded_bibs.append(expanded_astar)
        
        if cost != solution_costs[i]:
            print("There is a mismatch in the solution cost found by Bi-HS and what was expected for the problem:")
            print("Start state: ", start)
            print("Goal state: ", goal)
            print("Solution cost encountered: ", cost)
            print("Solution cost expected: ", solution_costs[i])
            print()
    
    if plots:
        from search.plot_results import PlotResults
        plotter = PlotResults()
        plotter.plot_results(nodes_expanded_bibs, nodes_expanded_dijkstra, "Nodes Expanded (Bi-HS)", "Nodes Expanded (Dijkstra)", "nodes_expanded")
    
    print('Finished running all experiments.')

if __name__ == "__main__":
    main()
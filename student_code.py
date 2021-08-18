# Implement A* search
import heapq
import math
import sys

# Helper function to calculate distance with Pythagoras
def dist(start, end):
    res = math.sqrt((start[0] - end[0])**2 + (start[1] - end[1])**2)
    return res

def shortest_path(M,start,goal):
    # Base case
    if start == goal:
        return [start]
    
    # 1. Create path MIN HEAP as nested list -> Insert/delete: O(log(n))
    # path[n][0] = Total cost (f)
    # path[n][1] = dict: key = g, value = path cost; 
    # key = path, value = list of intersection ids)
    path = []
    # Initial filling
    for neighbour in M.roads[start]:
        # Calculate distance cost with Pythagoras helper function
        g = dist(M.intersections[start], M.intersections[neighbour])
        h = dist(M.intersections[neighbour], M.intersections[goal])
        f = g + h
        heapq.heappush(path, [f, {'g' : g, 'route' : [start, neighbour]}])
    
    # Traverse map by chosing lowest cost as next
    final_paths = [(sys.maxsize, 0)]
    search = True
    
    while search:
        # Current lowest cost f -> MIN HEAP
        current = heapq.heappop(path)
        cur_f = current[0]
        
        # Break-up condition: traverse as long current is smaller than final_paths
        # final_paths MIN HEAP tuple: vlaue1 = f, value2 = route
        if cur_f > final_paths[0][0]:
            search = False
            break
        
        cur_g = current[1]['g']
        cur_route = current[1]['route']
        cur_intersec = current[1]['route'][-1]
        
        for neighbour in M.roads[cur_intersec]:
            # Continue only for not yet explored neighbours -> important for obstacles
            if neighbour not in cur_route:
                
                # Calculate distance cost with Pythagoras helper function
                g_new = dist(M.intersections[cur_intersec], M.intersections[neighbour])
                g = g_new + cur_g
                h = dist(M.intersections[neighbour], M.intersections[goal])
                f = g + h
                route = cur_route + [neighbour]
                # If goal achieved, add to final_paths
                if neighbour == goal:
                    heapq.heappush(final_paths, (f, route))
                else:
                    heapq.heappush(path, [f, {'g' : g, 'route' : route}])
    
    # Return lowest cost path as list
    return final_paths[0][1]
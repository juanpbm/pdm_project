import numpy as np
import cv2 as cv
import sys
from tqdm import tqdm

sys.setrecursionlimit(20_000)

class Conex:
    # Constructor of the class
    def __init__(self,p,c,cst, id):
        self.id = id

        self.parent = (p[0],p[1])

        self.child = (c[0],c[1])

        self.cost = cst

class RRT:
    def __init__(self):
        self.N_max = 2500                                                                                  # Maximum number of iterations
        self.N = 2000
        self.gamma = 400                                                                                  # Gamma value for the algorithm
        self.d = 2                                                                                          # d value for the algorithm

    # Returns the euclidean distance between 2 points
    def EuclideanDistance(self, p_1,p_2):

        return (((p_1[0]-p_2[0])**2)+((p_1[1]-p_2[1])**2))**(1/2)

    # Returns true if the given point is not inside an obstacle nor is around one
    def No_obstacle(self, img,p_w,p_h):
        if(img[p_w,p_h] == 255 and img[p_w,p_h+1] == 255 and img[p_w+1,p_h] == 255 and img[p_w+1,p_h+1] == 255
        and img[p_w,p_h-1] == 255 and img[p_w-1,p_h] == 255 and img[p_w-1,p_h-1] == 255 and img[p_w-1,p_h+1]==255 and img[p_w+1,p_h-1]==255 and img[p_w,p_h+2] == 255 
        and img[p_w+2,p_h] == 255 and img[p_w+2,p_h+2] == 255 and img[p_w,p_h-2] == 255 and img[p_w-2,p_h] == 255 and img[p_w-2,p_h-2] == 255 
        and img[p_w-2,p_h+2]==255 and img[p_w+2,p_h-2]==255):
            return True
        else:
            return False

    # Returns the closest neighbor in the already generated graph to the newly randomly created point
    def GetClosestNeighbor(self, p_w,p_h,V_E,r):
        # Initialize distance, closest neighbor and a list to add all the neighbors within a radius 'r'
        dist = 100000000
        closest = Conex((None,None),(None,None), None,-1)
        neighbor = []

        # Go through all the nodes of the graph
        for i in V_E:
            d = self.EuclideanDistance(i.child,(p_w,p_h))                                                       # Get the distance between each node and the new point
            
            # If the distance is less or equal than the radius
            if(d<=r):
                neighbor.append(i)                                                                                # Append the node
                
                # If it is the smallest distance among all of the ones computed until now
                if(d<dist):
                    # Update the closest neighbor 
                    if(i.cost != None):
                        dist = d + i.cost
                    else:
                        dist = d
                    closest = i            

        return closest, neighbor, dist

    # Returns true if the position between the 2 points is clear (no obstacles)
    def ClearLine(self,new_w,new_h,p,img):
        # Get the coordinates of the point
        x1, y1 = p.child
        
        # Get points between the coordinates of both points
        points = zip(
            np.linspace(x1, new_w, num=10000, dtype=int),
            np.linspace(y1, new_h, num=10000, dtype=int))
        
        # If it is not crossing an obstacle, return True
        for x, y in points:
            if img[x, y] < 255:
                return False
        return True
    
    # Update the costs of the different nodes when updating the position of one of the nodes of the graph
    def update_costs(self,node, V_E):
        stack = [node]                                                                                              # Initialize a stack and a visited set
        visited = set()                                                                                             # Set to track visited nodes

        while stack:
            current_node = stack.pop()                                                                              # Pop the current node from the stack

            # Skip the visited nodes
            if current_node.child in visited:                                                                       
                continue

            visited.add(current_node.child)                                                                         # Mark the current node as visited

            # Update the costs of the edges where the current node is the parent
            for edge in V_E:
                if edge.parent == current_node.child:
                    V_E[edge.id].cost = current_node.cost + self.EuclideanDistance(current_node.child, edge.child)               # Update the edge cost
                    
                    if edge.child not in visited:                                                                   # Only add unvisited nodes
                        stack.append(edge)                                                                          # Add this edge to the stack

    # Get a smoother trajectory within the shortest path
    def smooth_path(self,path, img):
        smoothed_path = [path[0]]                                                                                   # Initialize the smoothed path with the last node
        i = 0

        while i < len(path) - 1:
            j = len(path) - 1                                                                                       # Start from the beggining of the path
            while j > i:
                # If there is a clear line in between
                if self.ClearLine(path[j].child[0], path[j].child[1], path[i], img):
                    smoothed_path.append(path[j])                                                                   # Add the farthest visible node
                    i = j
                    break
                j = j - 1
            else:
                smoothed_path.append(path[i + 1])
                i = i + 1

        return smoothed_path

    # RRT* algorithm
    def RRT_star(self, img,start,end,rad_f,size,image):
        # Create the start node and append it to the list of the graph
        s = Conex((None,None),start, None,0)
        V_E = []
        V_E.append(s)
        end_reached = False
        idx = -1

        # Go through iterations to get the trajectory in order to arrive at the goal position
        for n in tqdm(range(self.N_max)):
            # Change the radius of updating depending on the iteration number
            rad = self.gamma * (np.log(n+1) / (n+1)) ** (1 / self.d)

            # Auxiliary values to make the algorithm work
            new_node = None
            cost = 9999999
            ok = False

            # Iterate until it finds a node that can be introduced to the graph:
            #   - It has to be outside an obstacle
            #   - The line between the closes neighbor and the point must be clear (do not go through an obstacle)
            while(ok == False):
                # With a 10% of probability, the new node will be the last node - THIS IS NOT PART OF THE RRT* BUT AN IMPROVEMENT FOR OUR PROBLEM
                if (np.random.uniform(0, 1) < 0.1 and end_reached == False):
                    new_w = end[0]
                    new_h = end[1]
                else:
                    # Randomize the position of the new point with a certain margin
                    new_w = np.random.randint(2,size[0]-2)
                    new_h = np.random.randint(2,size[1]-2)

                # If the new point is not inside an obstacle
                if(self.No_obstacle(img,new_w,new_h)):
                    ok = True

                    [closest, neigh, cost] = self.GetClosestNeighbor(new_w,new_h,V_E,rad)                            # Get the closest neighbor to the point

                    # If it is a valid neighbor
                    if(closest.id != -1 and closest.child!=end):
                        # If there is a clear line between the point and the closest neighbor
                        if(self.ClearLine(new_w,new_h,closest,img) == True):
                            # Create and add the node to the list
                            if((new_w,new_h) == end):
                                end_reached = True

                            new_node = Conex(closest.child,(new_w,new_h), cost,n+1)
                            V_E.append(new_node)

                            image = cv.circle(image, (new_node.child[1], new_node.child[0]), 2, (0,255,0), -1)          # Draw the point on the image

                            # Go through all the neighbors of the new node to update their costs if necessary
                            for nei in neigh:
                                # The cost will be updated if:
                                #   - The cost of the neighbor is not None (not start point)
                                #   - The new cost is smaller that the older cost of the neighbor
                                #   - The new neighbor is  not the closest one
                                #   - There is a clear line between the neighbor and the new node
                                if((nei.cost != None and cost + self.EuclideanDistance(new_node.child,nei.child) < nei.cost) and nei.id!=closest.id 
                                and self.ClearLine(nei.child[0],nei.child[1],new_node,img) == True and nei.child != end):
                                    # Update the cost of the 
                                    V_E[nei.id].parent = new_node.child
                                    V_E[nei.id].cost = cost + self.EuclideanDistance(new_node.child, nei.child)

                                    # Update the costs of the child nodes of the update node
                                    self.update_costs(nei, V_E)

                        else:
                            ok = False
                    else:
                        ok= False
                else:
                    ok = False
            # If the new node is not None and the new node is inside the indicated radius
            if(new_node!=None):
                if(self.EuclideanDistance(end, new_node.child)==0):
                    idx =  new_node.id

            if(n >= self.N and idx >= 0):
                break                                                                                       # Finish the algorithm
            

        return V_E,image,idx

    # Find the shortest path between the last node and the start one
    def find_shortest(self, shortest, V_E,last):
        # If we get to the start point
        if(last.parent == (None,None)):
            shortest.append(last)
            return shortest

        # Go through all points on the list of the graph
        for point in V_E:
            if(point.child == last.parent):
                shortest.append(last)
                return self.find_shortest(shortest, V_E,point)
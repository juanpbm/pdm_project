import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import numpy as np
import cv2 as cv


message = []

class Conex:
    def __init__(self,p,c,cst, id):
        self.id = id

        self.parent = (p[0],p[1])

        self.child = (c[0],c[1])

        self.cost = cst


class RRT:
    def __init__(self):
        self.N = 10000
        self.gamma = 0.9
        self.d = 3

    def EuclideanDistance(self, p_1,p_2):

        return (((p_1[0]-p_2[0])**2)+((p_1[1]-p_2[1])**2))**(1/2)

    def No_obstacle(self, img,p_w,p_h):
        if(img[p_w,p_h] == 255 and img[p_w,p_h+1] == 255 and img[p_w+1,p_h] == 255 and img[p_w+1,p_h+1] == 255
        and img[p_w,p_h-1] == 255 and img[p_w-1,p_h] == 255 and img[p_w-1,p_h-1] == 255 and img[p_w-1,p_h+1]==255 and img[p_w+1,p_h-1]==255 and img[p_w,p_h+2] == 255 
        and img[p_w+2,p_h] == 255 and img[p_w+2,p_h+2] == 255 and img[p_w,p_h-2] == 255 and img[p_w-2,p_h] == 255 and img[p_w-2,p_h-2] == 255 
        and img[p_w-2,p_h+2]==255 and img[p_w+2,p_h-2]==255):
            return True
        else:
            return False

    def GetClosestNeightbour(self, p_w,p_h,V_E,r):
        dist = 100000000
        closest = Conex((None,None),(None,None), None,-1)
        neight = []

        for i in V_E:
            d = self.EuclideanDistance(i.child,(p_w,p_h))
            #print(d)
            if(d<=r):
                neight.append(i)
                if(d<dist):
                    if(i.cost != None):
                        dist = d + i.cost
                    else:
                        dist = d
                    closest = i           

        return closest,neight,dist


    def ClearLine(self, new_w,new_h,closest,img):
        x1, y1 = closest.child

        points = zip(
            np.linspace(x1, new_w, num=150, dtype=int),
            np.linspace(y1, new_h, num=150, dtype=int)
        )
        for x, y in points:
            if img[x, y] < 255:
                return False
        return True


    def update_costs(self, node, V_E, img):
        for edge in V_E:
            if edge.parent == node.child:
                edge.cost = node.cost + self.EuclideanDistance(node.child, edge.child)
                self.update_costs(edge, V_E, img)

    def smooth_path(self, path, img):

        smoothed_path = [path[0]]  # Start with the first node // End
        i = 0

        while i < len(path) - 1:
            j = len(path) - 1  # Start from the end of the path // Start
            while j-1 > i:
                if self.ClearLine(path[j].child[0], path[j].child[1], path[i], img):  # Check line of sight
                    smoothed_path.append(path[j])  # Add the farthest visible node
                    i = j  # Skip to this node
                    break
                j = j - 1
            else:
                i = i + 1

        return smoothed_path


    def RRT_star(self, img,start,end,rad_f,size,image):
        s = Conex((None,None),start, None,0)

        V_E = []
        V_E.append(s)
        gamma = 400
        d = 2

        for n in range(self.N):
            #rad = gamma*(np.log(n+1)/n+1)**(1/d)
            rad = gamma * (np.log(n+1) / (n+1)) ** (1 / d)

            new_node = None
            cost = 9999999
            ok = False
            while(ok == False):
                new_w = np.random.randint(2,size[0]-2)
                new_h = np.random.randint(2,size[1]-2)

                if np.random.uniform(0, 1) < 0.1:
                    new_w = end[0]
                    new_h = end[1]
                else:
                    new_w = np.random.randint(2,size[0]-2)
                    new_h = np.random.randint(2,size[1]-2)

                if(self.No_obstacle(img,new_w,new_h)):
                    ok = True

                    [closest, neight, cost] = self.GetClosestNeightbour(new_w,new_h,V_E,rad)

                    if(closest.id != -1):

                        if(self.ClearLine(new_w,new_h,closest,img) == True):
                            #print("Cost: "+str(cost))
                            new_node = Conex(closest.child,(new_w,new_h), cost,n+1)
                            V_E.append(new_node)
                            #print("Color: "+str(img[new_w,new_h]))
                            image = cv.circle(image, (new_node.child[1], new_node.child[0]), 2, (0,255,0), -1)

                            for n in neight:
                                #print(cost + EuclideanDistance(new_node.child,n.child))

                                #print("Another: "+str(n.cost)+ "    " + str(n.id))
                                if((n.cost != None and cost + self.EuclideanDistance(new_node.child,n.child) < n.cost) and n.id!=closest.id 
                                and self.ClearLine(n.child[0],n.child[1],new_node,img) == True):
                                    n.parent = new_node.child
                                    n.cost = cost + self.EuclideanDistance(new_node.child, n.child)
                                    # IMPORTANTE
                                    self.update_costs(n, V_E, img)

                        else:
                            ok = False
                    else:
                        ok= False
                else:
                    ok = False

            if(new_node!=None):
                if(self.EuclideanDistance(end, new_node.child)<=rad_f):
                    #print(new_node.child)
                    #print(end)
                    print("Done")
                    break

        return V_E,image

    def find_shortest(self, shortest, V_E,last):
        if(last.parent == (None,None)):
            shortest.append(last)
            return shortest

        for point in V_E:
            if(point.child == last.parent):
                shortest.append(last)
                return self.find_shortest(shortest, V_E,point)

        print("fuera")

        #return shortest.append(find_shortest(shortest, V_E,last))
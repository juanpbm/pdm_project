#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import numpy as np
import cv2 as cv
from robot_trials.msg import BasePlan

N = 10000
gamma = 0.9
d = 3
message = []

class Conex:
    def __init__(self,p,c,cst, id):
        self.id = id

        self.parent = (p[0],p[1])

        self.child = (c[0],c[1])

        self.cost = cst

def EuclideanDistance(p_1,p_2):

    return (((p_1[0]-p_2[0])**2)+((p_1[1]-p_2[1])**2))**(1/2)

def No_obstacle(img,p_w,p_h):
    if(img[p_w,p_h] == 255 and img[p_w,p_h+1] == 255 and img[p_w+1,p_h] == 255 and img[p_w+1,p_h+1] == 255
       and img[p_w,p_h-1] == 255 and img[p_w-1,p_h] == 255 and img[p_w-1,p_h-1] == 255 and img[p_w-1,p_h+1]==255 and img[p_w+1,p_h-1]==255 and img[p_w,p_h+2] == 255 
       and img[p_w+2,p_h] == 255 and img[p_w+2,p_h+2] == 255 and img[p_w,p_h-2] == 255 and img[p_w-2,p_h] == 255 and img[p_w-2,p_h-2] == 255 
       and img[p_w-2,p_h+2]==255 and img[p_w+2,p_h-2]==255):
        return True
    else:
        return False

def GetClosestNeightbour(p_w,p_h,V_E,r):
    dist = 100000000
    closest = Conex((None,None),(None,None), None,-1)
    neight = []

    for i in V_E:
        d = EuclideanDistance(i.child,(p_w,p_h))
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


def ClearLine(new_w,new_h,closest,img):
    x1, y1 = closest.child

    points = zip(
        np.linspace(x1, new_w, num=150, dtype=int),
        np.linspace(y1, new_h, num=150, dtype=int)
    )
    for x, y in points:
        if img[x, y] < 255:
            return False
    return True


def update_costs(node, V_E, img):
    for edge in V_E:
        if edge.parent == node.child:
            edge.cost = node.cost + EuclideanDistance(node.child, edge.child)
            update_costs(edge, V_E, img)

def smooth_path(path, img):

    smoothed_path = [path[0]]  # Start with the first node // End
    i = 0

    while i < len(path) - 1:
        j = len(path) - 1  # Start from the end of the path // Start
        while j-1 > i:
            if ClearLine(path[j].child[0], path[j].child[1], path[i], img):  # Check line of sight
                smoothed_path.append(path[j])  # Add the farthest visible node
                i = j  # Skip to this node
                break
            j = j - 1
        else:
            i = i + 1

    return smoothed_path


def RRT_star(img,start,end,rad_f,size,image):
    s = Conex((None,None),start, None,0)

    V_E = []
    V_E.append(s)
    gamma = 400
    d = 2

    for n in range(N):
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

            if(No_obstacle(img,new_w,new_h)):
                print(n)
                ok = True

                [closest, neight, cost] = GetClosestNeightbour(new_w,new_h,V_E,rad)

                if(closest.id != -1):

                    if(ClearLine(new_w,new_h,closest,img) == True):
                        #print("Cost: "+str(cost))
                        new_node = Conex(closest.child,(new_w,new_h), cost,n+1)
                        V_E.append(new_node)
                        #print("Color: "+str(img[new_w,new_h]))
                        image = cv.circle(image, (new_node.child[1], new_node.child[0]), 2, (0,255,0), -1)

                        for n in neight:
                            #print(cost + EuclideanDistance(new_node.child,n.child))

                            #print("Another: "+str(n.cost)+ "    " + str(n.id))
                            if((n.cost != None and cost + EuclideanDistance(new_node.child,n.child) < n.cost) and n.id!=closest.id 
                               and ClearLine(n.child[0],n.child[1],new_node,img) == True):
                                n.parent = new_node.child
                                n.cost = cost + EuclideanDistance(new_node.child, n.child)
                                # IMPORTANTE
                                update_costs(n, V_E, img)

                    else:
                        ok = False
                else:
                    ok= False
            else:
                ok = False

        if(new_node!=None):
            if(EuclideanDistance(end, new_node.child)<=rad_f):
                #print(new_node.child)
                #print(end)
                print("Done")
                break

    return V_E,image

def find_shortest(shortest, V_E,last):
    if(last.parent == (None,None)):
        shortest.append(last)
        return shortest

    for point in V_E:
        if(point.child == last.parent):
            shortest.append(last)
            return find_shortest(shortest, V_E,point)

    print("fuera")

    #return shortest.append(find_shortest(shortest, V_E,last))


class BasePlanning(Node):
    def __init__(self):
      super().__init__('base_planning_node')
      self.publisher_ = self.create_publisher(BasePlan, '/base_points', 10)
      timer_period = 0.5  # seconds
      self.timer = self.create_timer(timer_period, self.send_plan)
      self.i = 0

    def send_plan(self):
      if(self.i == 0):
        msg = BasePlan()
        msg.pos = message
        #msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.pos)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    base_plan = BasePlanning()

    rclpy.spin(base_plan)

    base_plan.destroy_node()
    rclpy.shutdown()



if __name__== "__main__":
    img = cv.imread('/home/zuleikarg/Desktop/pdm/second.jpg')

    # Properties of an Image
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    ret,thresh = cv.threshold(gray,127,255,0)

    size = img.shape
    start = (50,50)
    end = (325,450)
    rad = 10

    img_print = img.copy()
    image = img.copy()

    kernel = np.ones((8, 8), np.uint8) 
  
    # Using cv2.erode() method  
    image = cv.erode(thresh, kernel, cv.BORDER_REFLECT)  

    # Draw a circle of red color of thickness -1 px 
    img_print = cv.circle(img_print, (start[1],start[0]), 3, (100,255,255), -1) 

    # Draw a circle of red color of thickness -1 px 
    img_print = cv.circle(img_print, (end[1],end[0]), rad, (255,0,0), 1) 

    [V_E, img_print] = RRT_star(np.squeeze(image), start, end,rad,size,img_print)
    V_E = np.asarray(V_E)
    for i in V_E:
        #print(V_E.shape)
        if(i.parent != (None,None)):
            img_print = cv.line(img_print, (i.child[1],i.child[0]), (i.parent[1],i.parent[0]), (255,0,255), 1)


    V_E_shortest = find_shortest([V_E[-1]],V_E,V_E[-1])

    V_E_shortest_smoothed = smooth_path(V_E_shortest, np.squeeze(image))

    # Draw the smoothed path
    j = len(V_E_shortest_smoothed) - 1
    for i in range(len(V_E_shortest_smoothed) - 1):
        p1 = V_E_shortest_smoothed[i].child
        p2 = V_E_shortest_smoothed[i + 1].child
        if(i == 0):
            message.append(V_E_shortest_smoothed[j].child[0]/100)
            message.append(V_E_shortest_smoothed[j].child[1]/100)

        message.append(V_E_shortest_smoothed[j-1].child[0]/100)
        message.append(V_E_shortest_smoothed[j-1].child[1]/100)
        j = j-1

        img_print = cv.line(img_print, (p1[1], p1[0]), (p2[1], p2[0]), (0, 0, 255), 2)  # Smoothed path in yellow

    print(message)

    main()
    # Display the Binary Image
    #cv.imshow("Binary Image", img_print)
    #cv.waitKey(0)
    #cv.destroyAllWindows()

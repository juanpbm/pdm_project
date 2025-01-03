import numpy as np
class custom_URDF:
    def __init__(self,origin=np.identity(4),axis=np.zeros(3)):
      self.origin=origin
      self.axis=axis
      

    def create_list(self):
        self.joints_list=[]

    def add_joint(self,joint):
        self.joints_list.append(joint)

    def get_joints(self):
        return self.joints_list
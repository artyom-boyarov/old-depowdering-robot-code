import numpy as np

def calculate_pos(points):
    points_num = len(points)
    sum_x = 0
    sum_y = 0
    sum_z = 0
    for i in range(points_num):
      sum_x += points[i,0]
      sum_y += points[i,1]
      sum_z += points[i,2]

    sum_x /= points_num
    sum_y /= points_num
    sum_z /= points_num
   
    return [sum_x,sum_y,sum_z]
   

stl_path='D:\\SOFTWARE\\solidworks\\Robotics_Depowdering\\stl_cart_model\\new_cart_model_object2_1.STL'
points=[]
f = open(stl_path)
lines = f.readlines()
prefix='         vertex'
num=3
for line in lines:
    if line.startswith(prefix): 
        values = line.strip().split()
        if num%3==0:
          points.append([float(values[1]),float(values[2]),float(values[3])])
          num=0
        num+=1

points=np.array(points)
f.close()
print(points.shape)

print(points)
print(calculate_pos(points))


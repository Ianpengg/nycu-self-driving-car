#%%
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import json
import os
import re



def natural_key(string_):
    return [int(s) if s.isdigit() else s for s in re.split(r'(\d+)', string_)]


root_dir = '/home/ee904/Downloads/tracking_test_v1.1/argoverse-tracking/test/0f0d7759-fa6e-3296-b528-6c862d061bdd/'

pose_dir = root_dir + 'poses/'
pose_names = sorted([f for f in os.listdir(pose_dir)], key = natural_key)
pose_files = pose_dir + pose_names[0]

#print(pose_files)
with open(pose_files) as f:
    pose_data = json.load(f)["translation"][0]
    #print(type(pose_data))

def distance(x, prev_x, y, prev_y):
    return np.sqrt((x-prev_x)**2 + (y-prev_y)**2,dtype=np.float64)  
  
def readJson():
    with open(pose_files) as f:
        waypoints = json.load(f)['translation']
        ww = pd.DataFrame(waypoints)
        #print(type(waypoints))
        # returns Json object as a dictionary
        trans_x = waypoints[0]
        trans_y = waypoints[1]
        trans_z = waypoints[2]
    return trans_x,trans_y,trans_z
x_traj = []
y_traj = []
z_traj = []
pose_distance = []
previous_pose = np.empty((1,3))
result = np.zeros((1,3))
for i in range(3652):
    pose_files = pose_dir + pose_names[i]
    x,y,z = readJson()
    x_traj+=[x]
    y_traj+=[y]
    z_traj+=[z]
    if i is not 0:
        pose_distance+=[distance(x, previous_pose[0,0], y, previous_pose[0,1])]
    previous_pose[0,0] = x
    previous_pose[0,1] = y
    previous_pose[0,2] = z
plt.plot(x_traj,y_traj)
plt.show()
#print(a)
a = distance(2,1,2,1)
print(type(a))
# %%
plt.figure(figsize=(20,10))
plt.plot(pose_distance[0:150])
plt.show

# %%

#%%
from collections import deque
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import json
import os
import re
from pathlib import Path
from argo_tracking.src.python_version.processing_utils import *
from pandas import json_normalize


def natural_key(string_):
    return [int(s) if s.isdigit() else s for s in re.split(r'(\d+)', string_)]
def read_track_json(tracking_file):
    
    
    center_list = []
    track_id = []
    with open (tracking_file) as f:
        #print((track_data))
        track_data = (json.load(f))
        for i, data in enumerate(track_data):
            center = np.zeros((2))
            
            track_id.append(data['track_label_uuid'])
            center[0] = data['center']['x']
            center[1] = data['center']['y']
            center_list.append(center)
        print(center_list)
        
            #print(center)
            # center[2] = track_data[i]['center']['z']
            #print(center)
            #print(center_list)
            
        pass
        #print(len(track_data))
        #print(cc['x'])
        #print(center_list)
    #print(track_id)
    #print(center_list)    
    #return track_id, center_list
tracking_dir = "/home/ee904/argoverse_cbgs_kf_tracker/temp_files/test-split-track-preds-maxage15-minhits5-conf0.3/0f0d7759-fa6e-3296-b528-6c862d061bdd/per_sweep_annotations_amodal/"
tracking_file = sorted([f for f in os.listdir(tracking_dir)], key = natural_key)
centers = {}
id, object_center = read_track_json(tracking_dir+tracking_file[1])
for track_id, center in zip(id,object_center):
    centers[track_id] = center
#print(centers)
#print(id)




#%%
root_dir = '/home/ee904/Downloads/tracking_test_v1.1/argoverse-tracking/test/1b5c73a4-402b-3139-846f-11e74ba3faed/'
lidar_dir = root_dir + 'lidar/'
pose_dir = root_dir + 'poses/'

lidar_names = sorted([f for f in os.listdir(lidar_dir)], key = natural_key)
pose_names = sorted([f for f in os.listdir(pose_dir)], key = natural_key)
# pose_files = pose_dir + pose_names[0]
po = string_split(pose_names)
li = string_split(lidar_names)
index = []
for idx,data in enumerate((po)):
    if data in li:
        index+=[idx]



def readJson(pose_files):
    with open(pose_files) as f:
        pose_data = json.load(f)
        pose = np.array(pose_data['translation'])
        rotation = np.array(pose_data['rotation'])
        r,p,y = euler_from_quaternion(rotation[0],rotation[1],rotation[2],rotation[3]) 
        # returns Json object as a dictionary
        
    return pose, y





#%%
previous_pose = None
previous_rotation = None
locations = []
for i in range(len(pose_names)):
#for i in range(5):    
    #pose_files = pose_dir + pose_names[i]
    pose,rotation = readJson(pose_dir + pose_names[i])
    #print(pose)
    #displacement = distance(pose[[0,1]],previous_pose[[0,1]])
    if previous_pose is not None:
        displacement = distance(pose[[0,1,2]],previous_pose[[0,1,2]])
        yaw_change = float(rotation- previous_rotation)
        
        for j in range(len(locations)):
            x0, y0 = locations[j]
            x1 = x0 * np.cos(yaw_change) + y0 * np.sin(yaw_change) -displacement
            y1 = -x0 * np.sin(yaw_change) + y0 * np.cos(yaw_change)
            locations[j] = np.array([x1,y1])
    locations += [np.array([0,0])]
    previous_pose = pose
    previous_rotation = rotation

plt.figure(figsize=(20,10))
plt.plot(np.array(locations)[:,0], np.array(locations)[:,1])
#plt.ylim((-1,1))

plt.show()

print(locations)


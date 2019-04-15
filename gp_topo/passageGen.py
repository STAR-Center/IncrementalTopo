#!/usr/bin/env python
'''
This script will take rainFlow count map and split the river(skeleton) to generate passage
'''
'''
How to make passage better:
1. if the k1,k2 for saddle, end1,end2 are not parallel, dismissed.
2. if both the end1,end2 are lane, dismissed
'''
import numpy as np
import sys
import cv2
from skimage.morphology import skeletonize
import matplotlib.pyplot as plt
import pdb
#actions
action = np.repeat(np.asarray([-1,0,1]),3).reshape((3,3))

local_action = (action.flatten(), action.T.flatten())



def find_passage_point(saddle,gau,bina):
    '''
        Here we have the middle point of passage, here we want the end point of end passage
        input:
            saddle:tuple(hp,wp)
            gau:array(h,w)
    '''
    h,w = gau.shape
    passageEnd_set = []
    for i in range(-2,3):
        for j in range(-2,3):
            posi = (saddle[0]+i, saddle[1]+j)
            #find the end vertex            
            stopped = False
            while not stopped:
                update_hs = local_action[0]+posi[0]
                update_hs = np.minimum(update_hs, h-1)
                update_hs = np.maximum(update_hs, 0)
                update_ws = local_action[1]+posi[1]
                update_ws = np.minimum(update_ws, w-1)
                update_ws = np.maximum(update_ws, 0)
                local_feature = gau[(update_hs, update_ws)].copy()
                best_update_id = np.argmax(local_feature)

                update_posi = (update_hs[best_update_id], update_ws[best_update_id])
                if gau[posi] == gau[update_posi]:
                    stopped = True
                    passageEnd_set.append(posi)
                else:
                    posi = update_posi
    #check freq and sort
    freqBook = [[x,passageEnd_set.count(x)] for x in set(passageEnd_set)]
    freqs = []
    for i in range(len(freqBook)):
        freqs.append(freqBook[i][1])
    freqs = np.asarray(freqs)
    End_id1 = np.argmax(freqs) 
    freqs[End_id1] = 0
    End_id2 = np.argmax(freqs)

    return freqBook[End_id1][0], freqBook[End_id2][0]
def getEnds(saddlesMap,gau,bina):
    '''
        provide saddles,endpt1s,endpt2s correspondingly
    '''
    saddles,endpt1s,endpt2s = [],[],[]
    saddles_hs, saddles_ws = np.where(saddlesMap>0)
    for i in range(len(saddles_hs)):
        end1,end2 = find_passage_point((saddles_hs[i],saddles_ws[i]), gau,bina)
        saddles.append((saddles_hs[i],saddles_ws[i]))
        endpt1s.append(end1)
        endpt2s.append(end2)
    return saddles,endpt1s,endpt2s
def connect(amap, state1, state2):
    '''
        connect point state1 and state2 on amap
    '''
    left = [0,0]
    right = [0,0]

    if state1[1] < state2[1]:
        left[0] = state1[0]
        left[1] = state1[1]
        right[0] = state2[0]
        right[1] = state2[1]
    elif state1[1] > state2[1]:
        left[0] = state2[0]
        left[1] = state2[1]
        right[0] = state1[0]
        right[1] = state1[1]
    else:
        #verticle line
        amap[min(state2[0],state1[0]):max(state2[0],state1[0]), state2[1]] = 1
        return

    if right[0] == left[0]:
        #horizon line
        amap[right[0],min(state2[1],state1[1]):max(state2[1],state1[1])] = 1
        return

    #slash
    k = float(right[0] - left[0]) / (right[1] - left[1])

    trajectory_X = np.asarray(range(left[1],right[1]+1))
    trajectory_Y = (k*(trajectory_X-left[1]) + left[0]).astype(np.int)

    amap[(trajectory_Y,trajectory_X)] = 1

def connectPassage(saddles, endpt1s, endpt2s, bina, gau):
    '''
        Here we gonna connect saddle with endpt1 and endpt2
        input:
            saddles:[n]list(tuple)
            endpt1s:as above(correspondingly)
            endpt2s:as above(correspondingly)
    '''
    bina_with_passage = bina.copy()
    bina_skele = bina.copy()#skeletonize(bina_with_passage)
    for i in range(len(saddles)):
#print abs((saddles[i][0]-endpt1s[i][0])*(saddles[i][1]-endpt2s[i][1]) -  (saddles[i][0]-endpt2s[i][0])*(saddles[i][1]-endpt1s[i][1]))
        #end1,end2 are parallel, set to reasonable
        if abs((saddles[i][0]-endpt1s[i][0])*(saddles[i][1]-endpt2s[i][1]) -  (saddles[i][0]-endpt2s[i][0])*(saddles[i][1]-endpt1s[i][1])) < 30:
#if laplacian[endpt1s[i]] <-30 and laplacian[endpt2s[i]] < -30:
#print laplacian[endpt1s[i]], laplacian[endpt2s[i]]
#print np.sum(gau[endpt1s[i][0]-2:endpt1s[i][0]+3,  endpt1s[i][1]-2:endpt1s[i][1]+3]),  np.sum(gau[endpt2s[i][0]-2:endpt2s[i][0]+3,  endpt2s[i][1]-2:endpt2s[i][1]+3])
            if np.sum(bina_skele[endpt1s[i][0]-1:endpt1s[i][0]+2,  endpt1s[i][1]-1:endpt1s[i][1]+2]) == 2 or  np.sum(bina_skele[endpt2s[i][0]-1:endpt2s[i][0]+2,  endpt2s[i][1]-1:endpt2s[i][1]+2]) == 2:
                print abs((saddles[i][0]-endpt1s[i][0])*(saddles[i][1]-endpt2s[i][1]) -  (saddles[i][0]-endpt2s[i][0])*(saddles[i][1]-endpt1s[i][1]))
     
                connect(bina_with_passage, saddles[i], endpt1s[i])
                connect(bina_with_passage, saddles[i], endpt2s[i])
    return bina_with_passage







#-----------not used
def find_passage(river_pixel_posi, homeRiver_map):
    '''
    h,w,_ = homeRiver_map.shape
    rainDrop_hs, rainDrop_ws = [], []
    for hi in range(h):
        for wi in range(w):
            if homeRiver_map[hi,wi,0] == river_pixel_posi[0]:
                if homeRiver_map[hi,wi,1] == river_pixel_posi[1]:
                    rainDrop_hs.append(hi)
                    rainDrop_ws.append(wi)
    '''
    mask = np.logical_and(homeRiver_map[:,:,0] == river_pixel_posi[0],
                        homeRiver_map[:,:,1] == river_pixel_posi[1])
    rainDrop_hws = np.where(mask == True)
    return rainDrop_hws[0],rainDrop_hws[1]

#return np.asarray(rainDrop_hs), np.asarray(rainDrop_ws)

def passageGen(gau, bina, skeleton, count_map, homeRiver_map, th = 100):
    '''
        1. the riverPixel with less than 10 river will be dismissed 
        2. find the rainDrops position that touch this dismissed river pixel at their very frist.(Those rivers are passage)
        ---
        input:
            gau:array[h,w](gaussian density map)
            bina:array[h,w](binary map for the origin map)
            skeleton:array[h,w](binary map for the skeleton map)
            count_map:array[h,w](only the river pixel have value, count how many waterDrop flow through)
            homeRiver_map:array[h,w,2](record the position river pixel that it first touched)
            th:value(threshold to set passage)
    '''
    new_wall = bina.copy()
    #find skeleton pixel
    skele_hs, skele_ws = np.where(skeleton>0)
    #count of each river pixel
    riverCount = count_map[(skele_hs, skele_ws)]#1-D, index on skele_hs/ws
    #get the river pixel that should be dismissed
    passage_ids = np.where(riverCount < th)[0]
    for i in range(len(passage_ids)):
        sys.stdout.write('%d / %d\r'%(i, len(passage_ids)))
        sys.stdout.flush()  
        passage_id = passage_ids[i]
        #position of one passage river pixel
        river_posi = (skele_hs[passage_id], skele_ws[passage_id])
        #those rainDrops that can shape this passage
        rainDrop_hs, rainDrop_ws = find_passage(river_posi, homeRiver_map)
        new_wall[(rainDrop_hs, rainDrop_ws)] = 1
    sys.stdout.write('\n')
    return new_wall

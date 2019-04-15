#!/usr/bin/env python
import numpy as np
import sys
import pdb
'''
The main task for this script is to implement my rainFlow algorithm
'''
def flow(originPosi, gau, bina, skeleton, count_map, local_action):

    posi = (originPosi[0], originPosi[1])
    h,w = skeleton.shape
    #1. if it is on the wall
    if bina[posi] > 0:
        return
    #2. if blank space
    if gau[posi] == 0:
        return
    #3. flow moving
    stopped = False
    while not stopped:
        update_hs = local_action[0]+posi[0]
        update_hs = np.minimum(update_hs, h-1)
        update_hs = np.maximum(update_hs, 0)
        update_ws = local_action[1]+posi[1]
        update_ws = np.minimum(update_ws, w-1)
        update_ws = np.maximum(update_ws, 0)
        local_feature = gau[(update_hs, update_ws)].copy()
        #if it is in river, it have to move to higher density river pixel
        if skeleton[posi]:
            local_feature[ np.where(skeleton[(update_hs,update_ws)] == 0)] = 0
            best_update_id = np.argmax(local_feature)
        #else not in river, it should flow down to find river pixel
        else:
            best_update_id = np.argmin(local_feature)

        update_posi = (update_hs[best_update_id], update_ws[best_update_id])

        #if stopped moving
        if gau[posi] == gau[update_posi]:
            stopped = True
            if skeleton[posi]:
                count_map[posi] += 1
        else:
            posi = update_posi


def rainFlow(OriGau, bina, skeleton):
    '''
        input:
            OriGau:array[h,w](gaussian density map of the path)
            bina:array[h,w](binary map for the origin map)
            skeleton:array[h,w](binary map for the skeleton map)
        output:
            count_map:array[h,w](only the river pixel have value, count how many waterDrop flow through)
            homeRiver_map:array[h,w,2](record the position river pixel that it first touched)
    '''
    gau = OriGau.copy()

    #actions
    action = np.repeat(np.asarray([-1,0,1]),3).reshape((3,3))
    local_action = (action.flatten(), action.T.flatten())

    h,w = gau.shape
    count_map = np.zeros(gau.shape, dtype=np.uint32)
    #it on each pixel
    for hi in range(h):
        sys.stdout.write('%d / %d \r'%(hi+1,h))
        sys.stdout.flush()
        for wi in range(w):
            #Here for each position, we will run the flow algo
            flow((hi,wi), gau, bina, skeleton, count_map, local_action)#array in function should only pass address
    sys.stdout.write('\n')
    return count_map

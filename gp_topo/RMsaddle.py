#!/usr/bin/env python
'''
This script will remove some saddle point
However there're some degree one point that is not saddle point and some low density saddle that may not be a passage.
'''
'''
Here we just let the non-saddle point removed that's ok
'''
import numpy as np
import pdb
def removeSaddle(gau, skeleton, count_map, th = 0.5, rmSize = 5, neighbourTh = 3):
    sk1 = skeleton.copy()
    countTb = (count_map>0)
    newCountTb = countTb.copy()
    chs,cws = np.where(countTb > 0)
    for i in range(len(chs)):
        #if it is long lane, not include
        if np.sum(countTb[chs[i]-rmSize/2:chs[i]+rmSize/2+1, cws[i]-rmSize/2:cws[i]+rmSize/2+1]) >neighbourTh:
            newCountTb[chs[i], cws[i]] = 0
    saddles = gau*newCountTb
    sk1[np.where(saddles>th)] = 0
    return sk1, saddles>th

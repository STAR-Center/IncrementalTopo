#!/usr/bin/env python

import numpy as np
from scipy.misc import imread, imresize
import matplotlib.pyplot as plt
import cv2
import scipy.io
import scipy.stats as st
from scipy import ndimage as ndi
from skimage import feature
from skimage.morphology import skeletonize
import pdb
def gkern(kernlen=21, nsig=3):
        """Returns a 2D Gaussian kernel array."""

        interval = (2*nsig+1.)/(kernlen)
        x = np.linspace(-nsig-interval/2., nsig+interval/2., kernlen+1)
        kern1d = np.diff(st.norm.cdf(x))
        kernel_raw = np.sqrt(np.outer(kern1d, kern1d))
        kernel = kernel_raw/np.max(kernel_raw)#.sum()
        return kernel
def point_gaussian(k_w, k_h, sigma_w, sigma_h):
    return gkern(k_w, sigma_w)

def map_gaussian(im, k_w, k_h, sigma_w, sigma_h):
    if im.ndim == 3:
        h,w,_ = im.shape
    elif im.ndim == 2:
        h,w = im.shape
    else:
        print "Unknow im format."
        return None

    k_w_r = (k_w-1)/2
    k_h_r = (k_h-1)/2



    point_gs = point_gaussian(k_w, k_h, sigma_w, sigma_h)
    map_gs = np.zeros((h,w),dtype=np.float)

    occ_loc = np.where(im[:,:,0] == 0)
    for i in range(len(occ_loc[1])):
        loc_i = [occ_loc[0][i],occ_loc[1][i]]
#map_gs[loc_i[0]-k_h_r: loc_i[0]+k_h_r+1, loc_i[1]-k_w_r: loc_i[1]+k_w_r+1]  = np.maximum(map_gs[loc_i[0]-k_h_r: loc_i[0]+k_h_r+1, loc_i[1]-k_w_r: loc_i[1]+k_w_r+1], point_gs)

        #map_gs bound
        lby, lbx = max(0,loc_i[0]-k_h_r), max(0,loc_i[1]-k_w_r)
        uby, ubx = min(h-1,loc_i[0]+k_h_r), min(w-1,loc_i[1]+k_w_r)
        #kernel bound
        k_lby, k_lbx = -min(0,loc_i[0]-k_h_r), -min(0,loc_i[1]-k_w_r)
        k_uby, k_ubx = k_h + (h-1 - max(h-1, loc_i[0]+k_h_r)), k_w + (w-1 - max(w-1, loc_i[1]+k_w_r))
        #maximum on pixel
        map_gs[lby:uby+1, lbx:ubx+1] = np.maximum(map_gs[lby:uby+1, lbx:ubx+1], point_gs[k_lby:k_uby, k_lbx:k_ubx])

    return map_gs 


def preprocess(im_path):
    im = imread(im_path)
    map_gs = map_gaussian(im, 455,455,5,5)
    return im, map_gs



def edgeprocess(gs):
    #laplacian
    gray_lap = cv2.Laplacian(gs*255,cv2.CV_64F, ksize=5)    
    dst = cv2.convertScaleAbs(gray_lap)
    return dst

def binarize(dst, th):
    res = np.zeros(dst.shape, dtype=np.float)
    res[np.where(dst > th)] = 1
    return res


Tidu = (np.array([0,1,1,1]),np.array([1,0,1,2]))
Tidd = (np.array([1,1,1,2]),np.array([0,1,2,1]))
Tidl = (np.array([0,1,1,2]),np.array([1,0,1,1]))
Tidr = (np.array([0,1,1,2]),np.array([1,1,2,1]))
def remove_Tcenter(skele):
    '''
        it is possible to have T shape in skeleton, we should remove the center of it
    '''
    w,h = skele.shape
    for i in range(1,w-1):
        for j in range(1,h-1):
            if skele[i,j]:
                patch = skele[i-1:i+2,j-1:j+2].copy()
                if np.sum(patch[Tidu]) == 4 or \
                    np.sum(patch[Tidd]) == 4 or \
                    np.sum(patch[Tidl]) == 4 or \
                    np.sum(patch[Tidr]) == 4: 
                    skele[i,j] = False
    #i = 0
    for j in range(1,h-1):
        if skele[0,j]:
            if np.sum(skele[0,j-1:j+2]) + skele[1,j] == 4:
                skele[0,j] = False
    #i = w-1
    for j in range(1,h-1):
        if skele[w-1,j]:
            if np.sum(skele[w-1,j-1:j+2]) + skele[w-2,j] == 4:
                skele[w-1,j] = False
    #j = 0
    for i in range(1,w-1):
        if skele[i,0]:
            if np.sum(skele[i-1:i+2,0]) + skele[i,1] == 4:
                skele[i,0] = False
    #j = h-1
    for i in range(1,w-1):
        if skele[i,h-1]:
            if np.sum(skele[i-1:i+2,h-1]) + skele[i,h-2] == 4:
                skele[i,h-1] = False
 
    #i=0,j=0
    if skele[0,0]:
        if skele[0,0] + skele[0,1] + skele[1,0] == 3:
            skele[0,0] = False
    #i=0,j=h-1
    if skele[0,h-1]:
        if skele[0,h-2] + skele[0,h-1] + skele[1,h-1] == 3:
            skele[0,h-1] = False
    #i=w-1,j=0
    if skele[w-1,0]:
        if skele[w-2,0] + skele[w-1,0] + skele[w-1,1] == 3:
            skele[w-1,0] = False
    #i=w-1,j=h-1
    if skele[w-1,h-1]:
        if skele[w-1,h-2] + skele[w-1,h-1] + skele[w-2,h-1] == 3:
            skele[w-1,h-1] = False
    return skele

def im2skeleton(im_path, give_bin = False):
    #1. gaussian map
    im, map_gs = preprocess(im_path)
    #2. edge detection
    dst = edgeprocess(map_gs)
    #3. binarize
    res = binarize(dst,10)
    #4. suppress
    skeleton = skeletonize(res) 
    #5. remove T shape center
    skeleton = remove_Tcenter(skeleton)

#plt.imshow(skeleton)
#plt.show()   
#   print np.max(skeleton)
    if give_bin:
        binary_wall = im[:,:,0] == 0
        return im, map_gs, skeleton, binary_wall
    return im, map_gs, skeleton

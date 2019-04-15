#! /usr/bin/env python

import numpy as np
from scipy.misc import imread, imresize
import matplotlib.pyplot as plt
import cv2
import scipy.io
import scipy.stats as st
from scipy import ndimage as ndi
from skimage import feature
from skimage.morphology import skeletonize
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
def _point_gaussian(k_w, k_h, sigma_w, sigma_h):
    point_gs = np.zeros((k_h,k_w),dtype = np.float)
    h_r = (k_h-1)/2
    w_r = (k_w-1)/2
    hp,wp = np.mgrid[-h_r:k_h-h_r, -w_r:k_w-w_r]
    print hp[h_r,w_r], wp[h_r,w_r]
#point_gs = np.exp(-(np.abs(hp))/(2*sigma_h**2) - (np.abs(wp))/(2*sigma_w**2))
#    point_gs = 1/np.abs(wp*hp/(sigma_h*sigma_w))
    point_gs = np.exp(-(hp**2)/(2*sigma_h**2) - (wp**2)/(2*sigma_w**2))

    point_gs/=(point_gs[h_r,w_r])
    plt.imshow(point_gs)
    plt.show()
    plt.imshow(hp)
    plt.show()
    plt.imshow(wp)
    plt.show()

    return point_gs

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
        #map_gs bound
        lby, lbx = max(0,loc_i[0]-k_h_r), max(0,loc_i[1]-k_w_r)
        uby, ubx = min(h-1,loc_i[0]+k_h_r), min(w-1,loc_i[1]+k_w_r)
        #kernel bound
        k_lby, k_lbx = -min(0,loc_i[0]-k_h_r), -min(0,loc_i[1]-k_w_r)
        k_uby, k_ubx = k_h + (h-1 - max(h-1, loc_i[0]+k_h_r)), k_w + (w-1 - max(w-1, loc_i[1]+k_w_r))
        #maximum on pixel
        map_gs[lby:uby+1, lbx:ubx+1] = np.maximum(map_gs[lby:uby+1, lbx:ubx+1], point_gs[k_lby:k_uby, k_lbx:k_ubx])
    return map_gs 

def grad_gau(map_gs):
    gx,gy = np.gradient(map_gs)
    return np.exp(np.abs(gx)**2 +np.abs(gy)**2)
def sobel_gau(gs):
    gx = cv2.Sobel(gs*255,cv2.CV_64F,1,0,ksize=5)
    gy = cv2.Sobel(gs*255,cv2.CV_64F,0,1,ksize=5)

    return (np.abs(gx)**2 +np.abs(gy)**2)


def preprocess(im_path):
    im = imread(im_path)
    map_gs = map_gaussian(im, 455,455,5,5)


#    plt.imshow(map_gs, vmin=0, vmax=0.5)
#    plt.show()


    gau = grad_gau(map_gs)
    plt.imshow(gau)
    plt.colorbar()
    plt.show()
    '''
    gau2 = grad_gau(gau)
    plt.imshow(gau2)
    plt.colorbar()
    plt.show()
    '''


#plt.imshow(gau*100+ (255-im[:,:,0])/255)
#   plt.show()
    return gau, map_gs



def edgeprocess(im_path):
    gau, gs = preprocess(im_path)

    #canny
    edges1 = feature.canny(gs*255,sigma=4.)
    plt.imshow(edges1)
    plt.show()
    #laplacian
    gray_lap = cv2.Laplacian(gs*255,cv2.CV_64F, ksize=5)    
    dst = cv2.convertScaleAbs(gray_lap)
    plt.imshow(dst)
    plt.show()


    #laplacian of gradient
    gray_lap = cv2.Laplacian((gau-1)*10000*255,cv2.CV_64F)
    dst2 = cv2.convertScaleAbs(gray_lap)
    plt.imshow(dst2)
    plt.show()
    '''
    #sobel
    sobel = sobel_gau(gs)
    plt.imshow(sobel)
    plt.show()


    #canny of laplacian
    edges2 = feature.canny(dst*255,sigma=0.0)
    plt.imshow(edges2)
    plt.show()
    '''
    return dst, dst2
def binarize(dst, th):
    res = np.zeros(dst.shape, dtype=np.float)
    res[np.where(dst > th)] = 1
    return res

def topo(im_path):
    dst, dst2 = edgeprocess(im_path)
    print np.max(dst), np.max(dst2)
    res = binarize(dst,10)
    plt.imshow(res)
    plt.show()

    res2 = binarize(dst2, 145)
    plt.imshow(res2)
    plt.show()

    #suppress before binarize(not work)
    #suppressed = suppression(dst)
    #res3 = binarize(suppressed,10)
    #plt.imshow(res3)
    #plt.show()

    #suppress after binarize
    skeleton = skeletonize(res) 
    plt.imshow(skeleton)
    plt.show()   
    print np.max(skeleton)
    return skeleton

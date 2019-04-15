import numpy as np
import cv2
import others_draw
import matplotlib.pyplot as plt
import pdb
def arrayToKeyPoint(arr,regionSize, angle):
    '''
    input:
        arr[n,2] [h,w]
        regionSize
    output:
        KeyPoint[n]
    '''
    return [cv2.KeyPoint(arr[i][1],arr[i][0],regionSize, angle[i]) for i in range(len(arr))]


def represent(gray, kps, patch_size=128):
    '''
    input:
        gray
        kps[n]
    output:
        feature[n,k], k is the dimension of feature. 
    '''
    orb = cv2.ORB_create(patchSize=patch_size)
    return orb.compute(gray, kps)[1]


def matching(des1, des2):
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    return bf.match(des1, des2)

def arr2ori(gray, arr):
    '''
    input:
        gray
        arr[n,2][h,w]
    '''
    gx,gy = np.gradient(gray)
    ori = []
    rate = 360/2/np.pi
    for pos in arr:
        ori.append(np.arctan2( gy[pos[0],pos[1]],  gx[pos[0],pos[1]]  ) * rate)
    return ori

def DrawMatching(gray1, gray2, arr1, arr2,topo1,topo2, regionSize, num, patch_size=128):
    '''
    input:

    output:
        matched id pair from arr1 to arr2
    '''

    angle1 = arr2ori(gray1, arr1)
    angle2 = arr2ori(gray2, arr2)

    kps1 = arrayToKeyPoint(arr1, regionSize, angle1)
    kps2 = arrayToKeyPoint(arr2, regionSize, angle2)

    feature1 = represent(gray1, kps1, patch_size=patch_size)
    feature2 = represent(gray2, kps2, patch_size=patch_size)

#pdb.set_trace()

    matches = matching(feature1, feature2)

    # Sort them in the order of their distance.
    matches = sorted(matches, key = lambda x:x.distance)

    #draw 10 matched 
#img3 = others_draw.drawMatches(gray1,kps1,gray2,kps2,matches[:30])#, flags=2)
    img3 = others_draw.drawMatches(topo1,kps1,topo2,kps2,matches[:num])#, flags=2)

    plt.imshow(img3),plt.show()

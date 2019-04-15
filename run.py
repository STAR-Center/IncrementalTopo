from gp_topo.im2skeleton import im2skeleton
from gp_topo.skeleton2topoMap import skeleton2topoMap, drawMap, drawtoposkele
from gp_topo.reprsntAndMatching import DrawMatching

import numpy as np
import matplotlib.pyplot as plt
import scipy.io
import pdb
import os
from scipy.misc import imsave
if __name__ == '__main__':
    print 'Im to Skeleton...'
#intel, office
    im_folder = ["./test_im/intel","./test_im/office","./test_im/a_scan","./frames"][3]
    gray1, gau1, skeleton = im2skeleton(im_folder+'/clean.png')
    graph = skeleton2topoMap(skeleton)
    fig = plt.figure(frameon=False)
    ax = plt.Axes(fig, [0., 0., 1., 1.])
    ax.set_axis_off()
    fig.add_axes(ax)
    #remove the white border, the only worked method found by adding the below two line before show out:>
    ax.xaxis.set_major_locator(plt.NullLocator())
    ax.yaxis.set_major_locator(plt.NullLocator())

    ax.imshow(gau1, cmap='GnBu')
    fig.savefig(im_folder+'/pure_dist.png', format='png', dpi=500, bbox_inches='tight',pad_inches=0)

    ax.xaxis.set_major_locator(plt.NullLocator())
    ax.yaxis.set_major_locator(plt.NullLocator())
    ax.imshow(skeleton+(1-gray1[:,:,0]/255)*2, cmap='GnBu')
    fig.savefig(im_folder+'/pure_skele.png', format='png', dpi=500, bbox_inches='tight',pad_inches=0 )
 
    ax.xaxis.set_major_locator(plt.NullLocator())
    ax.yaxis.set_major_locator(plt.NullLocator())

    drawMap(graph, skeleton, ax = ax)
    fig.savefig(im_folder+'/pure_topo.png', format='png', dpi=500, bbox_inches='tight',pad_inches=0)

    fig = plt.figure(frameon=False)
    ax = plt.Axes(fig, [0., 0., 1., 1.])
    ax.set_axis_off()
    fig.add_axes(ax)
    ax.xaxis.set_major_locator(plt.NullLocator())
    ax.yaxis.set_major_locator(plt.NullLocator())
    drawtoposkele(graph, skeleton+(1-gray1[:,:,0]/255)*2, ax=ax)
    fig.savefig(im_folder+'/skele_topo.png', format='png', dpi=500, bbox_inches='tight',pad_inches=0)

    imsave(im_folder+"/out.png",255-skeleton*127+gray1[:,:,0])



    '''
        store the vertex location
    '''
    h,w = skeleton.shape
    v_loc_lst = []
    def exist_in_list(v_in):
        for v in v_loc_lst:
            dist = np.sum((v-v_in)**2)
            if dist < 0.0000000000000001:
                return True
        return False
    for ed in graph.edges:
        if ed.parentEdgeId == -1:
            v1 = np.array((ed.vertices[0]%w,ed.vertices[0]/w)).reshape((1,2))
            v2 = np.array((ed.vertices[-1]%w,ed.vertices[-1]/w)).reshape((1,2))
            if not exist_in_list(v1):
                v_loc_lst.append(v1)
            if not exist_in_list(v2):
                v_loc_lst.append(v2)
    v_loc_lst_mat = np.concatenate(v_loc_lst, axis=0)


    scipy.io.savemat(im_folder+"/gau_locs.mat", {'pts':v_loc_lst_mat})




    '''
    matname = './tmp.mat'
    if os.path.exists(matname):
        mat = scipy.io.loadmat(matname)
        gray1 = mat['gray1']
        gray2 = mat['gray2']
        arr1 = mat['arr1']
        arr2 = mat['arr2']
        gau1 = mat['gau1']
        gau2 = mat['gau2']
    else:
        print 'Im to Skeleton...'
        gray1, gau1, skeleton = im2skeleton('im/test.png')
        print 'Skeleton to topoGraph...'
        graph = skeleton2topoMap(skeleton)
#   print 'Draw topoGraph...'
        drawMap(graph, skeleton)


        #second im
        print 'Im to Skeleton...'
        gray2, gau2, skeleton2 = im2skeleton('im/testrt.png')
        print 'Skeleton to topoGraph...'
        graph2 = skeleton2topoMap(skeleton2)
#print 'Draw topoGraph...'
#drawMap(graph2, skeleton2)

        #matching here
        arr1 = graph.getTopoVertex()
        arr2 = graph2.getTopoVertex()

        scipy.io.savemat(matname, {'gray1':gray1, 'gray2':gray2, 'arr1':arr1, 'arr2': arr2, 'gau1':gau1, 'gau2':gau2})

    DrawMatching((gau1*255).astype(np.uint8), (gau2*255).astype(np.uint8), arr1, arr2, gray1[:,:,1], gray2[:,:,1], 5,10,patch_size=64)
    '''

#!/usr/bin/env python
'''
this code is for offline_gp_topograph
'''
#import fast_gpom as mcgpom
import gp_topo.increment_topo as mcgpom
import rospy
import rosbag

import numpy as np
import matplotlib.pyplot as plt
import matplotlib

from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, PoseArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, MapMetaData
from nav_msgs.srv import GetMap
from tf.transformations import euler_from_quaternion

from scipy.misc import imsave, imread
import scipy.io
import sys
import time
import pdb

from gp_topo.skeleton2topoMap import drawMap

'''
for each frame:
    get current posi, ori, x, y
    get local frame
    update on the global frame
    if visualize_iter:
        extract valley
        thinning
'''


#1. for each frame, build the local gaussian process map [mu, sigma, prob]
    #1.1 read one frame, get current posi, ori, x,y
    
    #1.2 gaussian process
        #1.2.1 for the first frame, do maximum likelyhood to estimate hyperparam

        #1.2.2 predict on top of kernel. achieve mu and sigma

        #1.2.3 logistic regression to calculate prob

    #1.3. fuse two frame whenever update

def plot_current_map():
    font = {'weight': 'normal',
            'size': 20}

    matplotlib.rc('font', **font)

    plt.figure("GP Occupancy Map")
    plt.clf()
    plt.pcolor(gp_map.X, gp_map.Y, gp_map.map, vmin=0, vmax=1)
    plt.colorbar()
    '''
    if not gp_map.current_pose is None:
        plt.quiver(gp_map.current_pose[0], gp_map.current_pose[1], 1. * np.cos(gp_map.current_pose[2]),
                   1. * np.sin(gp_map.current_pose[2]), angles='xy', scale_units='xy', scale=1,
                   edgecolors='m', pivot='mid', facecolor='none', linewidth=1, width=0.001, headwidth=400, headlength=800)
    plt.axis('equal')
    '''
    '''
    plt.figure("GP Frontier Map")
    plt.clf()
    plt.pcolor(gp_map.X, gp_map.Y, gp_map.frontier_map, vmin=0, vmax=1)
    plt.quiver(gp_map.current_pose[0], gp_map.current_pose[1], 1. * np.cos(gp_map.current_pose[2]),
               1. * np.sin(gp_map.current_pose[2]), angles='xy', scale_units='xy', scale=1,
               edgecolors='m', pivot='mid', facecolor='none', linewidth=1, width=0.001, headwidth=400, headlength=800)
    if not gp_map.expl_goal is None:
        plt.plot(gp_map.expl_goal[:, 0], gp_map.expl_goal[:, 1], linestyle='-.', c='m', marker='+', markersize=14)
    plt.axis('equal')
    '''
    plt.draw()
    plt.pause(.1)



def feed_input():
    #TODO
    global gp_com_msg

    # reading dataset(bag file)
    file_name = './bag/stdr_data.bag'
    bag = rosbag.Bag(file_name)

    # We want to get scan and pose, so we should do mapping after 2 iterations.
    mapping_flag = False

    # 2 msgs
    scan_msg = None
    pose_msg = None

    count = 0

    for topic, msg, t in bag.read_messages(topics=['/slam_out_pose', '/robot0/laser_0']):
        if topic == '/robot0/laser_0':
            scan_msg = msg
#scan_msg.ranges = np.asarray(scan_msg.ranges)
#gp_map.set_scan(scan_msg)
            continue
        elif topic == '/slam_out_pose':
            pose_msg = msg
            '''
            q = [pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z,
                 pose_msg.pose.orientation.w]
            angles = euler_from_quaternion(q)
            pose = np.array([pose_msg.pose.position.x, pose_msg.pose.position.y, angles[2]])
#gp_map.current_pose = pose
            '''
            # if scan_msg is None:
            #     continue

        yield{"scan":scan_msg, "slam_out_pose":pose_msg}



def publish_map_image():
    grid_msg = OccupancyGrid()
    grid_msg.header.stamp = rospy.Time.now()
    grid_msg.header.frame_id = "map"

    grid_msg.info.resolution = gp_map.map_res
    grid_msg.info.width = gp_map.width
    grid_msg.info.height = gp_map.height

    grid_msg.info.origin = Pose(Point(gp_map.map_limit[0],gp_map.map_limit[2], 0),
                                Quaternion(0, 0, 0, 1))

    flat_grid = gp_map.topo_map.copy()
    flat_grid = flat_grid.reshape((gp_map.map_size,))

    flat_grid[np.where(flat_grid<0.4)] = 0
    flat_grid[np.where(flat_grid > 0.4)] = 1

    #flat_grid = gp_map.threshold(flat_grid)

    flat_grid = flat_grid.astype(int)

    grid_msg.data = flat_grid.tolist()

    occ_map_pub.publish(grid_msg)

if __name__ == '__main__':
    gp_map = mcgpom.GPMap() 
#    rospy.init_node('gp_occ_map', anonymous=True)
    # publish map
#    occ_map_pub = rospy.Publisher('map', OccupancyGrid, queue_size=10, latch=True)


    #gen
    count = 1
    gen = feed_input()

    #time record
    dist_lp_lst = []
    skele_lp_lst = []
    topo_lp_lst = []

    for feeds in gen:
        if(count > 3040):
            break

        #==============distance map loop===============
        dist_st = time.time()

        #fetch data
        scan_msg,slam_out_pose_msg = feeds['scan'],feeds['slam_out_pose']
        #data process
        #scan
        # rospy.loginfo(scan_msg)
        scan_msg.ranges = np.array(scan_msg.ranges)
        gp_map.set_scan(scan_msg)

        #pose
        q = [slam_out_pose_msg.pose.orientation.x, slam_out_pose_msg.pose.orientation.y, slam_out_pose_msg.pose.orientation.z,
             slam_out_pose_msg.pose.orientation.w]
        angles = euler_from_quaternion(q)
        pose = np.array([slam_out_pose_msg.pose.position.x, slam_out_pose_msg.pose.position.y, angles[2]])
        gp_map.current_pose = pose


        #build map
        gp_map.build_map()

        dist_ed = time.time()
        dist_lp_lst.append(dist_ed-dist_st)

        #==============================================
        #publish
        #===============skeleton loop==================
        #update skeleton  
        if(count %20 == 0):

            skele_st = time.time()

            sys.stdout.write("\nDraw skeleton count id %d"%count)
            sys.stdout.flush()
            gp_map.extract_and_thin()

            skele_m1 = time.time()

            plt.figure(1)
            plt.clf()
            plt.imshow(2*gp_map.topo_map+gp_map.cur_global_occ_map+gp_map.local_bin_mask, cmap='GnBu')
            #save image for paper
            '''
            ax.xaxis.set_major_locator(plt.NullLocator())
            ax.yaxis.set_major_locator(plt.NullLocator())
            ax.imshow(2*gp_map.topo_map+gp_map.cur_global_occ_map+gp_map.local_bin_mask, cmap='GnBu')
            fig.savefig("./frames/skele/"+str(count)+"_skele.png", format='png', dpi=500, bbox_inches='tight',pad_inches=0)
            ''' 

            skele_m2 = time.time()
            gp_map.reset_skele_update_mask()
            skele_ed = time.time()
            skele_lp_lst.append(skele_ed - skele_st - (skele_m2-skele_m1))

#imsave( './frames/'+str(count)+'.png', 255-127*(gp_map.topo_map+gp_map.cur_global_occ_map*2) )
            plt.title("Skeleton Til Frame %d / 3040"%count)
            plt.draw()

#plt.draw()
            plt.pause(0.001)

        #==============================================

        #================topo loop=====================
        #update topograph
        if(count%80 == 0):
            topo_st = time.time()

            sys.stdout.write("\nDraw Graph count id %d"%count)
            sys.stdout.flush()
            gp_map.update_graph()

            topo_m1 = time.time()

            plt.figure(2)
            plt.clf()
            plt.gca().xaxis.set_major_locator(plt.NullLocator())
            plt.gca().yaxis.set_major_locator(plt.NullLocator())
            drawMap(gp_map.global_graph, gp_map.topo_map, mask=gp_map.local_skeleton_mask)
            #save image for paper
            plt.savefig("./frames/topo/"+str(count)+"_topo.png",format='png', dpi=500, bbox_inches='tight',pad_inches=0)

            topo_m2 = time.time()

            gp_map.reset_topo_update_mask()        
            topo_ed = time.time()
            topo_lp_lst.append(topo_ed - topo_st - (topo_m2-topo_m1))


            plt.title("TopoMap Til Frame %d / 3040"%count)
            plt.draw()
            plt.pause(0.001)
        #==============================================


        count += 1
    print(count)
    fig = plt.figure(frameon=False)
    ax = plt.Axes(fig, [0., 0., 1., 1.])
    ax.set_axis_off()
    fig.add_axes(ax)

    ax.xaxis.set_major_locator(plt.NullLocator())
    ax.yaxis.set_major_locator(plt.NullLocator())
    ax.imshow(gp_map.cur_global_heat_map,cmap='GnBu')
    fig.savefig("./frames/incre_dist.png", format='png', dpi=500, bbox_inches='tight',pad_inches=0)

    ax.xaxis.set_major_locator(plt.NullLocator())
    ax.yaxis.set_major_locator(plt.NullLocator())
    ax.imshow(gp_map.topo_map+gp_map.cur_global_occ_map*2,cmap='GnBu')
    fig.savefig("./frames/incre_skeleton.png", format='png', dpi=500, bbox_inches='tight',pad_inches=0)
    
    ax.xaxis.set_major_locator(plt.NullLocator())
    ax.yaxis.set_major_locator(plt.NullLocator())
    drawMap(gp_map.global_graph, gp_map.topo_map, ax) 
    fig.savefig("./frames/incre_topo.png", format='png', dpi=500, bbox_inches='tight',pad_inches=0)


    imsave('./frames/incre_out.png', 255-gp_map.topo_map*127+(1-gp_map.cur_global_occ_map)*255)


    print('\nsave map!')
    scipy.io.savemat("./frames/final.mat",{"occ_map":gp_map.cur_global_occ_map, "dist_map":gp_map.cur_global_heat_map, "skeleton":gp_map.topo_map, "dist_time":np.array(dist_lp_lst), "skele_time":np.array(skele_lp_lst), "topo_time":np.array(topo_lp_lst)})
    im3 = np.stack([gp_map.cur_global_occ_map,gp_map.cur_global_occ_map,gp_map.cur_global_occ_map],2)
    imsave("./frames/clean.png", 255-im3*255)



    '''
        store the vertex location
    '''
    h,w = gp_map.topo_map.shape
    v_loc_lst = []
    def exist_in_list(v_in):
        for v in v_loc_lst:
            dist = np.sum((v-v_in)**2)
            if dist < 0.0000000000000001:
                return True
        return False
    for ed in gp_map.global_graph.edges:
        if ed.parentEdgeId == -1:
            v1 = np.array((ed.vertices[0]%w,ed.vertices[0]/w)).reshape((1,2))
            v2 = np.array((ed.vertices[-1]%w,ed.vertices[-1]/w)).reshape((1,2))
            if not exist_in_list(v1):
                    v_loc_lst.append(v1)
            if not exist_in_list(v2):
                    v_loc_lst.append(v2)
    v_loc_lst_mat = np.concatenate(v_loc_lst, axis=0)


    scipy.io.savemat("./frames/incre_gau_locs.mat", {'pts':v_loc_lst_mat})






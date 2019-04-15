
import numpy as np
#from scipy.cluster.vq import *
#import pyGPs
#import gpflow
import copy
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, PoseArray
from scipy.stats import norm
#from utils.tools import graph_in_poly
from skimage.draw import polygon
import time
import matplotlib.pyplot as plt

from im2skeleton import point_gaussian, edgeprocess, binarize, remove_Tcenter
from skeleton2topoMap import skeleton2topoMap, drawMap 
from skimage.morphology import thin

import pdb

reso = 0.05#0. as the base
expand_coeff = 1.5
ceoff = int(expand_coeff * 0.1 / reso)


def local_heatmap(w,h,k_w,k_h,sigma_w,sigma_h, occ_pix_loc):
    point_gs = point_gaussian(k_w, k_h, sigma_w, sigma_h)
    map_gs = np.zeros((h,w),dtype=np.float)
    k_w_r = (k_w-1)/2
    k_h_r = (k_h-1)/2
#occ_pix_loc = np.where(im[:,:,0] == 0)
    occ_map = np.zeros((h,w),dtype=np.bool) 
    for i in range(occ_pix_loc.shape[0]):
        loc_i = [occ_pix_loc[i][1].astype(int),occ_pix_loc[i][0].astype(int)]
        #map_gs bound
        lby, lbx = max(0,loc_i[0]-k_h_r), max(0,loc_i[1]-k_w_r)
        uby, ubx = min(h-1,loc_i[0]+k_h_r), min(w-1,loc_i[1]+k_w_r)
        #kernel bound
        k_lby, k_lbx = -min(0,loc_i[0]-k_h_r), -min(0,loc_i[1]-k_w_r)
        k_uby, k_ubx = k_h + (h-1 - max(h-1, loc_i[0]+k_h_r)), k_w + (w-1 - max(w-1, loc_i[1]+k_w_r))
        map_gs[lby:uby+1, lbx:ubx+1] = np.maximum(map_gs[lby:uby+1, lbx:ubx+1], point_gs[k_lby:k_uby, k_lbx:k_ubx])


        occ_map[loc_i[0], loc_i[1]] = True

# map_gs[loc_i[0]-k_h_r: loc_i[0]+k_h_r+1, loc_i[1]-k_w_r: loc_i[1]+k_w_r+1]  = np.maximum(map_gs[loc_i[0]-k_h_r: loc_i[0]+k_h_r+1, loc_i[1]-k_w_r: loc_i[1]+k_w_r+1], point_gs)
    return map_gs, occ_map
def get_parnetEdge_number(edges):
    count = 0
    for e in edges:
        if e.parentEdgeId == -1:
            count+=1
    return count

class GPMap():
    def __init__(self, mcmc = False):
        #mode
        #map property
        self.width = 300 * ceoff
        self.height = 300 * ceoff
        self.map_size = self.width * self.height
        self.map_limit = [x*expand_coeff for x in [-15.0, 15.0, -15.0, 15.0]]# * expand_coeff
        self.map_res = (self.map_limit[1] - self.map_limit[0]) / self.width

        self.td_res = 0.25
        #map
        self.map = .5 * np.ones((self.width, self.height), dtype=np.float)
        self.map_mu = .5 * np.ones((self.width, self.height), dtype=np.float)
        self.map_sigma = np.ones((self.width, self.height), dtype=np.float)
        #data
        self.x = np.zeros((1, 2))
        self.y = np.zeros((1, 1))
        self.X, self.Y = np.meshgrid(np.linspace(self.map_limit[0], self.map_limit[1], self.width),
                                     np.linspace(self.map_limit[2], self.map_limit[3], self.height))
        self.Xs = np.vstack([self.X.reshape(self.Y.size), self.Y.reshape(self.Y.size)]).T

        self.first_frame = True
        #oneline gpom stuff
        self.first_link = None
        self.second_link = None
        self.third_link = None


        #local map
        self.local_width = 80 * ceoff
        self.local_height = 80 * ceoff
        self.local_map_size = self.local_width * self.local_height
        self.local_map_limit =[x * expand_coeff for x in [-4.0, 4.0, -4.0, 4.0] ]
        self.local_map = .5 * np.ones((self.local_width, self.local_height), dtype=np.float)
        self.local_map_mu = .5 * np.ones((self.local_width, self.local_height), dtype=np.float)
        self.local_map_sigma = np.ones((self.local_width, self.local_height), dtype=np.float)
        self.local_X, self.local_Y = np.meshgrid(
            np.linspace(self.local_map_limit[0], self.local_map_limit[1], self.local_width),
            np.linspace(self.local_map_limit[2], self.local_map_limit[3], self.local_height))
        self.xs = np.vstack([self.local_X.reshape(self.local_Y.size), self.local_Y.reshape(self.local_Y.size)]).T
        #robot property
        self.current_pose = None

        #time recording
        self.timeTable = np.zeros((100))
        self.times = 0
        
        #gaussian heatmap kernel
        self.k_w = 445
        self.k_h = 445
        self.sigma_w = 5
        self.sigma_h = 5
        self.cur_local_heat_map = None
        self.cur_global_heat_map = np.zeros((self.height, self.width), dtype = np.float)
        self.cur_local_occ_map = None
        self.cur_global_occ_map = np.zeros((self.height, self.width), dtype = np.bool)
        self.topo_map = np.zeros((self.height, self.width), dtype = np.bool)
        self.local_skeleton = np.zeros((self.height, self.width), dtype = np.bool)

        self.local_canvas_mask = np.zeros(self.topo_map.shape, dtype=np.bool)
        self.canvas_width = 5
        self.local_bin_mask = np.zeros(self.topo_map.shape, dtype=np.bool)

        self.local_skeleton_mask = np.zeros(self.topo_map.shape, dtype=np.bool)

        self.global_graph = skeleton2topoMap(self.topo_map)

    def set_scan(self, scan):
        self.scan = scan

    def logistic_reg(self, mu, sigma, alpha=100, beta=0):
        prob = norm.cdf((alpha*mu+beta)/(1+(alpha*sigma)**2))
#amap = copy.deepcopy(np.reshape(prob, (awidth, aheight)))
#        amap_mu = copy.deepcopy(np.reshape(mu, (awidth, aheight)))
#        asigma = copy.deepcopy(np.reshape(sigma, (awidth, height)))
        return prob

    def transform2global(self):
        self.y = np.ones((1, 1))
        self.x = np.array(-1)
        for i in range(self.scan.ranges.size):
            if self.scan.ranges[i] != np.Inf:
                x1 = self.current_pose[0] + self.scan.ranges[i] * np.cos(
                    self.current_pose[2] + (self.scan.angle_min + (i + 1) * self.scan.angle_increment))
                x2 = self.current_pose[1] + self.scan.ranges[i] * np.sin(
                    self.current_pose[2] + (self.scan.angle_min + (i + 1) * self.scan.angle_increment))
                if self.x.size == 1:
                    self.x = np.array([[x1, x2]])
                else:
                    self.x = np.vstack([self.x, [x1, x2]])
                    self.y = np.vstack([self.y, [1.]])
        #outline
        self.first_link = (self.x.copy(), self.y.copy())

    def update_map(self):
        '''
            put the local frame into the global frame
        '''
        ix = int(np.round(self.current_pose[0] / self.map_res) + (self.width / 2) - (self.local_width / 2))
        iy = int(np.round(self.current_pose[1] / self.map_res) + (self.height / 2) - (self.local_height / 2))
        #update heatmap
        self.cur_global_heat_map[iy:iy+self.cur_local_heat_map.shape[0] , ix:ix+self.cur_local_heat_map.shape[1]] = np.maximum(self.cur_local_heat_map, self.cur_global_heat_map[iy:iy+self.cur_local_heat_map.shape[0] , ix:ix+self.cur_local_heat_map.shape[1]])
        #update gridmap
        self.cur_global_occ_map[iy:iy+self.cur_local_occ_map.shape[0] , ix:ix+self.cur_local_occ_map.shape[1]] = (self.cur_local_occ_map | self.cur_global_occ_map[iy:iy+self.cur_local_occ_map.shape[0] , ix:ix+self.cur_local_occ_map.shape[1]])
        #update the mask
#self.local_bin_mask[iy:iy+self.cur_local_occ_map.shape[0] , ix:ix+self.cur_local_occ_map.    shape[1]] = True
        #to avoid border detected in bin, we avoid 4 pixel
        self.local_bin_mask[iy+4:iy+self.cur_local_occ_map.shape[0]-4 , ix+4:ix+self.cur_local_occ_map.shape[1]-4] = True
        self.local_canvas_mask[iy-self.canvas_width:iy+self.cur_local_occ_map.shape[0]+self.canvas_width , ix-self.canvas_width:ix+self.cur_local_occ_map.shape[1]+self.canvas_width] = True



    def build_map(self):
        '''
            1. get pose, point location on pixel
            2. build local heat map
            3. update to the global
            4. visualization if in certain iteration
        '''
        self.times += 1
        #get the true position of each range point & get the occupy observation
        #0
        self.transform2global()
        #1

        upper_left = (self.local_map_limit[0] + self.current_pose[0],self.local_map_limit[2] + self.current_pose[1])
        dx = 2*self.local_map_limit[1] / self.local_width
        dy = 2*self.local_map_limit[3] / self.local_height
        plt_1link = self.first_link[0].copy()
#plt_2link = self.second_link[0].copy()
        plt_1link[:,0] -= upper_left[0] 
        plt_1link[:,1] -= upper_left[1] 
#        plt_2link[:,0] -= upper_left[0] 
#        plt_2link[:,1] -= upper_left[1] 
        #plt_1link is the location on image 
        plt_1link[:,0] /= dx 
        plt_1link[:,1] /= dy 
        #
        self.cur_local_heat_map, self.cur_local_occ_map = local_heatmap(self.local_width, self.local_height, self.k_w, self.k_h, self.sigma_w, self.sigma_h, plt_1link)

        #build model
        self.update_map()

    def extract_and_thin(self):
        #2. edge detection
        dst = edgeprocess(self.cur_global_heat_map)
        #3. binarize
        res = binarize(dst,10)
    
        canvas = np.zeros(self.topo_map.shape)
        canvas[self.local_canvas_mask] = self.topo_map[self.local_canvas_mask]
        canvas[self.local_bin_mask] = res[self.local_bin_mask] 

        #4. suppress
        self.local_skeleton = thin(canvas) 
        #5. remove T shape center 
        self.local_skeleton = remove_Tcenter(self.local_skeleton)

        self.topo_map[self.local_bin_mask]  = self.local_skeleton[self.local_bin_mask] 
        #5. update local_skeleton_mask for the later topo_graph update
        self.local_skeleton_mask = np.logical_or(self.local_skeleton_mask, self.local_bin_mask)
    
    def reset_skele_update_mask(self):
        #6. reset the local mask
        self.local_canvas_mask = np.zeros(self.topo_map.shape,dtype=bool)
        self.local_bin_mask = np.zeros(self.topo_map.shape,dtype=bool)

    
    def update_graph(self):
        #keep one local_skeleton_mask
        feed_skeleton = np.zeros(self.topo_map.shape)
        feed_skeleton[self.local_skeleton_mask] = self.topo_map[self.local_skeleton_mask]
        local_graph = skeleton2topoMap(feed_skeleton, self.local_skeleton_mask) 
        #trim those former edge with one end in the mask and instead with a new node on the boundary outside the mask
        self.global_graph.trim_edges_in_mask(self.local_skeleton_mask)
        #for each vertex in the boundary of local_graph, build edge that connected to outer graph
        h,w = self.topo_map.shape
        graph_vertices = []
        for e in local_graph.edges:
            if e.parentEdgeId == -1:
#graph_vertices.extend(e.vertices)
                graph_vertices.append(e.vertices[0])
                graph_vertices.append(e.vertices[-1])
        graph_vertices = set(graph_vertices) 

        #find the exist_outer_vs
        outer_skeleton = self.topo_map.copy()
        outer_skeleton[self.local_skeleton_mask] = False
        exist_outer_vs = []
        for e in self.global_graph.edges:
            if e.parentEdgeId == -1:
                if not self.local_skeleton_mask[e.vertices[0]/w, e.vertices[0]%w]:
                    exist_outer_vs.append(e.vertices[0]) 
                if not self.local_skeleton_mask[e.vertices[-1]/w, e.vertices[-1]%w]:
                    exist_outer_vs.append(e.vertices[-1]) 
        exist_outer_vs = set(exist_outer_vs)
        inner_outer_connect_edges = [] #store the edges between local and outer global
        for v in graph_vertices:
            if(np.sum(self.local_skeleton_mask[v/w-1:v/w+2,v%w])+np.sum(self.local_skeleton_mask[v/w,v%w-1:v%w+2]) != 6):
                inserted_tag = local_graph.insert_outer_edge(v, outer_skeleton, exist_outer_vs)
                if inserted_tag:#another copy of global_outer
                    inner_outer_connect_edges.append(local_graph.edges[-1])

        #merge two graph 
        self.global_graph.merge_graph(local_graph) 

        #remove degree 2 vertex
        for ed in inner_outer_connect_edges:
            v1,v2 = ed.vertices[0], ed.vertices[-1]
            self.global_graph.remove_2_degree_vertex(v1) 
            self.global_graph.remove_2_degree_vertex(v2)
    def reset_topo_update_mask(self):
        #reset the mask
        self.local_skeleton_mask = np.zeros(self.topo_map.shape)

    def map_message(self):
        """ Return a nav_msgs/OccupancyGrid representation of this map. """

        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.header.frame_id = "map"

        grid_msg.info.resolution = self.occ_map.map_res
        grid_msg.info.width = self.occ_map.width
        grid_msg.info.height = self.occ_map.height

        grid_msg.info.origin = Pose(Point(self.occ_map.map_limit[0], self.occ_map.map_limit[2], 0),
                                    Quaternion(0, 0, 0, 1))

        flat_grid = copy.deepcopy(self.map.reshape((self.occ_map.map_size,)))
        for i in range(self.occ_map.map_size):
            if flat_grid[i] > self.occ_map.prob_occ:
                flat_grid[i] = 100
            elif flat_grid[i] < self.occ_map.prob_free:
                flat_grid[i] = 0
            else:
                flat_grid[i] = -1
        flat_grid = np.round(flat_grid)
        flat_grid = flat_grid.astype(int)
        grid_msg.data = list(flat_grid)
        return grid_msg

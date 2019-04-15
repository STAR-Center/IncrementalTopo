#!/usr/bin/env python

from graphClass import Graph 
import numpy as np
import matplotlib.pyplot as plt
def skeleton2topoMap(skeleton, local_skeleton_mask=None):
    #1. build a topograph from skeleton, the edges are between two skeleton pixel
    graph = Graph(skeleton)
    #2. skip edges around degree 2 vertices
    graph.edgeSkip(skeleton, local_skeleton_mask)
#
    return graph

def drawMap(graph, skeleton, ax=plt, mask = None):
    h,w = skeleton.shape
    res = np.zeros((h,w), dtype=np.uint8)
    res[np.where(skeleton == True)] = 127
    if mask is not None:
        ax.imshow(res+mask*63, cmap='GnBu')
    else:
        ax.imshow(res, cmap='GnBu')
    x = []
    y = []
    for ed in graph.edges:
        if ed.parentEdgeId == -1:
            y.append(ed.vertices[0]/w)
            x.append(ed.vertices[0]%w)
            y.append(ed.vertices[-1]/w)
            x.append(ed.vertices[-1]%w)
            ax.plot([ed.vertices[0]%w, ed.vertices[-1]%w], [ed.vertices[0]/w, ed.vertices[-1]/w], 'k-', lw=1)
    ax.scatter(x=x,y=y,c='r',s=1) 
def drawtoposkele(graph, skeleton, ax=plt, mask = None):
    h,w = skeleton.shape
    res = skeleton#np.zeros((h,w), dtype=np.uint8)
#res[np.where(skeleton == True)] = 127
    if mask is not None:
        ax.imshow(res+mask*63, cmap='GnBu')
    else:
        ax.imshow(res, cmap='GnBu')
    x = []
    y = []
    for ed in graph.edges:
        if ed.parentEdgeId == -1:
            y.append(ed.vertices[0]/w)
            x.append(ed.vertices[0]%w)
            y.append(ed.vertices[-1]/w)
            x.append(ed.vertices[-1]%w)
    ax.scatter(x=x,y=y,c='r',s=0.1) 

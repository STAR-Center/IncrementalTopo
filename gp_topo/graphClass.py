#!/usr/bin/env/python
import pdb
import numpy as np
'''
fromP, toP or someP will have a index id, for which
id = h_i * w + w_i
'''
class GraphEdge:
    #vertices = [] Not shared
    #parentEdgeId = -1 Not shared
    def __init__(self,fromP,toP, lst=None):
        self.vertices = []
        self.parentEdgeId = -1
        #-1 is not parent, -2 is removed, >=0 shows its parent's ID
        if lst == None:
            #build while init the graph
            if(fromP == 0 or toP == 0):
                print lst
                print "it is zero!!!"
            self.vertices.append(fromP)
            self.vertices.append(toP)
        else:
            #build while merge edge
            self.vertices = lst
            if len(lst) == 0:
                print "wt"

class Graph:
    def __init__(self,skeleton):
        '''
        build edges and V2E from a traverse
        '''
        self.edges = []#store the GraphEdge
        self.V2E = []#store the edge id list for each V
        
        self.h, self.w = skeleton.shape
        h,w = skeleton.shape
        self.V2E  = [[] for hw_i in range(h*w)]
        for i in range(h-1):
            for j in range(0, w-1):
                if skeleton[i,j]:
                    currentId = i*w+j
                    if skeleton[i+1,j]:
                        newE = GraphEdge(currentId, currentId + w  )
                        self.edges.append(newE)
                        self.V2E[currentId].append(len(self.edges)-1)
                        self.V2E[currentId+w].append(len(self.edges)-1)
                    if skeleton[i,j+1]:
                        newE = GraphEdge(currentId, currentId + 1  )
                        self.edges.append(newE)
                        self.V2E[currentId].append(len(self.edges)-1)
                        self.V2E[currentId+1].append(len(self.edges)-1)
                    
                    if skeleton[i+1,j+1]:
                        #----- if triangle, doesn't add
                        if not skeleton[i,j+1] and not skeleton[i+1,j]:
                        #-----
                            newE = GraphEdge(currentId, currentId + w + 1  )
                            self.edges.append(newE)
                            self.V2E[currentId].append(len(self.edges)-1)
                            self.V2E[currentId+w+1].append(len(self.edges)-1)

                    if j>0:
                        if skeleton[i+1, j-1]:
                            #------ if triangle, doesn't add
                            if not skeleton[i+1,j] and not skeleton[i,j-1]:
                            #-----
                                newE = GraphEdge(currentId, currentId + w - 1  )
                                self.edges.append(newE)
                                self.V2E[currentId].append(len(self.edges)-1)
                                self.V2E[currentId+w-1].append(len(self.edges)-1)
        i = h - 1
        for j in range(w-1):
            if skeleton[i,j]:
                if skeleton[i,j+1]:
                        currentId = i*w+j
                        newE = GraphEdge(currentId, currentId + 1  )
                        self.edges.append(newE)
                        self.V2E[currentId].append(len(self.edges)-1)
                        self.V2E[currentId+1].append(len(self.edges)-1)
        j = w - 1
        for i in range(h-1):
            if skeleton[i,j]:
                currentId = i*w+j
                if skeleton[i+1,j]:
                    newE = GraphEdge(currentId, currentId + w  )
                    self.edges.append(newE)
                    self.V2E[currentId].append(len(self.edges)-1)
                    self.V2E[currentId+w].append(len(self.edges)-1)
                if skeleton[i+1, j-1]:
                    newE = GraphEdge(currentId, currentId + w - 1  )
                    self.edges.append(newE)
                    self.V2E[currentId].append(len(self.edges)-1)
                    self.V2E[currentId+w-1].append(len(self.edges)-1)

            
    def mergeTwoEdges(self, edge1, edge2):
        '''
        two edge as input and output a new edge
        '''
        if edge1.vertices[0] == edge2.vertices[0]:
            newLst = edge1.vertices[:]
            newLst.reverse()
            newLst.pop()
            newLst.extend(edge2.vertices)
        elif edge1.vertices[len(edge1.vertices)-1] == edge2.vertices[0]:
            newLst = edge1.vertices[:]
            newLst.pop()
            newLst.extend(edge2.vertices)
        elif edge1.vertices[len(edge1.vertices)-1] == edge2.vertices[len(edge2.vertices)-1]:
            newLst = edge1.vertices[:]
            newLst.pop()
            tmpLst = edge2.vertices[:]
            tmpLst.reverse()
            newLst.extend(tmpLst)
        elif edge1.vertices[0] == edge2.vertices[len(edge2.vertices)-1]:
            newLst = edge2.vertices[:]
            newLst.pop()
            newLst.extend(edge1.vertices)
        else:
            print edge1.vertices
            print edge2.vertices
            print 'Error in MergeTwoEdges, cannot find joint vertex!!!'#shouldn't happen

        return GraphEdge(-1,-1,newLst)
       
    
    def insert_outer_edge(self,pix_loc, outer_skeleton, exist_outer_vs):
        '''
            insert the edges that connect to outer vertex to self.edges
            pix_loc: y*w+x that want another end
            outer_skeleton: skeleton in outerspace
            exist_outer_vs: list of vertexes id in outer_skeleton

        '''
        h,w = outer_skeleton.shape
        visited = np.zeros(outer_skeleton.shape, bool)
        q = [pix_loc]
        visited[pix_loc/w, pix_loc%w] = True
        #TODO check out of bound
        #this should be neighbour pixels in our algorithms, so we don't traverse back the path
        while len(q)!=0:
            loc = q.pop(0)
            for i in [-1,0,1]:
                for j in [-1,0,1]:
                    if not visited[loc/w+i, loc%w+j]:
                        visited[loc/w+i, loc%w+j] = True;
                        if outer_skeleton[loc/w+i, loc%w+j]:
                            new_id = (loc/w+i)*w+(loc%w+j) 
                            q.append(new_id)
                            if new_id in exist_outer_vs:
                                newE = GraphEdge(pix_loc, new_id )
                                self.edges.append(newE)
                                self.V2E[pix_loc].append(len(self.edges)-1)
                                self.V2E[new_id].append(len(self.edges)-1)
                                return True
        return False
    def remove_2_degree_vertex(self, pix_loc):
        edge_IDs = []
#self.V2E[pix_loc] = list(set(self.V2E[pix_loc]))#TODO: find one point with repeat edge, fix the bug
        for ed_id in self.V2E[pix_loc]:
            if self.edges[ed_id].parentEdgeId == -1:
                edge_IDs.append(ed_id)
        if(len(edge_IDs) == 2):
            #1. build new list
            newGraphEdge = self.mergeTwoEdges(self.edges[edge_IDs[0]], self.edges[edge_IDs[1]])
            #2. append the new edge
            self.edges.append(newGraphEdge)
            #3. append new edge ID into V2E
            self.V2E[newGraphEdge.vertices[0]].append(len(self.edges)-1)
            self.V2E[newGraphEdge.vertices[-1]].append(len(self.edges)-1)
            #4. set the new edge as parent of the merged edges
            self.edges[edge_IDs[0]].parentEdgeId = len(self.edges)-1
            self.edges[edge_IDs[1]].parentEdgeId = len(self.edges)-1
 
    def trim_edges_in_mask(self, local_mask):
        '''
        #only check two endpoints
        _, w = local_mask.shape
        for ed in self.edges:
            if ed.parentEdgeId == -1:
                in_mask_bina = 0
                if(local_mask[ed.vertices[0]/w, ed.vertices[0]%w]):
                    in_mask_bina += 1
                if(local_mask[ed.vertices[-1]/w, ed.vertices[-1]%w]):
                    in_mask_bina += 2
                if(in_mask_bina > 0):
                    ed.parentEdgeId = -2
                    #here we have to insert new edges if only one vertex in the mask
                    if(len(ed.vertices) > 2 and in_mask_bina < 3):
                        new_vertices = []
                        for i in range(len(ed.vertices)):
                            if not local_mask[ed.vertices[i]/w, ed.vertices[i]%w]:
                                new_vertices.append(ed.vertices[i])
                        self.edges.append(GraphEdge(-1,-1, new_vertices)) 
                        self.V2E[new_vertices[0]].append(len(self.edges)-1)
                        self.V2E[new_vertices[-1]].append(len(self.edges)-1)
        '''
        #check not the endpoint but the whole line on the mask
        _, w = local_mask.shape
        for ed in self.edges:
            if ed.parentEdgeId == -1:

                exist_v_in_mask = False #find a vertex in mask
                v_lst = []
                mask_line = False#indicate current line is masked

                if local_mask[ed.vertices[0]/w, ed.vertices[0]%w]:
                    mask_line = True
                    exist_v_in_mask = True
                else:
                    v_lst.append(ed.vertices[0])
                
                for i in range(1, len(ed.vertices)):
                    if mask_line:
                        if local_mask[ed.vertices[i]/w, ed.vertices[i]%w]:
                            pass
                        else:
                            #init one edge
                            v_lst = [ed.vertices[i]]
                            #change mask_line_tag
                            mask_line = False
                    else:
                        if local_mask[ed.vertices[i]/w, ed.vertices[i]%w]:
                            #insert one edge
                            self.edges.append(GraphEdge(-1,-1,v_lst))
                            self.V2E[v_lst[0]].append(len(self.edges)-1)
                            self.V2E[v_lst[-1]].append(len(self.edges)-1)
                            #init v list
                            v_lst = []
                            #change mask_line_tag
                            mask_line = True
                            exist_v_in_mask = True
                        else:
                            v_lst.append(ed.vertices[i])

                #check if exist one point in the mask
                if exist_v_in_mask:
                    ed.parentEdgeId = -2
                    if not mask_line:
                        #insert one edge
                        self.edges.append(GraphEdge(-1,-1,v_lst))
                        self.V2E[v_lst[0]].append(len(self.edges)-1)
                        self.V2E[v_lst[-1]].append(len(self.edges)-1)


    def parentEdge(self, edgeId):
        '''
        like union set to find the root edge node
        '''
        if(self.edges[edgeId].parentEdgeId == -1):
            return edgeId
        parentId = self.edges[edgeId].parentEdgeId
        while(parentId != -1):
            edgeId = parentId
            parentId = self.edges[parentId].parentEdgeId
        return edgeId

    def edgeSkip(self,skeleton, mask = None):
        ''' Just after init
        merge edges around degree 2 verticse during initialization
        
        WARNING:
            when mask is not None, it is in incremental alglrithm
            will not clear the vertexes on the boundary of mask to keep the connectivity
        '''
        h,w = skeleton.shape
        for i in range(h):
            for j in range(w):
                if skeleton[i,j]: 
                    currentId = i*w+j
                    '''Wrong condition
                    if len(self.V2E[currentId]) == 2:
                    '''
                    prun_cond = False

                    edge_IDs = []
                    for ed_id in self.V2E[currentId]:
                        if self.edges[ed_id].parentEdgeId == -1:
                            edge_IDs.append(ed_id)
                    if(len(edge_IDs) == 2):
#if len(self.V2E[currentId]) == 2:
                        if mask is None:
                            prun_cond = True#it's ok because the graph is just init
                        elif (np.sum(mask[i-1:i+2,j])+np.sum(mask[i, j-1:j+2]) == 6 ):
                            prun_cond = True
                    if prun_cond:
#edgeId1 = self.parentEdge(self.V2E[currentId][0])
#edgeId2 = self.parentEdge(self.V2E[currentId][1])
                        edgeId1 = edge_IDs[0]
                        edgeId2 = edge_IDs[1]
                        #1. build new list
                        newGraphEdge = self.mergeTwoEdges(self.edges[edgeId1], self.edges[edgeId2])
                        #2. append the new edge
                        self.edges.append(newGraphEdge)
                        #3. append new edge ID into V2E
                        self.V2E[newGraphEdge.vertices[0]].append(len(self.edges)-1)
                        self.V2E[newGraphEdge.vertices[-1]].append(len(self.edges)-1)
                        #4. set the new edge as parent of the merged edges 
                        self.edges[edgeId1].parentEdgeId = len(self.edges)-1
                        self.edges[edgeId2].parentEdgeId = len(self.edges)-1
    def merge_graph(self,graph_to_merge):
        #copy the edges if it is -1
        for e in graph_to_merge.edges:
            if e.parentEdgeId == -1:
                self.edges.append(e)
                self.V2E[e.vertices[0]].append(len(self.edges)-1)
                self.V2E[e.vertices[-1]].append(len(self.edges)-1)

    def getTopoVertex(self):
        '''
        after edgeSkip, we ahieve a topograph,
        this function will generate arr[n,2][h,w] for those vertex
        '''
        arr = []
        for eg in self.edges:
            if eg.parentEdgeId == -1:
                arr.append((eg.vertices[0]/self.w, eg.vertices[0]%self.w))
                arr.append((eg.vertices[len(eg.vertices)-1]/self.w, eg.vertices[len(eg.vertices)-1]%self.w))

        return list(set(arr))

import numpy as np

#Dijkstra alogrithm
#2D Digital map whish 2 is vertices , 1 is road , 0 is nothing 

class Vertex:
    def __init__(self,i:int,j:int,dist:int) :
        self.i=i
        self.j=j
        self.dist=dist
        self.prev=None
        self.isExplore=False
        self.entry_i=i
        self.entry_j=j

class Dijkstra:
    def __init__(self,digitmap:np.array):
        self.digitmap=digitmap
        self.vertices=[]
        i=0
        for row in digitmap:
            j=0
            for col in digitmap:
                if(digitmap[i][j]==2):
                    self.vertices.append(Vertex(int(i),int(j),100))
                j+=1
            i+=1
    #return vertices list and entry of road
    def getpath(self,i0:int,j0:int,i_dest:int,j_dest:int):
        path=[]
        map=self.digitmap
        for v in self.vertices:
            if(v.i==i0 and v.j==j0):
                v.dist=0
                Go_V=v
            if(v.i==i_dest and v.j==j_dest):
                Dest_V=v
        # Reference : https://www.youtube.com/watch?v=EFg3u_E6eHU
        while not Dest_V.isExplore:
            mindist=100
            for v in self.vertices:
                if v.dist<mindist:
                    if not v.isExplore:
                        mindist=v.dist
                        Target=v
            surround=[]
            Target.isExplore=True
            if(self.inBound(Target.i+1,Target.j) and map[Target.i+1][Target.j]==1):
                dist,i,j=self.findSurVertex(Target.i+1,Target.j,Target.i,Target.j)
                V=Vertex(i,j,dist)
                V.entry_i=Target.i+1
                V.entry_j=Target.j
                surround.append(V)
            if(self.inBound(Target.i-1,Target.j) and map[Target.i-1][Target.j]==1):
                dist,i,j=self.findSurVertex(Target.i-1,Target.j,Target.i,Target.j)
                V=Vertex(i,j,dist)
                V.entry_i=Target.i-1
                V.entry_j=Target.j
                surround.append(V)
            if(self.inBound(Target.i,Target.j-1) and map[Target.i][Target.j-1]==1):
                dist,i,j=self.findSurVertex(Target.i,Target.j-1,Target.i,Target.j)
                V=Vertex(i,j,dist)
                V.entry_i=Target.i
                V.entry_j=Target.j-1
                surround.append(V)
            if(self.inBound(Target.i,Target.j+1) and map[Target.i][Target.j+1]==1):
                dist,i,j=self.findSurVertex(Target.i,Target.j+1,Target.i,Target.j)
                V=Vertex(i,j,dist)
                V.entry_i=Target.i
                V.entry_j=Target.j+1
                surround.append(V)

            for sur in surround:
                for v in self.vertices:
                    if (v.i==sur.i and v.j==sur.j):
                        if Target.dist+sur.dist<v.dist:
                            v.dist=Target.dist+sur.dist
                            v.entry_i=sur.entry_i
                            v.entry_j=sur.entry_j
                            v.prev=Target
        i=100
        j=100
        v = Dest_V
        while not (i==Go_V.i and j==Go_V.j):
            path.insert(0,v)
            i=v.i
            j=v.j
            v=v.prev
        return path
    #find surround vertex and distance
    def findSurVertex(self,i,j,pos_i,pos_j):
        dist=1
        map = self.digitmap
        if (self.inBound(i+1,j) and map[i+1][j]==1 and  (i+1!=pos_i or j!=pos_j)):
            dist+=1
            d,k,l=self.findSurVertex(i+1,j,i,j)
            dist+=d
            return d,k,l
        elif (self.inBound(i-1,j) and map[i-1][j]==1 and  (i-1!=pos_i or j!=pos_j)):
            dist+=1
            d,k,l=self.findSurVertex(i-1,j,i,j)
            dist+=d
            return d,k,l
        elif (self.inBound(i,j+1) and map[i][j+1]==1 and  (i!=pos_i or j+1!=pos_j)):
            dist+=1
            d,k,l=self.findSurVertex(i,j+1,i,j)
            dist+=d
            return d,k,l
        elif (self.inBound(i,j-1) and map[i][j-1]==1 and  (i!=pos_i or j-1!=pos_j)):
            dist+=1
            d,k,l=self.findSurVertex(i,j-1,i,j)
            dist+=d
            return d,k,l
        elif (self.inBound(i+1,j) and map[i+1][j]==2 and  (i+1!=pos_i or j!=pos_j)):
            return dist,i+1,j
        elif (self.inBound(i-1,j) and map[i-1][j]==2 and  (i-1!=pos_i or j!=pos_j)):
            return dist,i-1,j
        elif (self.inBound(i,j+1) and map[i][j+1]==2 and  (i!=pos_i or j+1!=pos_j)):
            return dist,i,j+1
        elif (self.inBound(i,j-1) and map[i][j-1]==2 and  (i!=pos_i or j-1!=pos_j)):
            return dist,i,j-1
        return dist,i,j
    def inBound(self,i,j):
            if(i<0 or j<0 or i>6 or j>6):
                return False
            return True
import OpenGL.GL as gl
#import OpenGL.GLU as glu
import OpenGL.arrays.vbo as vbo
import numpy as np
#-------------------------------------------------------------------------------
# Visual representation of the objects in the simulation
#-------------------------------------------------------------------------------
class VisObj(object):
    def __init__(self):
        self.vbo = None
        self.color = None
        self.vertices = None

    def verticesFromFixtures(self, fixtures):
        """ Creates vertices from fixtures. Default behaviour assumes that
            every fixture is a square on a tile map. Should be overriden
            when the body is more specialized. It draws to CCW triangles for
            each tile.
        """
        count = len(fixtures)
        # The simulation uses 4 vertices per tile.
        # We need 6 vertices per tile (2 triangles per tile).
        hop = 4
        self.vertices = np.zeros((6*count, 3), 'f')
        for i in xrange(count):
            vertices = fixtures[i].shape.vertices
            self.vertices[i*6+0,:2] = vertices[0]
            self.vertices[i*6+1,:2] = vertices[1]
            self.vertices[i*6+2,:2] = vertices[2]
            self.vertices[i*6+3,:2] = vertices[0]
            self.vertices[i*6+4,:2] = vertices[2]
            self.vertices[i*6+5,:2] = vertices[3]
#            for j in xrange(4):
#                self.vertices[i*6+j, :2] = vertices[j]
#                self.vertices[i*6+j+3, :2] = vertices[j+1]
#                self.vertices[i*2+1, j] = fixtures[i].shape.vertices[j+1]
#            self.vertices[i*2, 0] = self.vertices[i*2,0]
        self.vbo = vbo.VBO(self.vertices, usage=gl.GL_STATIC_DRAW)

#-------------------------------------------------------------------------------
class StaticObj(VisObj):
    def __init__(self):
        super(StaticObj, self).__init__()
#-------------------------------------------------------------------------------
#class VisObj(object):
#    def __init__(self):
#        pass
#
#-------------------------------------------------------------------------------
#class VisObj(object):
#    def __init__(self):
#        pass
#

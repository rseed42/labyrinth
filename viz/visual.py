import OpenGL.GL as gl
import OpenGL.GLU as glu
import OpenGL.arrays.vbo as glvbo
#-------------------------------------------------------------------------------
# Visual representation of the objects in the simulation
#-------------------------------------------------------------------------------
class VisObj(object):
    def __init__(self):
        self.vbo = None
        self.color = None
#-------------------------------------------------------------------------------
class StaticObj(VisObj):
    def __init__(self):
        super(StaticObj, __init__())
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

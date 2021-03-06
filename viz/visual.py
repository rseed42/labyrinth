import OpenGL.GL as gl
import OpenGL.arrays.vbo as vbo
import numpy as np
#-------------------------------------------------------------------------------
# Visual representation of the objects in the simulation
#-------------------------------------------------------------------------------
class VisObj(object):
    def __init__(self, primitive=gl.GL_TRIANGLES):
        self.vbo = None
        self.color = None
        self.vertices = None
        self.drawPrimitive = primitive

    def configure(self, cfg):
        self.color = cfg.color

    def verticesFromFixtures(self, fixtures):
        """ Creates vertices from fixtures. Default behaviour assumes that
            every fixture is a square on a tile map. Should be overriden
            when the body is more specialized. It draws to CCW triangles for
            each tile.
        """
        count = len(fixtures)
        # The simulation uses 4 vertices per tile.
        # We need 6 vertices per tile (2 triangles per tile).
        self.vertices = np.zeros((6*count, 3), 'f')
        for i in xrange(count):
            vertices = fixtures[i].shape.vertices
            self.vertices[i*6+0,:2] = vertices[0]
            self.vertices[i*6+1,:2] = vertices[1]
            self.vertices[i*6+2,:2] = vertices[2]
            self.vertices[i*6+3,:2] = vertices[0]
            self.vertices[i*6+4,:2] = vertices[2]
            self.vertices[i*6+5,:2] = vertices[3]
        self.vbo = vbo.VBO(self.vertices)

    def draw(self, prog):
        """ The prog reference is used to pass object-specific uniform values
        """
        prog.setUniform('vec_Color', self.color)
        self.vbo.bind()
        gl.glVertexPointerf(self.vbo)
        gl.glDrawArrays(self.drawPrimitive, 0, self.vertices.size)
        self.vbo.unbind()

#-------------------------------------------------------------------------------
class StaticObj(VisObj):
    def __init__(self):
        super(StaticObj, self).__init__()
#-------------------------------------------------------------------------------
class DynObj(VisObj):
    def __init__(self, primitive=gl.GL_TRIANGLES):
        super(DynObj, self).__init__(primitive=primitive)
        self.translation = np.identity(4,'f')
        self.rotation = np.identity(4,'f')
        self.body = None

    def setBody(self, body):
        self.body = body

    def setVertices(self, vertices):
        """ vertices must define an (N,3) array with float32 types
        """
        self.vertices = vertices
        self.vbo = vbo.VBO(vertices)

    def draw(self, prog):
        """ The prog reference is used to pass object-specific uniform values
        """
        # Update the translation and rotation matrices
        trafo = self.body.transform
        # Later: Make the multiplication in the shader for better efficiency!
        self.translation[0,3] = self.body.transform.position[0]
        self.translation[1,3] = self.body.transform.position[1]

        # Possible b2 bug
        self.rotation[0,0] = np.cos(trafo.R.angle)
        # self.rotation[0,1] = self.body.transform.R.col2.x
        self.rotation[0,1] = -self.body.transform.R.s
        self.rotation[1,0] = np.sin(trafo.R.angle)
        # self.rotation[1,1] = self.body.transform.R.col2.y
        self.rotation[1,1] = self.body.transform.R.c


#        print self.body.transform.R.angle

#        print self.rotation[:2,:2]
        # Very strange. This should not be empty like that. Possibly a bug?
#        print self.body.transform.R.col1
#        print self.body.transform.R.col2

        prog.setUniform('mat_ModelView',
                        np.dot(self.translation, self.rotation))
        super(DynObj, self).draw(prog)
#-------------------------------------------------------------------------------
class SensorObj(DynObj):
    def __init__(self):
        super(SensorObj, self).__init__(primitive=gl.GL_LINE_LOOP)

    def setVertices(self, fixture):
        self.vertices = np.zeros((4,3),'f')
        for i in xrange(len(fixture.shape.vertices)):
            self.vertices[i,:2] = fixture.shape.vertices[i]
        super(SensorObj, self).setVertices(self.vertices)

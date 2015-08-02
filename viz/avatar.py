import OpenGL.GL as gl
import visual
import numpy as np
#-------------------------------------------------------------------------------
# Visual representation of an agent
#-------------------------------------------------------------------------------
class Avatar(object):
    def __init__(self):
        self.agent = None
        self.visCar = None
        self.visFLWheel = None
        self.visFRWheel = None
        self.visRLWheel = None
        self.visRRWheel = None
        self.visSensor = None

    def configure(self, cfg):
        pass

    def setAgent(self, agent):
        self.agent = agent

    def calculateVertices(self):
        self.visCar = visual.DynObj(primitive=gl.GL_TRIANGLE_FAN)
        self.visCar.setBody(self.agent.body)
        self.visCar.color = [0.25,0.55,0.8,1]
        # Calculate vertices manually, since we have an
        # interesting shape for the car
        vList = self.visCar.body.fixtures[0].shape.vertices
        vCount = len(vList)
        verts = np.zeros((vCount+1, 3), 'f')
        for i in xrange(vCount):
            verts[i+1,:2] = vList[i]
        self.visCar.setVertices(verts)

        wheelBody = self.agent.frontLeftWheel.body
        self.visFLWheel = visual.DynObj()
        self.visFLWheel.setBody(wheelBody)
        self.visFLWheel.color = [1,0,0,1]
        self.visFLWheel.verticesFromFixtures(wheelBody.fixtures)

        wheelBody = self.agent.frontRightWheel.body
        self.visFRWheel = visual.DynObj()
        self.visFRWheel.setBody(wheelBody)
        self.visFRWheel.color = [1,0,0,1]
        self.visFRWheel.verticesFromFixtures(wheelBody.fixtures)

        wheelBody = self.agent.rearLeftWheel.body
        self.visRLWheel = visual.DynObj()
        self.visRLWheel.setBody(wheelBody)
        self.visRLWheel.color = [.5,0,0,1]
        self.visRLWheel.verticesFromFixtures(wheelBody.fixtures)

        wheelBody = self.agent.rearRightWheel.body
        self.visRRWheel = visual.DynObj()
        self.visRRWheel.setBody(wheelBody)
        self.visRRWheel.color = [.5,0,0,1]
        self.visRRWheel.verticesFromFixtures(wheelBody.fixtures)

#        self.visSensor = visual.SensorObj()
#        self.visSensor.setBody(self.agent.body)
#        self.visSensor.color = [1,1,1,1]
#        self.visSensor.verticesFromFixtures([self.agent.sensor])
#        self.visSensor.setVertices(self.agent.sensor)

    def draw(self, prog):
        """ Overrides the default function
        """
        self.visCar.draw(prog)
        self.visFLWheel.draw(prog)
        self.visFRWheel.draw(prog)
        self.visRLWheel.draw(prog)
        self.visRRWheel.draw(prog)
#        self.visSensor.draw(prog)

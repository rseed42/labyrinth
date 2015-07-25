import OpenGL.GL as gl
import visual
#-------------------------------------------------------------------------------
# Visual representation of an agent
#-------------------------------------------------------------------------------
class Avatar(object):
    def __init__(self):
        self.agent = None
        self.visCar = None
        self.visFLWheel = None
        self.visFRWheel = None

    def configure(self, cfg):
        pass

    def setAgent(self, agent):
        self.agent = agent

    def calculateVertices(self):
        self.visCar = visual.DynObj()
        self.visCar.setBody(self.agent.body)
        self.visCar.color = [1,1,1,1]
        self.visCar.verticesFromFixtures([self.agent.body.fixtures[0]])

        self.visFLWheel = visual.DynObj()
        self.visFLWheel.setBody(self.agent.frontLeftWheel)
        self.visFLWheel.color = [1,0,0,1]
        self.visFLWheel.verticesFromFixtures(self.agent.frontLeftWheel.fixtures)

        self.visFRWheel = visual.DynObj()
        self.visFRWheel.setBody(self.agent.frontRightWheel)
        self.visFRWheel.color = [1,0,0,1]
        self.visFRWheel.verticesFromFixtures(self.agent.frontRightWheel.fixtures)

    def draw(self, prog):
        """ Overrides the default function
        """
        self.visCar.draw(prog)
        self.visFLWheel.draw(prog)
        self.visFRWheel.draw(prog)

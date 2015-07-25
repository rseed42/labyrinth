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
        self.visRLWheel = None
        self.visRRWheel = None

    def configure(self, cfg):
        pass

    def setAgent(self, agent):
        self.agent = agent

    def calculateVertices(self):
        self.visCar = visual.DynObj()
        self.visCar.setBody(self.agent.body)
        self.visCar.color = [0.25,0.55,0.8,1]
        self.visCar.verticesFromFixtures([self.agent.body.fixtures[0]])

        self.visFLWheel = visual.DynObj()
        self.visFLWheel.setBody(self.agent.frontLeftWheel)
        self.visFLWheel.color = [1,0,0,1]
        self.visFLWheel.verticesFromFixtures(self.agent.frontLeftWheel.fixtures)

        self.visFRWheel = visual.DynObj()
        self.visFRWheel.setBody(self.agent.frontRightWheel)
        self.visFRWheel.color = [1,0,0,1]
        self.visFRWheel.verticesFromFixtures(self.agent.frontRightWheel.fixtures)

        self.visRLWheel = visual.DynObj()
        self.visRLWheel.setBody(self.agent.rearLeftWheel)
        self.visRLWheel.color = [.5,0,0,1]
        self.visRLWheel.verticesFromFixtures(self.agent.rearLeftWheel.fixtures)

        self.visRRWheel = visual.DynObj()
        self.visRRWheel.setBody(self.agent.rearRightWheel)
        self.visRRWheel.color = [.5,0,0,1]
        self.visRRWheel.verticesFromFixtures(self.agent.rearRightWheel.fixtures)

    def draw(self, prog):
        """ Overrides the default function
        """
        self.visCar.draw(prog)
        self.visFLWheel.draw(prog)
        self.visFRWheel.draw(prog)
        self.visRLWheel.draw(prog)
        self.visRRWheel.draw(prog)

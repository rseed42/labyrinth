import json
import Box2D as b2
import numpy as np
import bunch
import worldmap
import staticobject
import dynamicobject
import agent
#from contactlistener import WorldContactListener
#-------------------------------------------------------------------------------
class Simulation(object):
    def __init__(self):
        # World map
        self.worldCfg = None
        self.staticObjects = {}
        self.dynamicObjects = {}
        self.agents = {}
        # Non-interactive simulation:
        self.running = False
        # Solver params
        self.timestep = 0
        self.velocityIterations = 0
        self.positionIterations = 0
        # World parameters
        self.width = 0
        self.height = 0

    def loadDynamicObject(self, geometry, obj):
        pass

    def loadStaticObject(self, geometry, obj):
        tileSize = 1.0
        # We create a single static body for all the maze walls
        bodyDef = b2.b2BodyDef()
        bodyDef.type = b2.b2_staticBody
        bodyDef.position = (0, 0)
        body = self.world.CreateBody(bodyDef)
        for i in xrange(self.width):
            for j in xrange(self.height):
                if geometry[i,j] == 0: continue
                if geometry[i,j] == obj.id:
                    # Calculate tile vertices and add the tile as a fixture
                    # to the body
                    w,h = i+tileSize, j+tileSize
                    body.CreatePolygonFixture(vertices=((i,j),(w,j), (w,h),(i,h)),
                                              density=0)
        body.userData = obj
        obj.setBody(body)

    def load(self, worldCfgFilename):
        # Load world map
        fp = file(worldCfgFilename, 'r')
        self.worldCfg = bunch.bunchify(json.load(fp))
        fp.close()
        # Configure the simulation parameters
        self.timestep = self.worldCfg.solver.timestep
        self.velocityIterations = self.worldCfg.solver.velocityIterations
        self.positionIterations = self.worldCfg.solver.positionIterations
        # Load world geometry
        fp = file(self.worldCfg.geometryFilename, 'rb')
        geometry = np.load(fp).transpose()
        fp.close()
        # Set world parameters
        self.width, self.height = geometry.shape
        # Create the dynamics world
        self.world = b2.b2World((0,0))
        # Load static objects
        for name, objCfg in self.worldCfg.objects.static.items():
            self.staticObjects[name] = staticobject.StaticObject()
            self.staticObjects[name].configure(objCfg)
            self.loadStaticObject(geometry, self.staticObjects[name])
        # Load dynamics objects
#        for name, objCfg in self.worldCfg.objects.dynamic.items():
#            self.dynamicObjects[name] = dynamicobject.DynamicObject(name, objCfg)
#            self.loadDynamicObject(geometry, self.dynamicObjects[name])
        for name, agentCfg in self.worldCfg.objects.agents.items():
            self.agents[name] = agent.Agent(name)
            self.agents[name].construct(self.world, agentCfg)

#        # Set up oevent handling
#        self.contactListener = WorldContactListener(self.user)
#        self.world.contactListener = self.contactListener
#        return True

    def reset(self):
        self.running = False
#        self.wmap = None
#        self.user = None
#        if not self.load():
#            sys.stderr.write(msg.sim_load_fail+msg.newline)


    def start(self):
        """ Used in non-interactive simulations"""
        self.running = True
        self.run()

    def run(self):
        """ Used in non-interactive simulations"""
        while self.running:
            self.step()

    def step(self):
        """ Update with a given step size """
        self.world.Step(self.timestep,
                        self.velocityIterations,
                        self.positionIterations)
        for name, agent in self.agents.items():
            agent.update()

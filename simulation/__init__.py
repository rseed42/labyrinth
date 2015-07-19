import sys
from conf import msg
try:
    import Box2D as b2
except ImportError:
    sys.stderr.write(msg.import_box2d_fail+msg.newline)
    sys.exit(1)
try:
    import bunch
except ImportError:
    sys.stderr.write(msg.import_bunch_fail+msg.newline)
    sys.exit(1)

from loader import MapLoader
#-------------------------------------------------------------------------------
class Simulation(object):
    def __init__(self):
        # Non-interactive simulation:
        self.running = False
        # Solver params
        self.timestep = 0
        self.velocityIterations = 0
        self.positionIterations = 0
        # World parameters
        self.worldLowerBound = None
        self.worldUpperBound = None
#        worldAABB = b2.b2AABB()
#        worldAABB.lowerBound = (-10, -10)
#        worldAABB.upperBound = (10, 10)
#        worldAABB.lowerBound = (-40, -40)
#        worldAABB.upperBound = (40, 40)

#        gravity = (0,0)
#        doSleep = True
#        self.world = b2.b2World(worldAABB, gravity, doSleep)
        # Add maze
#        self.mapLoader = MapLoader()
#        self.mapLoader.load(MAZE_FILENAME)
#        scale = 1.
#        trans = (-40, -40)
#        for wall in self.mapLoader.walls:
#            for brick in wall:
#                brickBodyDef = b2.b2BodyDef()
#                brickBodyDef.position = (brick[0]*scale+trans[0],
#                                         brick[1]*scale+trans[1])
#                brickBody = self.world.CreateBody(brickBodyDef)
#                brickShapeDef = b2.b2PolygonDef()
#                brickShapeDef.SetAsBox(0.5*scale, 0.5*scale)
#                brickBody.CreateShape(brickShapeDef)

        # Add ground
#        groundBodyDef = b2.b2BodyDef()
#        groundBodyDef.position = (0,-9)
#        self.groundBody = self.world.CreateBody(groundBodyDef)
#        groundShapeDef = b2.b2PolygonDef()
#        groundShapeDef.SetAsBox(8, 0.25)
#        self.groundBody.CreateShape(groundShapeDef)
#        self.groundBody.angle = 8*(b2.b2_pi/180)
#        # Static shapes
#        obstacleBodyDef = b2.b2BodyDef()
#        obstacleBodyDef.position = (0,4)
#        obstacleBody = self.world.CreateBody(obstacleBodyDef)
#        obstacleShapeDef = b2.b2PolygonDef()
#        verts = ((-5,-0.2), (1,-0.2), (3,3), (2.8,3.3), (2, 1.8), (-5,0.2))
#        obstacleShapeDef.setVertices(verts)
#        obstacleBody.CreateShape(obstacleShapeDef)

        # Add dynamic bodies
#        bodyDef = b2.b2BodyDef()
#        bodyDef.position = (0,0)
#
#        self.body = self.world.CreateBody(bodyDef)
#        shapeDef = b2.b2PolygonDef()
#        shapeDef.SetAsBox(0.5,0.5)
#        shapeDef.density = 1
#        shapeDef.friction = 0.08
#
#        self.body.CreateShape(shapeDef)
#        self.body.SetMassFromShapes()
#        self.body.SetLinearVelocity((2, 10))
#        self.body.SetAngularVelocity(3)

#        self.agentBody = AgentBody(self.world)

    def configure(self, cfg_file):
        """ Configuration before the map is loaded
        """
        import json
        fp = file(cfg_file, 'r')
        self.cfg = bunch.bunchify(json.load(fp))
        fp.close()
        # Solver configuration
        self.timestep = self.cfg.solver.timestep
        self.velocityIterations = self.cfg.solver.velocityIterations
        self.positionIterations = self.cfg.solver.positionIterations
        # World configuration ---remove, this should come from the map ---
        self.worldLowerBound = tuple(self.cfg.world.lowerBound)
        self.worldUpperBound = tuple(self.cfg.world.upperBound)



    def load(self):
        # Build the world
        worldAABB = b2.b2AABB()
        worldAABB.lowerBound = self.worldLowerBound
        worldAABB.upperBound = self.worldUpperBound
        self.world = b2.b2World(worldAABB, (0,0), True)
        # Load the map and populate the world
        return True

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
#        self.world.Step(TIME_STEP, VEL_ITER, POS_ITER)
        self.world.Step(self.timestep,
                        self.velocityIterations,
                        self.positionIterations)
#        self.agentBody.update()
#        print self.body.position, self.body.angle, self.body.linearVelocity
#        pass

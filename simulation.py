import msg
try:
    import Box2D as b2
except ImportError:
    import sys
    sys.stderr.write(msg.cant_import_box2d+msg.newline)
    sys.exit(1)
#-------------------------------------------------------------------------------
TIME_STEP = 1.0/60.0
VEL_ITER = 10
POS_ITER = 8
#-------------------------------------------------------------------------------
# Main Simulation Object
#-------------------------------------------------------------------------------
class Simulation(object):
    def __init__(self):
        # Non-interactive simulation:
        self.running = False
        # Create the world
        worldAABB = b2.b2AABB()
        worldAABB.lowerBound = (-10, -10)
        worldAABB.upperBound = (10, 10)
        gravity = (0,-10)
        doSleep = True
        self.world = b2.b2World(worldAABB, gravity, doSleep)
        # Add ground
        groundBodyDef = b2.b2BodyDef()
        groundBodyDef.position = (0,-9)
        self.groundBody = self.world.CreateBody(groundBodyDef)
        groundShapeDef = b2.b2PolygonDef()
        groundShapeDef.SetAsBox(8, 0.25)
        self.groundBody.CreateShape(groundShapeDef)
        self.groundBody.angle = 8*(b2.b2_pi/180)
        # Static shapes
        obstacleBodyDef = b2.b2BodyDef()
        obstacleBodyDef.position = (0,4)
        obstacleBody = self.world.CreateBody(obstacleBodyDef)
        obstacleShapeDef = b2.b2PolygonDef()
        verts = ((-5,-0.2), (1,-0.2), (3,3), (2.8,3.3), (2, 1.8), (-5,0.2))
        obstacleShapeDef.setVertices(verts)
#        obstacleShapeDef.SetAsBox(6, 0.25)
        obstacleBody.CreateShape(obstacleShapeDef)
#        obstacleBody.angle = -3*(b2.b2_pi/180)

        # Add dynamic bodies
        bodyDef = b2.b2BodyDef()
        bodyDef.position = (0,0)

        self.body = self.world.CreateBody(bodyDef)
        shapeDef = b2.b2PolygonDef()
        shapeDef.SetAsBox(0.5,0.5)
        shapeDef.density = 1
        shapeDef.friction = 0.08

        self.body.CreateShape(shapeDef)
        self.body.SetMassFromShapes()
        self.body.SetLinearVelocity((2, 10))
        self.body.SetAngularVelocity(3)

    def start(self):
        """ Used in non-interactive simulations"""
        print msg.starting_simulation
        self.running = True
        self.run()

    def run(self):
        """ Used in non-interactive simulations"""
        while self.running:
            self.step()

    def step(self):
        """ Update with a given step size """
        self.world.Step(TIME_STEP, VEL_ITER, POS_ITER)
#        print self.body.position, self.body.angle, self.body.linearVelocity
        pass

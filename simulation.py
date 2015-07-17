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
#
#-------------------------------------------------------------------------------
class Simulation(object):
    def __init__(self):
        worldAABB = b2.b2AABB()
        worldAABB.lowerBound = (0, 0)
        worldAABB.upperBound = (50, 50)
        gravity = (0,-10)
        doSleep = True
        self.world = b2.b2World(worldAABB, gravity, doSleep)
        # Add ground
        groundBodyDef = b2.b2BodyDef()
        groundBodyDef.position = (0,0)
        groundBody = self.world.CreateBody(groundBodyDef)
        groundShapeDef = b2.b2PolygonDef()
        groundShapeDef.SetAsBox(1,50)
        groundBody.CreateShape(groundShapeDef)
        # Add dynamic bodies
        bodyDef = b2.b2BodyDef()
        bodyDef.position = (20,30)
        self.body = self.world.CreateBody(bodyDef)
        shapeDef = b2.b2PolygonDef()
        shapeDef.SetAsBox(1,1)
        shapeDef.density = 1
        shapeDef.friction = 0.3
        self.body.CreateShape(shapeDef)
        self.body.SetMassFromShapes()
#        self.body.SetLinearVelocity((0, 2))

    def start(self):
        """ Used in non-interactive simulations"""
        print msg.starting_simulation

    def run(self):
        """ Used in non-interactive simulations"""
        pass

    def step(self):
        """ Update with a given step size """
        self.world.Step(TIME_STEP, VEL_ITER, POS_ITER)
        print self.body.position, self.body.angle, self.body.linearVelocity
        pass

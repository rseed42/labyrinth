import msg
try:
    import Box2D as b2
except ImportError:
    import sys
    sys.stderr.write(msg.cant_import_box2d+msg.newline)
    sys.exit(1)
#-------------------------------------------------------------------------------
#
#-------------------------------------------------------------------------------
class Simulation(object):
    def __init__(self):
        worldAABB = b2.b2AABB()
        worldAABB.lowerBound = (-100, -100)
        worldAABB.lowerBound = (100, 100)
        gravity = (0,-9.8)
        doSleep = True
        self.world = b2.b2World(worldAABB, gravity, doSleep)
        # Add ground
        groundBodyDef = b2.b2BodyDef()
        groundBodyDef.position = (0,-10)
        groundBody = self.world.CreateBody(groundBodyDef)
        groundShapeDef = b2.b2PolygonDef()
        groundShapeDef.SetAsBox(50,10)
        groundBody.CreateShape(groundShapeDef)
        # Add dynamic bodies
        bodyDef = b2.b2BodyDef()
        bodyDef.position = (0,4)
        body = self.world.CreateBody(bodyDef)
        shapeDef = b2.b2PolygonDef()
        shapeDef.SetAsBox(1,1)
        shapeDef.density = 1
        shapeDef.friction = 0.3
        body.CreateShape(shapeDef)
        body.SetMassFromShapes()

    def start(self):
        """ Used in non-interactive simulations"""
        print msg.starting_simulation

    def run(self):
        """ Used in non-interactive simulations"""
        pass

    def step(self):
        """ Update with a given step size """
        pass

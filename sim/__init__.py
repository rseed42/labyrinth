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
import worldmap
from contactlistener import WorldContactListener
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
        self.wmap = None
        self.user = None

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
        self.world = b2.b2World((0,0))
        # Load the map and populate the world
        self.wmap = worldmap.WorldMap(self.cfg.worldmap.filename, self.world)
        self.wmap.load()
        self.user = self.wmap['user']
        # Set up oevent handling
        self.contactListener = WorldContactListener(self.user)
        self.world.contactListener = self.contactListener
        return True

    def reset(self):
        self.running = False
        self.wmap = None
        self.user = None
        if not self.load():
            sys.stderr.write(msg.sim_load_fail+msg.newline)

    def getAgents(self):
        return self.wmap

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
        self.user.update()

    def userAccelerate(self):
        self.user.accelerate()

    def userReleaseAccelerator(self):
        self.user.releaseAccelerator()

    def userReverse(self):
        self.user.reverse()

    def userReleaseReverse(self):
        self.user.releaseReverse()

    def userBrake(self):
        self.user.brake()

    def userReleaseBrake(self):
        self.user.releaseBrake()

    def userSteerLeft(self):
        self.user.steerLeft()

    def userSteerRight(self):
        self.user.steerRight()

    def userReleaseSteering(self):
        self.user.releaseSteering()

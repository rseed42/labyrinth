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
try:
    import numpy as np
except ImportError:
    sys.stderr.write(msg.import_numpy_fail+msg.newline)
    sys.exit(1)
import agent
#-------------------------------------------------------------------------------
# Contains the information needed to set up the simulation
#-------------------------------------------------------------------------------
class WorldMap(dict):
    """ Contains all the simulation objects. As a dictionary, contains the agents
    """
    def __init__(self, filename, world):
        # Physical Dimensions
        self.width = 0
        self.height = 0
        self.cfg_file = filename
        self.world = world

    def load(self):
        import json
        fp = file(self.cfg_file, 'r')
        self.cfg = bunch.bunchify(json.load(fp))
        fp.close()
        # Load some world properties
        self.width = self.cfg.width
        self.height = self.cfg.height
        # Static objects
        self.loadStaticObjects()
        self.loadDynamicObjects()

    def loadStaticObjects(self):
        # Load maze
        fp = file(self.cfg.mazefile, 'rb')
        # Load the maze file and invert it to correspond to the simulation's
        # coordinate system
        maze = np.load(fp)
        fp.close()
        self.maze = np.flipud(maze).transpose()
        # Convert the maze to static bodies and the graphical representations
        tileSize = float(self.width)/(self.maze.shape[0])
        # We create a single static body for all the maze walls
        mazeWallsDef = b2.b2BodyDef()
        mazeWallsDef.position = (0, 0)
        self.mazeWalls = self.world.CreateBody(mazeWallsDef)

        for i in xrange(self.maze.shape[0]):
            for j in xrange(self.maze.shape[1]):
                if self.maze[i,j] == 0: continue
                # Maze Walls
                if self.maze[i,j] == 1:
                    self.addWallBrick(i,j,tileSize)

    def addWallBrick(self, i, j, tileSize):
        w,h = i+tileSize, j+tileSize
        self.mazeWalls.CreatePolygonFixture(vertices=((i,j),(w,j), (w,h),(i,h)),
                                            density=0)

    def generateWallTriangles(self):
        """ This is a separate functions, since we might not always use the
            visualization and simulate interactively.
        """
        triangles = []
        tileSize = float(self.width)/(self.maze.shape[0])
        for i in xrange(self.maze.shape[0]):
            for j in xrange(self.maze.shape[1]):
                if self.maze[i,j] == 0: continue
                if self.maze[i,j] == 1:
                    w,h = i+tileSize, j+tileSize
                    triangle1=((i,j),(w,j),(w,h))
                    triangle2=((i,j),(w,h),(i,h))
                    triangles.append(triangle1)
                    triangles.append(triangle2)
        return np.array(triangles, dtype=np.float32)


    def loadDynamicObjects(self):
        """ Objects which are not static
        """
        self.loadAgents()

    def loadAgents(self):
        self.loadUserAgent()

    def loadUserAgent(self):
        if not self.cfg.user: return
        # Construct the agent
        self["user"] = agent.Agent()
        self["user"].construct(self.cfg.user, self.world)
        # Set up controller (later)


    def test(self):
        pass

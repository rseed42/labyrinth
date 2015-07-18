import numpy as np

class MapLoader(object):
    def __init__(self):
        pass

    def walk(self, maze, i, j, wall):
        wall.append((i,j))
        maze[i,j] = False
        # Going clockwork-wise, starting from the upper neighbor
        if maze[i,j-1] == True:
            self.walk(maze, i, j-1, wall)
        if maze[i+1,j] == True:
            self.walk(maze, i+1, j, wall)
        if maze[i,j+1] == True:
            self.walk(maze, i,j+1, wall)
        if maze[i-1,j] == True:
            self.walk(maze, i-1,j, wall)

    def findWalls(self, maze):
        """ Find contiguous walls in the maze
        """
        walls = []
        maze[0,:] = maze[-1, :] = False
        maze[:,0] = maze[:, -1] = False
        for i in xrange(1,maze.shape[0]-1):
            for j in xrange(1,maze.shape[1]-1):
                if maze[i,j] == False: continue
                # We are the root
                wall = []
                self.walk(maze,i,j,wall)
                walls.append(wall)
        return walls

    def load(self, filename):
        maze = np.load(file(filename, 'rb'))
#        self.walls = self.findWalls(np.flipud(maze))
        self.walls = self.findWalls(np.flipud(maze).transpose())


if __name__ == '__main__':
    print 'map loader'
    ml = MapLoader()
    fn = "maze.bin"
    ml.load(fn)
    print ml.walls

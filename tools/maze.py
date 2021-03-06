import numpy
from numpy.random import random_integers as rand
import matplotlib.pyplot as pyplot
from numpy.random import random_integers as rand
import matplotlib.pyplot as pyplot


WIDTH = 250
HEIGHT= 250
#WIDTH = 150
#HEIGHT= 150

COMPLEXITY = 0.012
DENSITY = 0.004

# 0 - free
# 1 - wall
# 2 - ?

def maze(width=81, height=51, complexity=.75, density=.75):
    # Only odd shapes
    shape = ((height // 2) * 2 + 1, (width // 2) * 2 + 1)
    # Adjust complexity and density relative to maze size
    complexity = int(complexity * (5 * (shape[0] + shape[1])))
    density    = int(density * (shape[0] // 2 * shape[1] // 2))
    # Build actual maze
    Z = numpy.zeros(shape, dtype=bool)
    # Fill borders
    Z[0, :] = Z[-1, :] = 1
    Z[:, 0] = Z[:, -1] = 1
    # Make aisles
    for i in range(density):
        x, y = rand(0, shape[1] // 2) * 2, rand(0, shape[0] // 2) * 2
        Z[y, x] = 1
        for j in range(complexity):
            neighbours = []
            if x > 1:             neighbours.append((y, x - 2))
            if x < shape[1] - 2:  neighbours.append((y, x + 2))
            if y > 1:             neighbours.append((y - 2, x))
            if y < shape[0] - 2:  neighbours.append((y + 2, x))
            if len(neighbours):
                y_,x_ = neighbours[rand(0, len(neighbours) - 1)]
                if Z[y_, x_] == 0:
                    Z[y_, x_] = 1
                    Z[y_ + (y - y_) // 2, x_ + (x - x_) // 2] = 1
                    x, y = x_, y_
    return Z.astype(numpy.int32)

z = maze(WIDTH, HEIGHT, COMPLEXITY, DENSITY)
# Add more features
# Show
pyplot.figure(figsize=(10, 5))
pyplot.imshow(z,cmap=pyplot.cm.binary, interpolation='nearest')
pyplot.xticks([]), pyplot.yticks([])
pyplot.show()
# Save
numpy.save(file('maze.bin', 'wb'), z)

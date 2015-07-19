import numpy
import matplotlib.pyplot as pyplot

z = numpy.load(file('data/maze.bin','rb'))
# Show
pyplot.figure(figsize=(10, 5))
pyplot.imshow(z,cmap=pyplot.cm.binary, interpolation='nearest')
pyplot.xticks([]), pyplot.yticks([])
pyplot.show()

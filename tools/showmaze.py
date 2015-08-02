import sys
import numpy
import matplotlib.pyplot as pyplot

fn = sys.argv[1]
if not fn:
    sys.stderr.write('Need path to file\n')
    sys.exit(1)
z = numpy.load(file(fn,'rb'))
# Show
pyplot.figure(figsize=(10, 5))
pyplot.imshow(z,cmap=pyplot.cm.binary, interpolation='nearest')
pyplot.xticks([]), pyplot.yticks([])
pyplot.show()

import msg
try:
    import box2d
except ImportError:
    import sys
    sys.stderr.write(msg.cant_import_box2d+msg.newline)
    sys.exit(1)

class Simulation(object):
    def __init__(self):
        pass

    def start(self):
        print msg.starting_simulation

    def run(self):
        pass

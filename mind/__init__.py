""" Some missing stuff: Implement touch sensors and signals
"""
class Mind(object):
    def __init__(self):
        self.visionField = None
        self.actions = {}

    def configure(self, cfg, agent):
        # Set up action map
        self.actions['acc'] = agent.accelerate
        self.actions['racc'] = agent.releaseAccelerator
        self.actions['rev'] = agent.reverse
        self.actions['rrev'] = agent.releaseReverse
        self.actions['brk'] = agent.brake
        self.actions['rbrk'] = agent.releaseBrake
        self.actions['left'] = agent.steerLeft
        self.actions['right'] = agent.steerRight
        self.actions['rsteer'] = agent.releaseSteering
        # Connect vision field
        self.visionFIeld = agent.sensorField

#    def visionFieldToPercepts()
    def think(self):
        """ Override by subclasses
        """
        pass

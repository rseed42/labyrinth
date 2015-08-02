""" Some missing stuff: Implement touch sensors and signals
"""
class Mind(object):
    def __init__(self):
        self.visionField = None
        self.actions = {}
        self.agent = None

    def configure(self, cfg, agent):
        # Keep a reference to the agent to read states
        self.agent = agent
        # Set up action map
        self.actions['acc'] = agent.accelerate
        self.actions['racc'] = agent.releaseAccelerator
        self.actions['rev'] = agent.reverse
        self.actions['rrev'] = agent.releaseReverse
        self.actions['left'] = agent.steerLeft
        self.actions['rleft'] = agent.releaseLeft
        self.actions['right'] = agent.steerRight
        self.actions['rright'] = agent.releaseRight
#        self.actions['brk'] = agent.brake
#        self.actions['rbrk'] = agent.releaseBrake
        # Connect vision field
        self.visionField = agent.sensorField

#    def visionFieldToPercepts()
    def think(self):
        """ Override by subclasses
        """
        raise Exception('Please, override this function')

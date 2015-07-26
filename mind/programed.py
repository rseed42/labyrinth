import mind
import numpy as np
#-------------------------------------------------------------------------------
# Programmed Mind where the mind actually executes a Python program. Mostly
# used for tests or bots that are part of the environment. More advanced minds
# will be dynamic systems (like neural networks) and not Turing machines.
#-------------------------------------------------------------------------------
PROB_CHANGE_DIRECTION = 0.01
MAX_STEER_CNT = 1
#-------------------------------------------------------------------------------
class MindProgram(mind.Mind):
    def __init__(self):
        super(MindProgram, self).__init__()
        self.steerCount = 0
        self.steer = None
        # Store last 120 positions
        self.time = 0
        self.memory = np.zeros((120,2),'f')
        self.avgDistance = 1

    def configure(self, cfg, agent):
        super(MindProgram, self).configure(cfg, agent)
        # Simulate that we have been here for the last 2 seconds
        self.memory[:,0] = agent.body.transform.position.x
        self.memory[:,1] = agent.body.transform.position.y

    def think(self):
        # Calculate the average distance, relative to the current pos:
        pos = np.array(self.agent.body.transform.position, 'f')
        if self.time > self.memory.shape[0]:
            relpos = pos - self.memory
            self.avgDistance = np.sqrt((relpos**2).sum(axis=1)).sum()/pos.shape[0]
        # This means that the agent can not move
        if self.avgDistance < 1:
            # Try to get out by reversing
            print 'try rev'
            self.actions['left']()
            self.actions['rev']()
            return

        # Move ahead
        if len(self.visionField) == 0:
            if self.steerCount == 0:
                if self.agent.steeringAngle > 0:
#                    print 'RLS STEER'
                    self.actions['rsteer']()
            else:
#                print self.steerCount
                self.steer()
                self.steerCount -= 1
                return


            if self.agent.engineSpeed < 50:
                self.actions['acc']()

            # Randomly decide to change direction
            if np.random.binomial(1, PROB_CHANGE_DIRECTION):
#                print 'CHNG DIR'
                if np.random.randint(2):
#                    print 'RIGHT'
                    self.steer = self.actions['right']
                else:
#                    print 'LEFT'
                    self.steer = self.actions['left']
                self.steerCount = MAX_STEER_CNT
        # Store to memory
        self.memory = np.roll(self.memory, -1, axis=0)
        self.memory[-1,:] =  self.agent.body.transform.position
        self.time += 1

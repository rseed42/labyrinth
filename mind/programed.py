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

    def think(self):
        # Move ahead
        if len(self.visionField) == 0:
            if self.steerCount == 0:
                if self.agent.steeringAngle > 0:
                    print 'RLS STEER'
                    self.actions['rsteer']()
            else:
                print self.steerCount
                self.steer()
                self.steerCount -= 1
                return


            if self.agent.engineSpeed < 50:
                self.actions['acc']()

            # Randomly decide to change direction
            if np.random.binomial(1, PROB_CHANGE_DIRECTION):
                print 'CHNG DIR'
                if np.random.randint(2):
#                    print 'RIGHT'
                    self.steer = self.actions['right']
                else:
#                    print 'LEFT'
                    self.steer = self.actions['left']
                self.steerCount = MAX_STEER_CNT

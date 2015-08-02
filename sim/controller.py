import sdl2 as sdl
#-------------------------------------------------------------------------------
WDC_LEFT  = 0x1
WDC_RIGHT = 0x2
WDC_UP    = 0x4
WDC_DOWN  = 0x8
#-------------------------------------------------------------------------------
# Agent Controller
#-------------------------------------------------------------------------------
""" The controller translates the actions of the user/mind to physical responses
    in the effectors of the agent
"""
class UserController(object):
    def __init__(self, agent):
        self.agent = agent
#        print self.agent.controlState

    def keyDown(self, sb):
        if   sb == sdl.SDLK_a:
            self.agent.controlState |= WDC_LEFT
        elif sb == sdl.SDLK_d:
            self.agent.controlState |= WDC_RIGHT
        elif sb == sdl.SDLK_w:
            self.agent.controlState |= WDC_UP
        elif sb == sdl.SDLK_s:
            self.agent.controlState |= WDC_DOWN

#        if sb == sdl.SDLK_w:
#            self.agent.accelerate()
#        if sb == sdl.SDLK_k:
#            self.agent.reverse()
#        if sb == sdl.SDLK_s:
#            self.agent.brake()
#        if sb == sdl.SDLK_a:
#            self.agent.steerLeft()
#        if sb == sdl.SDLK_d:
#            self.agent.steerRight()

    def keyUp(self, sb):
        if   sb == sdl.SDLK_a:
            self.agent.controlState &= ~WDC_LEFT
        elif sb == sdl.SDLK_d:
            self.agent.controlState &= ~WDC_RIGHT
        elif sb == sdl.SDLK_w:
            self.agent.controlState &= ~WDC_UP
        elif sb == sdl.SDLK_s:
            self.agent.controlState &= ~WDC_DOWN

#        if sb == sdl.SDLK_w:
#            self.agent.releaseAccelerator()
#        if sb == sdl.SDLK_k:
#            self.agent.releaseReverse()
#        if sb == sdl.SDLK_s:
#            self.agent.releaseBrake()
#        if sb == sdl.SDLK_a:
#            self.agent.releaseSteering()
#        if sb == sdl.SDLK_d:
#            self.agent.releaseSteering()

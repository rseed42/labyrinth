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

    def keyDown(self, sb):
        if   sb == sdl.SDLK_a:
            self.agent.controlState |= WDC_LEFT
        elif sb == sdl.SDLK_d:
            self.agent.controlState |= WDC_RIGHT
        elif sb == sdl.SDLK_w:
            self.agent.controlState |= WDC_UP
        elif sb == sdl.SDLK_s:
            self.agent.controlState |= WDC_DOWN

    def keyUp(self, sb):
        if   sb == sdl.SDLK_a:
            self.agent.controlState &= ~WDC_LEFT
        elif sb == sdl.SDLK_d:
            self.agent.controlState &= ~WDC_RIGHT
        elif sb == sdl.SDLK_w:
            self.agent.controlState &= ~WDC_UP
        elif sb == sdl.SDLK_s:
            self.agent.controlState &= ~WDC_DOWN

import sdl2 as sdl
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
        if sb == sdl.SDLK_w:
            self.agent.accelerate()
        if sb == sdl.SDLK_k:
            self.agent.reverse()
        if sb == sdl.SDLK_s:
            self.agent.brake()
        if sb == sdl.SDLK_a:
            self.agent.steerLeft()
        if sb == sdl.SDLK_d:
            self.agent.steerRight()

    def keyUp(self, sb):
        if sb == sdl.SDLK_w:
            self.agent.releaseAccelerator()
        if sb == sdl.SDLK_k:
            self.agent.releaseReverse()
        if sb == sdl.SDLK_s:
            self.agent.releaseBrake()
        if sb == sdl.SDLK_a:
            self.agent.releaseSteering()
        if sb == sdl.SDLK_d:
            self.agent.releaseSteering()

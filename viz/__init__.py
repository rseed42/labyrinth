#!/usr/bin/env python
import sys
import sdl2 as sdl
import sdl2.ext as sdlx
import OpenGL.GL as gl
import numpy as np
import bunch
import stats
import renderer
from sim import controller
#-------------------------------------------------------------------------------
WND_FLAGS = sdl.SDL_WINDOW_OPENGL | sdl.SDL_WINDOW_SHOWN
#-------------------------------------------------------------------------------
# Visualization Application
#-------------------------------------------------------------------------------
class Visualization(object):
    def __init__(self, simulation):
        self.glcontext = None
        # Simulation
        self.sim = simulation
        self.runSimulation = False
        self.running = True
        self.targetFrameDuration = 0
        # Frame Rate Statistics
        self.frameStats = None
        # Window
        self.width = 0
        self.height = 0
        self.window = None
        self.glcontext = None
        self.bgColor = None
        # Rendering
        self.renderer = None
        self.worldView = False
        # User Controller
        self.userController = None

    def configure(self, vizCfgFilename):
        import json
        fp = file(vizCfgFilename, 'r')
        self.cfg = bunch.bunchify(json.load(fp))
        fp.close()
        self.runSimulation = self.cfg.runSimulationAtStart
        self.targetFrameDuration = 1000./self.cfg.fps
        self.worldView = self.cfg.worldView
        self.width = self.cfg.window.width
        self.height = self.cfg.window.height
        self.bgColor = self.cfg.window.bgColor

    def init_gl(self):
        gl.glClearColor(*self.bgColor)
        gl.glClearDepth(1.0)
        gl.glEnable(gl.GL_TEXTURE_2D)
        gl.glViewport(0,0,self.width, self.height)
        print '-'*40
        print 'GL INFO'
        print '-'*40
        print gl.glGetString(gl.GL_VERSION)
        print gl.glGetString(gl.GL_SHADING_LANGUAGE_VERSION)
        print '-'*40

    def initialize(self):
        sdl.SDL_GL_SetAttribute(sdl.SDL_GL_ACCELERATED_VISUAL,True)
        sdl.SDL_GL_SetAttribute(sdl.SDL_GL_CONTEXT_MAJOR_VERSION, 3)
        sdl.SDL_GL_SetAttribute(sdl.SDL_GL_CONTEXT_MINOR_VERSION, 0)
        self.glcontext = sdl.SDL_GL_CreateContext(self.window)

        # Frame stats
        self.frameStats = stats.FrameStats(self.cfg.fps)
        # Open GL
        self.init_gl()
        # Renderer
        self.renderer = renderer.Renderer()
        self.renderer.loadShaders(self.cfg.shaderDir,
                                  self.cfg.shaders,
                                  self.cfg.programs)
        self.renderer.initVisuals(self.cfg.visuals,
                                  self.sim.staticObjects,
                                  self.sim.dynamicObjects,
                                  self.sim.agents
        )
        # Set up user controller
        agent = self.sim.agents[self.sim.worldCfg.user]
        self.userController = controller.UserController(agent)
        # Set up the projection at the end, since we need the user
        # controller
        self.renderer.worldView = self.worldView
        if self.worldView == 1:
            self.renderer.setProjection(self.sim.width, self.sim.height)
        elif self.worldView == 0:
            width, height = self.userController.agent.fov
            self.renderer.setProjection(width, height)
        return True

    def cleanup(self):
        sdl.SDL_GL_DeleteContext(self.glcontext)
        sdl.SDL_DestroyWindow(self.window)
        sdl.SDL_Quit()

    def process_event(self, event):
        if event.type == sdl.SDL_KEYDOWN:
            self.on_key_down(event.key.keysym)
        if event.type == sdl.SDL_KEYUP:
            self.on_key_up(event.key.keysym)
        if event.type == sdl.SDL_QUIT:
            self.exit_()

    def exit_(self):
        self.running = False

    def on_key_down(self, keysym):
        # Control simulation
        if keysym.sym == sdl.SDLK_q:
            self.exit_()
        if keysym.sym == sdl.SDLK_r:
            self.reset()
        if keysym.sym == sdl.SDLK_p:
            self.runSimulation = not self.runSimulation
            print '-'*40
            print "Run Simulation: ", self.runSimulation
        if keysym.sym == sdl.SDLK_h:
            self.showHelp()
        if keysym.sym == sdl.SDLK_g:
            if self.worldView == 0:
                self.worldView = 1
                self.renderer.setProjection(self.sim.width, self.sim.height)
                self.renderer.matModelView = np.identity(4,'f')
                self.renderer.matView = np.identity(4,'f')
            elif self.worldView == 1:
                self.worldView = 0
                width, height = self.userController.agent.fov
                self.renderer.setProjection(width, height)
            self.renderer.worldView = self.worldView
        if keysym.sym == sdl.SDLK_f:
            self.frameStats.show()
        if keysym.sym == sdl.SDLK_i:
            # use for tests
            pass
        # Control agent
        self.userController.keyDown(keysym.sym)

    def on_key_up(self, keysym):
        pass
        self.userController.keyUp(keysym.sym)

    def showHelp(self):
        print '-'*40
        print ' Help'
        print '-'*40
        print 'q: Quit'
        print 'r: Reset'
        print 'p: Start/Pause simulation (toggle)'
        print 'g: Switch world view/agent view'
        print '-'*40
        print 'w: Accelerate'
        print 'k: Reverse'
        print 's: Brake'
        print 'a: Turn left'
        print 'd: Turn right'
        print 'f: Frame statistics'

    def start(self):
        """ Main application loop
        """
        if sdl.SDL_Init(sdl.SDL_INIT_EVERYTHING) < 0:
            return False

        sdl.SDL_GL_SetAttribute(sdl.SDL_GL_DOUBLEBUFFER, 1)
        self.window = sdl.SDL_CreateWindow(self.cfg.window.title,
                                           sdl.SDL_WINDOWPOS_CENTERED,
                                           sdl.SDL_WINDOWPOS_CENTERED,
                                           self.cfg.window.width,
                                           self.cfg.window.height,
                                           WND_FLAGS
        )
        if not self.window: return False


        if not self.initialize():
            # Log error instead
#            sys.stderr.write(msg.viz_gl_init_fail+msg.newline)
            sys.exit(1)
        while self.running:
            frameStart = sdl.SDL_GetTicks()
            events = sdlx.get_events()
            for event in events:
                self.process_event(event)
            eventsEnd = sdl.SDL_GetTicks()
            self.frameStats.inputSum += eventsEnd - frameStart
            # Simulate
            if self.runSimulation: self.sim.step()
            simEnd = sdl.SDL_GetTicks()
            self.frameStats.simulationSum += simEnd - eventsEnd
            self.renderer.render(self.window, self.userController)
            renderEnd = sdl.SDL_GetTicks()
            self.frameStats.renderSum += renderEnd - simEnd
            # Agents calculations
            # (a1, a2, ...)
            agentsEnd = sdl.SDL_GetTicks()
            self.frameStats.agentSum += agentsEnd - renderEnd
            # Calculate compensation for the frame rate
            self.frameStats.count += 1
            frameDuration = agentsEnd - frameStart
            self.frameStats.totalSum += frameDuration
            delay = int(self.targetFrameDuration - frameDuration)
            self.frameStats.delaySum += delay
            if frameDuration < self.targetFrameDuration:
                sdl.SDL_Delay(delay)
        self.cleanup()
        sys.exit(0)

    def reset(self):
        # Stop simulation temporarily
        self.running = False
        self.runSimulation = False
        # Reset the simulation
        self.sim.reset()
        # Remove all objects that need to be recreated
        self.frameStats = None
        self.renderer = None
        self.userController = None
        # No need to reconfigure, what changes state should
        # be returned (see below) to its configuration
        self.initialize()

        self.runSimulation = self.cfg.runSimulationAtStart
        self.running = True

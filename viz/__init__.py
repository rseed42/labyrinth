#!/usr/bin/env python
import sys
import sdl2 as sdl
import sdl2.ext as sdlx
import OpenGL.GL as gl
import OpenGL.GLU as glu
import OpenGL.arrays.vbo as glvbo
from OpenGL.GL import shaders
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
        self.frameStats = stats.FrameStats(self.cfg.fps)

        self.worldView = self.cfg.worldView
        self.width = self.cfg.window.width
        self.height = self.cfg.window.height
        self.bgColor = self.cfg.window.bgColor

    def init_gl(self):
        gl.glClearColor(*self.bgColor)
        gl.glClearDepth(1.0)
        gl.glEnable(gl.GL_TEXTURE_2D)
        gl.glViewport(0,0,self.width, self.height)

    def initialize(self):
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
        self.glcontext = sdl.SDL_GL_CreateContext(self.window)
        # Open GL
        self.init_gl()
        # Renderer
        self.renderer = renderer.Renderer()
        self.renderer.setProjection(self.sim.width, self.sim.height)
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
        pass
        if keysym.sym == sdl.SDLK_q:
            self.exit_()
#        if keysym.sym == sdl.SDLK_r:
#            self.runSimulation = False
#            self.sim.reset()
#            self.runSimulation = True
        if keysym.sym == sdl.SDLK_p:
            self.runSimulation = not self.runSimulation
            print '-'*40
            print "Run Simulation: ", self.runSimulation

        if keysym.sym == sdl.SDLK_h:
            self.showHelp()
        if keysym.sym == sdl.SDLK_g:
            self.worldView = not self.worldView
        if keysym.sym == sdl.SDLK_f:
            self.frameStats.show()
        if keysym.sym == sdl.SDLK_i:
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
            self.renderer.render(self.window)
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

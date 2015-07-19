#!/usr/bin/env python
import sys
from conf import msg
# Refactor this nicely some day
try:
    import sdl2 as sdl
    import sdl2.ext as sdlx
except ImportError:
    sys.stderr.write(msg.import_sdl2_fail+msg.newline)
    sys.exit(1)
try:
    import OpenGL.GL as gl
    import OpenGL.arrays.vbo as glvbo
except ImportError:
    sys.stderr.write(msg.import_opengl_fail+msg.newline)
    sys.exit(1)
try:
    import numpy as np
except ImportError:
    sys.stderr.write(msg.import_numpy_fail+msg.newline)
    sys.exit(1)
try:
    import bunch
except ImportError:
    sys.stderr.write(msg.import_bunch_fail+msg.newline)
    sys.exit(1)
#-------------------------------------------------------------------------------
WND_FLAGS = sdl.SDL_WINDOW_OPENGL | sdl.SDL_WINDOW_SHOWN
#-------------------------------------------------------------------------------
# A very simple opengl app
#-------------------------------------------------------------------------------
class Visualization(object):
    """ Interactive visualization of the simulation. Default mode is to display the
    field of view of the agent. Further modes like top-down view of the whole
    world will be added. The simulation here is stepped, based on the frame rate.
    In the first implementation the frame rate is fixed to 60 fps.
    """
    def __init__(self, simulation):
        self.window = None
        self.glcontext = None
        self.running = True
        self.frameStart = 0
        self.frameTime = 0
        self.sim = simulation
        self.runSimulation = True
        # Rendering
        self.mazeWallColor = None

    def configure(self, cfg_file):
        import json
        fp = file(cfg_file, 'r')
        self.cfg = bunch.bunchify(json.load(fp))
        fp.close()
        # Set up configuration (override initialized vars if necessary)
        self.delay_time = 1000./self.cfg.fps
        self.mazeWallColor = tuple(self.cfg.color.mazeWall)

    def init_gl(self):
        gl.glClearColor(*self.cfg.color.background)
        gl.glClearDepth(1.0)
        gl.glEnable(gl.GL_TEXTURE_2D)
        gl.glViewport(0,0,self.cfg.window.width, self.cfg.window.height)
        gl.glOrtho(0, 101, 0, 101,-1, 1)
        gl.glMatrixMode(gl.GL_PROJECTION)
        gl.glLoadIdentity()
        # Load static map for the maze walls
        self.wallTriangles = self.sim.wmap.generateWallTriangles()
        self.vboMazeWalls = glvbo.VBO(self.wallTriangles)

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
        self.glcontect = sdl.SDL_GL_CreateContext(self.window)
        # Open GL
        self.init_gl()
        return True

    def start(self):
        if not self.initialize():
            sys.stderr.write(msg.viz_gl_init_fail+msg.newline)
            sys.exit(1)
        while self.running:
            self.frameStart = sdl.SDL_GetTicks()
            events = sdlx.get_events()
            for event in events:
                self.process_event(event)
            # Only run the simulation if allowed
            if self.runSimulation:
                self.sim.step()
            self.render()
            self.frameTime = sdl.SDL_GetTicks() - self.frameStart
            if self.frameTime < self.delay_time:
                sdl.SDL_Delay(int(self.delay_time - self.frameTime))
        self.cleanup()
        sys.exit(0)

    def render(self):
        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
        gl.glLoadIdentity()

        gl.glColor3f(*self.mazeWallColor)
        # Load static buffers
        try:
            self.vboMazeWalls.bind()
            try:
                gl.glEnableClientState(gl.GL_VERTEX_ARRAY)
                gl.glVertexPointerf(self.vboMazeWalls)
                gl.glDrawArrays(gl.GL_TRIANGLES, 0,
                                self.wallTriangles.size)
            finally:
                self.vboMazeWalls.unbind()
                gl.glDisableClientState(gl.GL_VERTEX_ARRAY)
        finally:
            # We can also use shaders, etc.
            pass


        sdl.SDL_GL_SwapWindow(self.window)

    def cleanup(self):
        sdl.SDL_GL_DeleteContext(self.glcontext)
        sdl.SDL_DestroyWindow(self.window)
        sdl.SDL_Quit()

    def process_event(self, event):
        if event.type == sdl.SDL_KEYDOWN:
            self.on_key_down(event.key.keysym)
#        if event.type == sdl.SDL_KEYUP:
#            self.on_key_up(event.key.keysym)
        if event.type == sdl.SDL_QUIT:
            self.exit_()

    def exit_(self):
        self.running = False

    def on_key_down(self, keysym):
        if keysym.sym == sdl.SDLK_q:
            self.exit_()
#        if keysym.sym == sdl.SDLK_UP:
#            self.sim.agentBody.body.WakeUp()
#            self.sim.agentBody.deccelerate()
#
#        if keysym.sym == sdl.SDLK_DOWN:
#            self.sim.agentBody.accelerate()
#
#        if keysym.sym == sdl.SDLK_LEFT:
#            self.sim.agentBody.steerLeft()
#        if keysym.sym == sdl.SDLK_RIGHT:
#            self.sim.agentBody.steerRight()
#
#    def on_key_up(self, keysym):
#        if keysym.sym == sdl.SDLK_UP:
#            self.sim.agentBody.stopEngine()
#        if keysym.sym == sdl.SDLK_DOWN:
#            self.sim.agentBody.stopEngine()
#        if keysym.sym == sdl.SDLK_LEFT:
#            self.steeringAngle = 0
#        if keysym.sym == sdl.SDLK_RIGHT:
#            self.steeringAngle = 0

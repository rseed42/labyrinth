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
    import OpenGL.GLU as glu
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
        self.agentFov = None
        self.worldView = False

    def configure(self, cfg_file):
        import json
        fp = file(cfg_file, 'r')
        self.cfg = bunch.bunchify(json.load(fp))
        fp.close()
        # Set up configuration (override initialized vars if necessary)
        self.delay_time = 1000./self.cfg.fps
        self.mazeWallColor = tuple(self.cfg.color.mazeWall)
        self.agentFov = self.cfg.agentFov

    def init_gl(self):
        gl.glClearColor(*self.cfg.color.background)
        gl.glClearDepth(1.0)
        gl.glEnable(gl.GL_TEXTURE_2D)
        gl.glViewport(0,0,self.cfg.window.width, self.cfg.window.height)
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
        # Global world view
        if self.worldView:
            aspect = float(800)/600
            gl.glMatrixMode(gl.GL_PROJECTION)
            gl.glLoadIdentity()
            gl.glOrtho(0, 101*aspect, 0, 101,-1, 1)
        # Agent View
        else:
            gl.glMatrixMode(gl.GL_PROJECTION)
            gl.glLoadIdentity()
            gl.glOrtho(0, 40, 0, 30,-1, 1)
            gl.glMatrixMode(gl.GL_MODELVIEW)
            gl.glLoadIdentity()
            user = self.sim.user
            translation = user.body.transform.position
            sensorVerts = user.body.fixtures[1].shape.vertices
            gl.glTranslatef(sensorVerts[0][0],-(16-0.5*30),0)
            gl.glRotatef((180/np.pi)*user.body.transform.angle, 0, 0, -1)
            gl.glTranslatef(-translation.x,-translation.y,0)

        # Load static buffers
        gl.glColor3f(*self.mazeWallColor)
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

        # The dynamics body are rendered in a more simple way
        for name, agent in self.sim.getAgents().items():
            agent.draw(gl, self.worldView)

        sdl.SDL_GL_SwapWindow(self.window)

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
            self.runSimulation = False
            self.sim.reset()
            self.runSimulation = True
        if keysym.sym == sdl.SDLK_g:
            self.worldView = not self.worldView

        # Control agent
        if keysym.sym == sdl.SDLK_w:
            self.sim.userAccelerate()
        if keysym.sym == sdl.SDLK_k:
            self.sim.userReverse()
        if keysym.sym == sdl.SDLK_s:
            self.sim.userBrake()
        if keysym.sym == sdl.SDLK_a:
            self.sim.userSteerLeft()
        if keysym.sym == sdl.SDLK_d:
            self.sim.userSteerRight()

    def on_key_up(self, keysym):
        if keysym.sym == sdl.SDLK_w:
            self.sim.userReleaseAccelerator()
        if keysym.sym == sdl.SDLK_k:
            self.sim.userReleaseReverse()
        if keysym.sym == sdl.SDLK_s:
            self.sim.userReleaseBrake()
        if keysym.sym == sdl.SDLK_a:
            self.sim.userReleaseSteering()
        if keysym.sym == sdl.SDLK_d:
            self.sim.userReleaseSteering()

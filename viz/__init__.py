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
    from OpenGL.GL import shaders
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
# Check if 330 is supported on this machine
#-------------------------------------------------------------------------------
VERTEX_SHADER = """#version 130
uniform mat4 mat_ModelView;
uniform mat4 mat_Proj;
in vec4 Vertex;
void main(){
//    gl_Position = mat_ModelView * Vertex;
    gl_Position = mat_Proj * mat_ModelView * Vertex;
}
"""
FRAGMENT_SHADER = """#version 130
void main(){
  gl_FragColor = vec4(0,0,1,0.7);
}
"""
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
        # Window
        self.width = 0
        self.height = 0
        # Rendering
        self.mazeWallColor = None
        self.agentFov = None
        self.worldView = False
        self.uniforms = {}
        self.mat_model = np.identity(4, 'f')
        self.mat_model[1,3] = 0.1
        self.mat_worldProj = np.identity(4,'f')

    def configure(self, cfg_file):
        import json
        fp = file(cfg_file, 'r')
        self.cfg = bunch.bunchify(json.load(fp))
        fp.close()
        # Set up configuration (override initialized vars if necessary)
        self.delay_time = 1000./self.cfg.fps
        self.mazeWallColor = tuple(self.cfg.color.mazeWall)
        self.agentFov = self.cfg.agentFov
        self.worldView = self.cfg.worldView
        self.width = self.cfg.window.width
        self.height = self.cfg.window.height
        # World Projection Matrix
        aspect = float(800)/600
        left = 0.
        right = 201.*aspect
        top = 0.
        bottom = 201.
        zNear = -1
        zFar = 1.
        self.mat_worldProj[0,0] = 2./(right-left)
        self.mat_worldProj[0,3] = - (right+left) / (right-left)
        self.mat_worldProj[1,1] = 2./(top-bottom)
        self.mat_worldProj[1,3] = -(top+bottom)/(top-bottom)
        self.mat_worldProj[2,2] = -2 / (zFar - zNear)
        self.mat_worldProj[2,3] = - (zFar+zNear)/ (zFar - zNear)
        self.mat_worldProj[3,3] = 1.


    def loadShaders(self, gl):
        vertex_shader = shaders.compileShader(VERTEX_SHADER,
                                              gl.GL_VERTEX_SHADER)
        fragment_shader = shaders.compileShader(FRAGMENT_SHADER,
                                                gl.GL_FRAGMENT_SHADER)
        self.shader = shaders.compileProgram(vertex_shader, fragment_shader)


#        self.mat_proj[0,0] = 1./(0.5*self.width)
#        self.mat_proj[1,1] = 1./(0.5*self.height)
#        self.mat_proj[2,2] = -1

        # Pass the transformation matrix
        self.uniforms['mat_ModelView'] =  gl.glGetUniformLocation(self.shader,
                                                                  'mat_ModelView')
        self.uniforms['mat_Proj'] =  gl.glGetUniformLocation(self.shader,
                                                             'mat_Proj')

        self.uniforms['mat_Test'] = gl.glGetUniformLocation(self.shader,
                                                           'mat_Test')

    def init_gl(self):
        gl.glClearColor(*self.cfg.color.background)
        gl.glClearDepth(1.0)
        gl.glEnable(gl.GL_TEXTURE_2D)
        gl.glViewport(0,0,self.width, self.height)
        # Load shaders
        self.loadShaders(gl)
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
#        if self.worldView:
#            aspect = float(800)/600
#            gl.glMatrixMode(gl.GL_PROJECTION)
#            gl.glLoadIdentity()
#            gl.glOrtho(0, 201*aspect, 0, 201,-1, 1)
#        # Agent View
#        else:
#            gl.glMatrixMode(gl.GL_PROJECTION)
#            gl.glLoadIdentity()
#            gl.glOrtho(0, 40, 0, 30,-1, 1)
#            gl.glMatrixMode(gl.GL_MODELVIEW)
#            gl.glLoadIdentity()
#            user = self.sim.user
#            translation = user.body.transform.position
#            sensorVerts = user.body.fixtures[1].shape.vertices
#            gl.glTranslatef(sensorVerts[0][0],-(16-0.5*30),0)
#            gl.glRotatef((180/np.pi)*user.body.transform.angle, 0, 0, -1)
#            gl.glTranslatef(-translation.x,-translation.y,0)

        # Load static buffers
#        gl.glColor3f(*self.mazeWallColor)

        shaders.glUseProgram(self.shader)
        gl.glUniformMatrix4fv(self.uniforms['mat_ModelView'], 1, True,
                              self.mat_model)
        gl.glUniformMatrix4fv(self.uniforms['mat_Proj'], 1, True,
                              self.mat_worldProj)


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
            shaders.glUseProgram(0)

        # The dynamics body are rendered in a more simple way
#        for name, agent in self.sim.getAgents().items():
#            agent.draw(gl, self.worldView)

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

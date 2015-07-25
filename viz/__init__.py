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
#import shader
import renderer
#-------------------------------------------------------------------------------
WND_FLAGS = sdl.SDL_WINDOW_OPENGL | sdl.SDL_WINDOW_SHOWN
class Visualization(object):
    """ Interactive visualization of the simulation. Default mode is to display the
    field of view of the agent. Further modes like top-down view of the whole
    world will be added. The simulation here is stepped, based on the frame rate.
    In the first implementation the frame rate is fixed to 60 fps.
    """
    def __init__(self, simulation):
        # Simulation
        self.sim = simulation
        self.runSimulation = False
        self.running = True
        self.targetFrameDuration = 0
        # Statistics
        self.frameCount = 0
        self.frameDurationSum = 0
        # Window
        self.width = 0
        self.height = 0
        self.window = None
        self.glcontext = None
        self.bgColor = None
        # Rendering
        self.renderer = None
#        self.mazeWallColor = None
#        self.agentFov = None
        self.worldView = False
#        self.uniforms = {}
#        self.mat_model = np.identity(4, 'f')
#        self.mat_worldProj = None
#        self.mat_agentProj = None
#        # Shaders
#        self.shaderVertex = shader.Shader()
#        self.shaderFragment = shader.Shader()
#
    def configure(self, vizCfgFilename):
        import json
        fp = file(vizCfgFilename, 'r')
        self.cfg = bunch.bunchify(json.load(fp))
        fp.close()
#        # Load shader source
#        self.shaderVertex.load(self.cfg.shader.shader_dir,
#                               self.cfg.shader.vertex)
#        self.shaderFragment.load(self.cfg.shader.shader_dir,
#                                 self.cfg.shader.fragment)
        self.runSimulation = self.cfg.runSimulationAtStart
        self.targetFrameDuration = 1000./self.cfg.fps
#        self.mazeWallColor = tuple(self.cfg.color.mazeWall)
#        self.agentFov = self.cfg.agentFov
        self.worldView = self.cfg.worldView
        self.width = self.cfg.window.width
        self.height = self.cfg.window.height
        self.bgColor = self.cfg.window.bgColor
#        # World Projection Matrix
#        self.mat_worldProj = self.projMatrix(0,self.sim.wmap.width,
#                                             0,self.sim.wmap.height,
#                                             -1,1)
#        # Agent Projection Matrix
#        self.mat_agentProj = self.projMatrix(0,40,0,30,-1,1)
#        user = self.sim.user
#        translation = user.body.transform.position
#        sensorVerts = user.body.fixtures[1].shape.vertices
##            gl.glTranslatef(sensorVerts[0][0],-(16-0.5*30),0)
#        sensorTrans = np.identity(4,'f')
##        sensorTrans[0,3] = sensorVerts[0][0]
##        sensorTrans[1,3] = -(16-0.5*30)
##        self.mat_agentProj = np.dot(self.mat_agentProj, sensorTrans)
#
#    def projMatrix(self,left,right,top,bottom,near,far):
#        mat = np.identity(4, 'f')
#        mat[0,0] = 2./(right-left)
#        mat[0,3] = - (right+left) / (right-left)
#        mat[1,1] = 2./(top-bottom)
#        mat[1,3] = -(top+bottom)/(top-bottom)
#        mat[2,2] = -2 / (far - near)
#        mat[2,3] = - (far+near)/ (far - near)
#        mat[3,3] = 1.
#        return mat
#
#    def loadShaders(self, gl):
#        pass
##        vertex_shader = shaders.compileShader(VERTEX_SHADER,
##                                              gl.GL_VERTEX_SHADER)
##        fragment_shader = shaders.compileShader(FRAGMENT_SHADER,
##                                                gl.GL_FRAGMENT_SHADER)
##        self.shader = shaders.compileProgram(vertex_shader, fragment_shader)
##
##        # Pass the transformation matrix
##        self.uniforms['mat_ModelView'] =  gl.glGetUniformLocation(self.shader,
##                                                                  'mat_ModelView')
##        self.uniforms['mat_Proj'] =  gl.glGetUniformLocation(self.shader,
##                                                             'mat_Proj')
##
##        self.uniforms['mat_Test'] = gl.glGetUniformLocation(self.shader,
##                                                           'mat_Test')
#
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
        self.renderer.loadShaders(self.cfg.shaderDir,
                                  self.cfg.shaders,
                                  self.cfg.programs)
        self.renderer.initVisuals(self.sim)
        return True

#    def render(self):
#        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
#        self.renderer.render()
##        shaders.glUseProgram(self.shader)
##        # Global world view
##        if self.worldView:
##            gl.glUniformMatrix4fv(self.uniforms['mat_Proj'], 1, True,
##                                  self.mat_worldProj)
##        # Agent view
##        else:
##            user = self.sim.user
##            pos = user.body.transform.position
##            R = user.body.transform.R
###            gl.glRotatef((180/np.pi)*user.body.transform.angle, 0, 0, -1)
###            gl.glTranslatef(-translation.x,-translation.y,0)
##            trans = np.identity(4,'f')
###            trans[0,3] = pos.x
###            trans[1,3] = -pos.y
##
##            rot = np.identity(4,'f')
###            rot[0,0] = R.col1.x
###            rot[0,1] = R.col2.x
###            rot[1,0] = R.col1.y
###            rot[1,1] = R.col2.y
###            self.mat_agentProj = np.dot(self.mat_agentProj, rot)
###            self.mat_agentProj = np.dot(rot, self.mat_agentProj)
##            gl.glUniformMatrix4fv(self.uniforms['mat_Proj'], 1, True,
##                                  self.mat_agentProj)
##
##        # Load static buffers
###        gl.glColor3f(*self.mazeWallColor)
##
##        gl.glUniformMatrix4fv(self.uniforms['mat_ModelView'], 1, True,
##                              self.mat_model)
##
##        self.vboMazeWalls.bind()
##        gl.glEnableClientState(gl.GL_VERTEX_ARRAY)
##        gl.glVertexPointerf(self.vboMazeWalls)
##        gl.glDrawArrays(gl.GL_TRIANGLES, 0, self.wallTriangles.size)
##        self.vboMazeWalls.unbind()
##        gl.glDisableClientState(gl.GL_VERTEX_ARRAY)
##
##        # The dynamics body are rendered in a more simple way
##        for name, agent in self.sim.getAgents().items():
##            gl.glUniformMatrix4fv(self.uniforms['mat_ModelView'], 1, True,
##                                  agent.translation)
##            agent.vboBody.bind()
##            gl.glEnableClientState(gl.GL_VERTEX_ARRAY)
##            gl.glVertexPointerf(agent.vboBody)
##            gl.glDrawArrays(gl.GL_TRIANGLES, 0, agent.bodyVertices.size)
##            agent.vboBody.unbind()
##            gl.glDisableClientState(gl.GL_VERTEX_ARRAY)
#
#        sdl.SDL_GL_SwapWindow(self.window)

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
            self.showFrameStat()

#        # Control agent
##        if keysym.sym == sdl.SDLK_w:
##            self.sim.userAccelerate()
##        if keysym.sym == sdl.SDLK_k:
##            self.sim.userReverse()
##        if keysym.sym == sdl.SDLK_s:
##            self.sim.userBrake()
##        if keysym.sym == sdl.SDLK_a:
##            self.sim.userSteerLeft()
##        if keysym.sym == sdl.SDLK_d:
##            self.sim.userSteerRight()
#
    def on_key_up(self, keysym):
        pass
##        if keysym.sym == sdl.SDLK_w:
##            self.sim.userReleaseAccelerator()
##        if keysym.sym == sdl.SDLK_k:
##            self.sim.userReleaseReverse()
##        if keysym.sym == sdl.SDLK_s:
##            self.sim.userReleaseBrake()
##        if keysym.sym == sdl.SDLK_a:
##            self.sim.userReleaseSteering()
##        if keysym.sym == sdl.SDLK_d:
##            self.sim.userReleaseSteering()
#
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

    def showFrameStat(self):
        print '-'*40
        print ' Frame Statistics'
        print '-'*40
        print 'Target FPS: %.1f' % self.cfg.fps
        print 'Target Frame Duration: %.2f ms' % self.targetFrameDuration
        print 'Frame Count: %d' % self.frameCount
        print 'Frame Duration Sum: %.2f ms %.2f s' % (self.frameDurationSum,
                                                      self.frameDurationSum/1000.)
        print 'Mean Frame Duration: %.2f ms' % (float(self.frameDurationSum) / self.frameCount)

    def start(self):
        if not self.initialize():
            # Log error instead
#            sys.stderr.write(msg.viz_gl_init_fail+msg.newline)
            sys.exit(1)
        while self.running:
            frameStart = sdl.SDL_GetTicks()
            events = sdlx.get_events()
            for event in events:
                self.process_event(event)
            # Only run the simulation if allowed
            if self.runSimulation:
                self.sim.step()
#            self.render()
            self.renderer.render(self.window)
            frameDuration = sdl.SDL_GetTicks() - frameStart
            self.frameCount += 1
            self.frameDurationSum += frameDuration
            if frameDuration < self.targetFrameDuration:
                sdl.SDL_Delay(int(self.targetFrameDuration - frameDuration))
        self.cleanup()
        sys.exit(0)

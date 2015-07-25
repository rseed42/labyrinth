import sdl2 as sdl
import OpenGL.GL as gl
import numpy as np
import shader
import shaderprogram
import bunch
import visual
#-------------------------------------------------------------------------------
# Renderer
#-------------------------------------------------------------------------------
class Renderer(object):
    def __init__(self):
        self.shaders = {}
        self.programs = {}
        self.visStatic = {}
        self.visDynamic = {}
        self.visAgents = {}
        self.matMV = np.identity(4,'f')
        self.matProj = None

    def loadShaders(self, shaderDir, cfgShaders, cfgPrograms):
        # Load shaders
        for name, s in cfgShaders.items():
            self.shaders[name] = shader.Shader()
            self.shaders[name].load(shaderDir, s)
            self.shaders[name].compile()
        # Load programs
        for name, p in cfgPrograms.items():
            self.programs[name] = shaderprogram.ShaderProgram()
            # Attach shaders
            for shaderName in p.shaderNames:
                s = self.shaders[shaderName]
                self.programs[name].attachShader(shaderName,s)
            # Link the program
            self.programs[name].link()
            self.programs[name].mapUniformLocations()

    def projMatrix(self,left,right,top,bottom,near,far):
        mat = np.identity(4, 'f')
        mat[0,0] = 2./(right-left)
        mat[0,3] = - (right+left) / (right-left)
        mat[1,1] = 2./(top-bottom)
        mat[1,3] = -(top+bottom)/(top-bottom)
        mat[2,2] = -2 / (far - near)
        mat[2,3] = - (far+near)/ (far - near)
        mat[3,3] = 1.
        return mat

    def setProjection(self, width, height):
        self.matProj = self.projMatrix(0,width, 0,height, -1,1)

    def initVisuals(self, cfg, staticObjects, dynamicObjects, agents):
        """ Because we want to completely decouple the visualization
            from the simulation, we can reference the objects in the
            simulation by their names only. This means, that the names
            in the visualization configuration file must be the same
            as the names in the world map file.
        """
        for name, sObj in staticObjects.items():
            # Visual configuration for this object
            visCfg = cfg.static.get(name)
            vObj = visual.StaticObj()
            self.visStatic[name] = vObj
            vObj.verticesFromFixtures(sObj.body.fixtures)

#        for name, dObj in dynamicsObjects.items():
#            print name

#        for name, agents in agents.items():
#            print name

    def createAvatar(self):
        pass

    def render(self, window):
        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
        # Render scene
        # Currently using the same shaders for all objects
        prog = self.programs['default']
        prog.use()
        # Set Projection Matrix
        u=prog.uniforms['mat_Proj']
        gl.glUniformMatrix4fv(u.loc, 1, True, self.matProj)
        u =prog.uniforms['mat_ModelView']
        gl.glUniformMatrix4fv(u.loc, 1, True, self.matMV)
        # Render static objects
        for name, vObj in self.visStatic.items():
            # Set static uniforms if necessary
            vObj.vbo.bind()
            gl.glEnableClientState(gl.GL_VERTEX_ARRAY)
            gl.glVertexPointerf(vObj.vbo)
            gl.glDrawArrays(gl.GL_TRIANGLES, 0, vObj.vertices.size)
            vObj.vbo.unbind()
            gl.glDisableClientState(gl.GL_VERTEX_ARRAY)
        gl.glUseProgram(0)
        #
        sdl.SDL_GL_SwapWindow(window)

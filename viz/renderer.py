import sdl2 as sdl
import OpenGL.GL as gl
import OpenGL.GLU as glu
import OpenGL.arrays.vbo as glvbo
import shader
import shaderprogram
import bunch
#-------------------------------------------------------------------------------
# Renderer
#-------------------------------------------------------------------------------
class Renderer(object):
    def __init__(self):
        self.shaders = {}
        self.programs = {}

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

    def initVisuals(self, sim):
        pass

    def createAvatar(self):
        pass

    def render(self, window):
        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
        # Render scene
        # Need which program to use
        self.programs['default'].use()


        gl.glUseProgram(0)
        #
        sdl.SDL_GL_SwapWindow(window)

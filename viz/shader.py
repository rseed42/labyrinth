import sys
import OpenGL.GL as gl
import OpenGL.GLU as glu
import OpenGL.arrays.vbo as glvbo
#from OpenGL.GL import shaders
import bunch
#-------------------------------------------------------------------------------
# Shader
#-------------------------------------------------------------------------------
class Shader(object):
    def __init__(self):
        self.type = None
        self.src = None
        self.shaderId = 0
        self.uniforms = bunch.Bunch()
        # Create shader

    def load(self, shaderDir, cfg):
        # Init type of shader
        if cfg.type == 'vertex':
            self.type = gl.GL_VERTEX_SHADER
        elif cfg.type == 'fragment':
            self.type = gl.GL_FRAGMENT_SHADER
        # Load shader source code
        fp = file(shaderDir+cfg.filename, 'r')
        self.src = fp.read()
        fp.close()
        # Load uniform types
        self.uniforms = cfg.uniforms.copy()
        # Create Shader
        self.shaderId = gl.glCreateShader(self.type)
        gl.glShaderSource(self.shaderId, self.src)

    def compile(self):
        gl.glCompileShader(self.shaderId)
        result = gl.glGetShaderiv(self.shaderId, gl.GL_COMPILE_STATUS)
        if result != 1:
            sys.stderr.write(gl.glGetShaderInfoLog(self.shaderId)+'\n')

    def mapUniformLocations(self, programId):
        """ This function must be called only after the shaders has been attached
            to the program
        """
        for name, u in self.uniforms.items():
            # Be careful! JSON Strings are unicode
            u.loc = gl.glGetUniformLocation(programId, str(name)),

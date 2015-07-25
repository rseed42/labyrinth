import OpenGL.GL as gl
import bunch
#-------------------------------------------------------------------------------
# Shader Proogram
#-------------------------------------------------------------------------------
class ShaderProgram(object):
    def __init__(self):
        self.programId = gl.glCreateProgram()
        self.attachedShaders = {}
        self.uniforms = bunch.Bunch()

    def link(self):
        gl.glLinkProgram(self.programId)

    def attachShader(self, name, shader):
        gl.glAttachShader(self.programId, shader.shaderId)
        self.attachedShaders[name] = shader

    def detachShader(self, name):
        gl.glDetachShader(self.programId,self.attachedShaders[name].shaderId)
        self.attachedShaders.pop(name)

    def mapUniformLocations(self):
        """ Aggregate the uniform locations from the shaders
        """
        for name, shader in self.attachedShaders.items():
            for name, u in shader.uniforms.items():
                # Be careful! JSON Strings are unicode
                u.loc = gl.glGetUniformLocation(self.programId, str(name))
                self.uniforms[name] = u

    def use(self):
        try:
            gl.glUseProgram(self.programId)
        except OpenGL.error.GLError:
            sys.stderr.write(gl.glGetProgramInfoLog(self.programId)+'\n')

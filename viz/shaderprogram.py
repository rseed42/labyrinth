import OpenGL.GL as gl
#-------------------------------------------------------------------------------
# Shader Proogram
#-------------------------------------------------------------------------------
class ShaderProgram(object):
    def __init__(self):
        self.programId = gl.glCreateProgram()
        self.attachedShaders = {}

    def link(self):
        gl.glLinkProgram(self.programId)

    def attachShader(self, name, shader):
        gl.glAttachShader(self.programId, shader.shaderId)
        self.attachedShaders[name] = shader

    def detachShader(self, name):
        gl.glDetachShader(self.programId,self.attachedShaders[name].shaderId)
        self.attachedShaders.pop(name)

    def mapUniformLocations(self):
        for name, shader in self.attachedShaders.items():
            shader.mapUniformLocations(self.programId)

    def use(self):
        try:
            gl.glUseProgram(self.programId)
        except OpenGL.error.GLError:
            sys.stderr.write(gl.glGetProgramInfoLog(self.programId)+'\n')

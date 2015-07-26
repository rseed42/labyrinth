import sdl2 as sdl
import OpenGL.GL as gl
import numpy as np
import shader
import shaderprogram
import bunch
import visual
import avatar
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
        self.worldView = True
        self.matModelView = np.identity(4,'f')
        self.matView = np.identity(4,'f')
        self.projRot = np.identity(4,'f')
        self.projTrans = np.identity(4,'f')
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
            vObj.configure(visCfg)
            self.visStatic[name] = vObj
            vObj.verticesFromFixtures(sObj.body.fixtures)

#        for name, dObj in dynamicsObjects.items():
#            print name

        for name, agent in agents.items():
            agtCfg = cfg.agents.get(name)
            av = avatar.Avatar()
            av.configure(agtCfg)
            self.visAgents[name] = av
            av.setAgent(agent)
            av.calculateVertices()

#        print self.programs['default'].uniforms['vec_Color']

    def createAvatar(self):
        pass

    def render(self, window, userController):
        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
        # Render scene
        # Currently using the same shaders for all objects
        prog = self.programs['default']
        prog.use()
        # Set Projection Matrix
        if self.worldView == 0 and userController != None:
            pos = userController.agent.body.transform.position
            self.projTrans[0,3] = pos[0]
            self.projTrans[0,1] = pos[1]
            R = userController.agent.body.transform.R
            self.projRot[0,0] = R.col1.x
            self.projRot[1,0] = R.col1.y
            self.projRot[0,1] = R.col2.x
            self.projRot[1,1] = R.col2.y
            self.matView = self.projTrans
#            self.matView = np.dot(self.projTrans, self.projRot)
#            self.matProj = np.dot(self.projTrans, self.projRot)
        prog.setUniform('mat_ModelView', self.matModelView)
        prog.setUniform('mat_View', self.matView)
        prog.setUniform('mat_Proj', self.matProj)
        # Render static objects in one go
        gl.glEnableClientState(gl.GL_VERTEX_ARRAY)
        for name, vObj in self.visStatic.items():
            vObj.draw(prog)

        for name, ag in self.visAgents.items():
            ag.draw(prog)

        gl.glDisableClientState(gl.GL_VERTEX_ARRAY)

        gl.glUseProgram(0)
        #
        sdl.SDL_GL_SwapWindow(window)

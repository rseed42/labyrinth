from conf import msg
try:
    import Box2D as b2
except ImportError:
    sys.stderr.write(msg.import_box2d_fail+msg.newline)
    sys.exit(1)
#CAR_STARTING_POS = b2.b2Vec2(0,0)
#
#MAX_STEER_ANGLE = b2.b2_pi/3
#STEER_SPEED = 1.5
#SIDEWAYS_FRICTION_FORCE = 10
#HORSEPOWER = 40
#
#MAZE_FILENAME = "maze.bin"
#-------------------------------------------------------------------------------
# Main Simulation Object
#-------------------------------------------------------------------------------
class Agent(object):
    def __init__(self):
        # Control variables
        self.engineSpeed = 0
        self.steeringAngle = 0
        self.colorChassis = None
        self.colorBorder = None

    def addWheel(self, world, carBody, carPos, cfg):
        wheelDef = b2.b2BodyDef()
        wheelDef.type = b2.b2_dynamicBody
        wheelDef.position = b2.b2Vec2(carPos) + b2.b2Vec2(cfg.position)
        wheel = world.CreateBody(wheelDef)
        wheel.CreatePolygonFixture(box=(cfg.size.width, cfg.size.height),
                                       friction=cfg.friction,
                                       density=cfg.density)
        return wheel

    def construct(self, cfg, world):
        # Update config
        self.colorChassis = cfg.color.chassis
        self.colorBorder = cfg.color.border
        # Agent body
        bodyDef = b2.b2BodyDef()
        bodyDef.type = b2.b2_dynamicBody
        bodyDef.linearDamping = cfg.linearDamping
        bodyDef.angularDamping = cfg.angularDamping
        bodyDef.position = cfg.position
        self.body = world.CreateBody(bodyDef)
        self.body.CreatePolygonFixture(box=(cfg.size.width, cfg.size.height),
                                       friction=cfg.friction,
                                       density=cfg.density)

        self.frontLeftWheel = self.addWheel(world,
                                            self.body,
                                            cfg.position,
                                            cfg.wheels.frontLeft)
        self.frontRightWheel = self.addWheel(world,
                                            self.body,
                                            cfg.position,
                                            cfg.wheels.frontRight)
        self.rearLeftWheel = self.addWheel(world,
                                            self.body,
                                            cfg.position,
                                            cfg.wheels.rearLeft)
        self.rearRightWheel = self.addWheel(world,
                                            self.body,
                                            cfg.position,
                                            cfg.wheels.rearRight)

#        leftJointDef = b2.b2RevoluteJointDef()
#        leftJointDef.Initialize(self.body,
#                                self.leftWheel,
#                                self.leftWheel.GetWorldCenter())
#        leftJointDef.enableMotor = True
#        leftJointDef.maxMotorTorque = 100
#
#        rightJointDef = b2.b2RevoluteJointDef()
#        rightJointDef.Initialize(self.body,
#                                 self.rightWheel,
#                                 self.rightWheel.GetWorldCenter())
#        rightJointDef.enableMotor = True
#        rightJointDef.maxMotorTorque = 100
#
##        leftJoint = b2.b2RevoluteJoint(world.CreateJoint(leftJointDef))
#        self.leftJoint = world.CreateJoint(leftJointDef)
##        rightJoint = b2.b2RevoluteJoint(world.CreateJoint(rightJointDef))
#        self.rightJoint = world.CreateJoint(rightJointDef)
#
#        leftRearJointDef = b2.b2PrismaticJointDef()
#        leftRearJointDef.Initialize(self.body,
#                                    self.leftRearWheel,
#                                    self.leftRearWheel.GetWorldCenter(),
#                                    b2.b2Vec2(1,0))
#        leftRearJointDef.enableLimit = True
#        leftRearJointDef.lowerTranslation = leftRearJointDef.upperTranslation = 0
#
#        rightRearJointDef = b2.b2PrismaticJointDef()
#        rightRearJointDef.Initialize(self.body,
#                                     self.rightRearWheel,
#                                     self.rightRearWheel.GetWorldCenter(),
#                                     b2.b2Vec2(1,0))
#        rightRearJointDef.enableLimit = True
#        rightRearJointDef.lowerTranslation = 0
#        rightRearJointDef.upperTranslation = 0
#
#        world.CreateJoint(leftRearJointDef)
#        world.CreateJoint(rightRearJointDef)
#
#    def steerLeft(self):
#        self.steeringAngle = MAX_STEER_ANGLE
#
#    def steerRight(self):
#        self.steeringAngle = -MAX_STEER_ANGLE
#
    def accelerate(self):
        print "accelerate"
        pass

    def break_(self):
        print "break"
        pass

    def turnLeft(self):
        print "turnLeft"
        pass

    def turnRight(self):
        print "turnRight"
        pass

#
#    def deccelerate(self):
#        self.engineSpeed = -HORSEPOWER
#
#    def stopEngine(self):
#        self.engineSpeed = 0
#
#    def neutralSteering(self):
#        self.steeringAngle = 0
#
#    def killOrthogonalVelocity(self, body):
#        """ This function applies a "friction" in a direction orthogonal to the
#            body's axis. """
#        localPoint = b2.b2Vec2(0,0)
#        velocity = body.GetLinearVelocityFromLocalPoint(localPoint)
#        sidewaysAxis = body.GetXForm().R.col2.copy()
#        sidewaysAxis.mul_float(b2.b2Dot(velocity, sidewaysAxis))
#        body.SetLinearVelocity(sidewaysAxis)
#
#
    def update(self):
        pass
#        self.killOrthogonalVelocity(self.leftWheel)
#        self.killOrthogonalVelocity(self.rightWheel)
#        self.killOrthogonalVelocity(self.leftRearWheel)
#        self.killOrthogonalVelocity(self.rightRearWheel)
#        # Driving
#        ldirection = self.leftWheel.GetXForm().R.col2.copy()
#        ldirection.mul_float(self.engineSpeed)
#        rdirection = self.rightWheel.GetXForm().R.col2.copy()
#        rdirection.mul_float(self.engineSpeed)
#        self.leftWheel.ApplyForce(ldirection, self.leftWheel.GetPosition())
#        self.rightWheel.ApplyForce(rdirection, self.rightWheel.GetPosition())
#        # Steering
#        lj = self.leftJoint.asRevoluteJoint()
#        mspeed = self.steeringAngle - lj.GetJointAngle()
#        lj.SetMotorSpeed(mspeed * STEER_SPEED)
#        rj = self.rightJoint.asRevoluteJoint()
#        mspeed = self.steeringAngle - rj.GetJointAngle()
#        rj.SetMotorSpeed(mspeed * STEER_SPEED)


    def drawBox(self, gl, colFill, colBorder, verts):
        """ Vertices are presupplied, in world coordinates
        """
        # Kinda inefficient, but we are not worried about performance yet
        vertices = verts[:3] + verts[2:] + [verts[0]]
        verticesBorder = [v for v in verts] + [verts[0]]
        # Box
        gl.glColor3f(*self.colorChassis)
        gl.glBegin(gl.GL_TRIANGLES)
        for v in vertices:
            gl.glVertex2f(*v)
        gl.glEnd()
        # Border
        gl.glColor3f(*self.colorBorder)
        gl.glBegin(gl.GL_LINE_STRIP)
        for v in verticesBorder:
            gl.glVertex2f(*v)
        gl.glEnd()

    def draw(self, gl):
        # Display the chassis
        self.drawBox(gl, self.colorChassis, self.colorBorder,
             map(self.body.GetWorldPoint, self.body.fixtures[0].shape.vertices))
        self.drawBox(gl, self.colorChassis, self.colorBorder,
             map(self.frontLeftWheel.GetWorldPoint,
                 self.frontLeftWheel.fixtures[0].shape.vertices))
        self.drawBox(gl, self.colorChassis, self.colorBorder,
             map(self.frontRightWheel.GetWorldPoint,
                 self.frontRightWheel.fixtures[0].shape.vertices))
        self.drawBox(gl, self.colorChassis, self.colorBorder,
             map(self.rearLeftWheel.GetWorldPoint,
                 self.rearLeftWheel.fixtures[0].shape.vertices))
        self.drawBox(gl, self.colorChassis, self.colorBorder,
             map(self.rearRightWheel.GetWorldPoint,
                 self.rearRightWheel.fixtures[0].shape.vertices))

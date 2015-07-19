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
#    def __init__(self, world):
    def __init__(self):
        pass
#        self.engineSpeed = 0
#        self.steeringAngle = 0
#
#        bodyDef = b2.b2BodyDef()
#        bodyDef.linearDamping = 1;
#        bodyDef.angularDamping = 1;
#        bodyDef.position = CAR_STARTING_POS
#        self.body = world.CreateBody(bodyDef)
#        self.body.SetMassFromShapes()
#
#        leftRearWheelPosition = b2.b2Vec2(-1.5,1.90)
#        rightRearWheelPosition = b2.b2Vec2(1.5,1.9)
#        leftFrontWheelPosition = b2.b2Vec2(-1.5,-1.9)
#        rightFrontWheelPosition = b2.b2Vec2(1.5,-1.9)
#
#        leftWheelDef = b2.b2BodyDef();
#        leftWheelDef.position = CAR_STARTING_POS
#        leftWheelDef.position.add_vector(leftFrontWheelPosition)
#        self.leftWheel = world.CreateBody(leftWheelDef)
#
#        rightWheelDef = b2.b2BodyDef()
#        rightWheelDef.position = CAR_STARTING_POS
#        rightWheelDef.position.add_vector(rightFrontWheelPosition)
#        self.rightWheel = world.CreateBody(rightWheelDef)


#
#        leftRearWheelDef = b2.b2BodyDef()
#        leftRearWheelDef.position = CAR_STARTING_POS
#        leftRearWheelDef.position.add_vector(leftRearWheelPosition)
#        self.leftRearWheel = world.CreateBody(leftRearWheelDef)
#
#        rightRearWheelDef = b2.b2BodyDef()
#        rightRearWheelDef.position = CAR_STARTING_POS
#        rightRearWheelDef.position.add_vector(rightRearWheelPosition)
#        self.rightRearWheel = world.CreateBody(rightRearWheelDef)
#
#        # Shapes
#        boxDef = b2.b2PolygonDef()
#        boxDef.SetAsBox(1.5,2.5)
#        boxDef.density = 1
#        self.body.CreateShape(boxDef)
#
#        # Left Wheel shape
#        leftWheelShapeDef = b2.b2PolygonDef()
#        leftWheelShapeDef.SetAsBox(0.2,0.5)
#        leftWheelShapeDef.density = 1
#        self.leftWheel.CreateShape(leftWheelShapeDef)
#
#        # Right Wheel shape
#        rightWheelShapeDef = b2.b2PolygonDef()
#        rightWheelShapeDef.SetAsBox(0.2,0.5)
#        rightWheelShapeDef.density = 1
#        self.rightWheel.CreateShape(rightWheelShapeDef)
#
#        # Left Wheel shape
#        leftRearWheelShapeDef = b2.b2PolygonDef()
#        leftRearWheelShapeDef.SetAsBox(0.2,0.5)
#        leftRearWheelShapeDef.density = 1
#        self.leftRearWheel.CreateShape(leftRearWheelShapeDef)
#
#        # Right Wheel shape
#        rightRearWheelShapeDef = b2.b2PolygonDef()
#        rightRearWheelShapeDef.SetAsBox(0.2,0.5)
#        rightRearWheelShapeDef.density = 1
#        self.rightRearWheel.CreateShape(rightRearWheelShapeDef)
#
#        self.body.SetMassFromShapes()
#        self.leftWheel.SetMassFromShapes()
#        self.rightWheel.SetMassFromShapes()
#        self.leftRearWheel.SetMassFromShapes()
#        self.rightRearWheel.SetMassFromShapes()
#
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
#    def accelerate(self):
#        self.engineSpeed = HORSEPOWER
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
#    def update(self):
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

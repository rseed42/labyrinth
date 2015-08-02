import Box2D as b2
import numpy as np
from staticobject import StaticObject
from mind import programed
#-------------------------------------------------------------------------------
# Wheel
#-------------------------------------------------------------------------------
WDC_LEFT  = 0x1
WDC_RIGHT = 0x2
WDC_UP    = 0x4
WDC_DOWN  = 0x8
DEGTORAD = 0.0174532925199432957
RADTODEG = 57.295779513082320876
#-------------------------------------------------------------------------------
# Wheel
#-------------------------------------------------------------------------------
class Wheel(object):
    def __init__(self):
        self.body = None
        self.maxForwardSpeed = 0
        self.maxBackwardSpeed = 0
        self.maxDriveForce = 0
        self.maxLateralImpulse = 0

    def create(self, world, carBody, carPos, cfg):
        self.maxForwardSpeed = cfg.maxForwardSpeed
        self.maxBackwardSpeed = cfg.maxBackwardSpeed
        self.maxDriveForce = cfg.maxDriveForce
        self.maxLateralImpulse = cfg.maxLateralImpulse
        bodyDef = b2.b2BodyDef()
        bodyDef.type = b2.b2_dynamicBody
        bodyDef.position = b2.b2Vec2(carPos) + b2.b2Vec2(cfg.position)
        self.body = world.CreateBody(bodyDef)
#        polygonShape = b2.b2PolygonShape()
#        polygonShape.SetAsBox(0.5,1.25)
        fx = self.body.CreatePolygonFixture(box=(0.5, 1.25), density=1)
        # Density = 1
#        self.body.CreateFixture(polygonShape, 1)

#    def setCharacteristics(self, forward, back, drive):
#        self.maxForwardSpeed = forward
#        self.maxBackwardSpeed = back
#        self.maxDriveForce = drive

    def getLateralVelocity(self):
        currentRightNormal = self.body.GetWorldVector((1,0))
        return b2.b2Dot(currentRightNormal, self.body.linearVelocity) * currentRightNormal

    def getForwardVelocity(self):
        currentForwardNormal = self.body.GetWorldVector((0,1))
        return b2.b2Dot(currentForwardNormal, self.body.linearVelocity) * currentForwardNormal

    def updateFriction(self):
        # Lateral linear velocity
        maxLateralImpulse = 2.5
        impulse = self.body.mass * -self.getLateralVelocity()
        if impulse.length > maxLateralImpulse:
            impulse *= maxLateralImpulse / impulse.length
        self.body.ApplyLinearImpulse(impulse, self.body.worldCenter, True)
        # Angular velocity
        self.body.ApplyAngularImpulse(0.1*self.body.inertia*-self.body.angularVelocity, True)
        # Forward linear velocity
        currentForwardNormal = self.getForwardVelocity()
        currentForwardSpeed = currentForwardNormal.Normalize()
        dragForceMagnitude = -2 * currentForwardSpeed
        self.body.ApplyForce(dragForceMagnitude * currentForwardNormal,
                             self.body.worldCenter, True)

#    def updateDrive(self, controlState):
    def updateDrive(self, controlState):
#        print bin(controlState)
#        return
        desiredSpeed = 0
#        print bin(self.controlState)
#        return
        if controlState & WDC_UP: desiredSpeed = self.maxForwardSpeed
        if controlState & WDC_DOWN: desiredSpeed = self.maxBackwardSpeed
#        else: return
#        print desiredSpeed
        # Current speed in forward direction
        currentForwardNormal = self.body.GetWorldVector((0,1))
        currentSpeed = b2.b2Dot(self.getForwardVelocity(), currentForwardNormal)
        # Apply necessary force
        force = 0
        if desiredSpeed > currentSpeed:
            force = self.maxDriveForce
        elif desiredSpeed < currentSpeed:
            force = -self.maxDriveForce
        else:
            return
        self.body.ApplyForce(force*currentForwardNormal,
                             self.body.worldCenter,
                             True)

#    def updateTurn(self, controlState):
    def updateTurn(self, controlState):
        desiredTorque = 0
#        print self.controlState
#        return
        if controlState & (WDC_LEFT|WDC_RIGHT):
            if controlState & WDC_LEFT:
                desiredTorque = 15
            elif controlState & WDC_RIGHT:
                desiredTorque = -15
            else:
                pass
        self.body.ApplyTorque(desiredTorque, True)

#-------------------------------------------------------------------------------
# Agent
#-------------------------------------------------------------------------------
class Agent(object):
    def __init__(self, name):
        self.name = name
        # Control variables
#        self.max_engine_speed = 0
#        self.reverse_engine_speed = 0
#        self.reverse_engine_acc_step = 0
#        self.acceleration_step = 0
#        self.engineSpeed = 0
#        self.steering_speed = 0
#        self.max_steer_angle = 0
#        self.steer_angle_step = 0
#        self.steeringAngle = 0
#        # Bodies
        self.body = None
        self.frontLeftWheel = Wheel()
        self.frontRightWheel = Wheel()
        self.rearLeftWheel = Wheel()
        self.rearRightWheel = Wheel()
        self.wheels = (self.frontLeftWheel, self.frontRightWheel,
                       self.rearLeftWheel,  self.rearRightWheel)
        self.flJoint = None
        self.frJoint = None
#        # Sensors
#        self.sensor = None
#        self.fov = None
#        self.sensorField = {}
#        self.frameNum = 0
        # Mind
        self.mind = None
        self.controlState = 0x0

    def addWheel(self, wheel, world, carBody, carPos, cfg):
        wheel.create(world, carBody, carPos, cfg)
        jointDef = b2.b2RevoluteJointDef()
        jointDef.bodyA = self.body
        jointDef.enableLimit = True
        jointDef.lowerAngle = 0
        jointDef.upperAngle = 0
        # Center of wheel
        jointDef.localAnchorB.SetZero()
        jointDef.bodyB = wheel.body
        jointDef.localAnchorA.Set(*cfg.anchor)
        return world.CreateJoint(jointDef)

#    def addSensor(self, world, cfg):
#        sensorFov = b2.b2PolygonShape()
#        # Define sensor shape
#        w, h = cfg.fov.width, cfg.fov.height
#        self.fov = (w,h)
#        fov = np.array([(-0.5*w,-0.5*h),(0.5*w,-0.5*h),
#                        (0.5*w,0.5*h),(-0.5*w, 0.5*h)])
#        # Move sensor relative to the body
#        relpos = np.array([cfg.relpos.x, cfg.relpos.y])
#        sensorFov.vertices = (fov+relpos).tolist()
#        sensorFixtureDef = b2.b2FixtureDef()
#        sensorFixtureDef.isSensor = True
#        sensorFixtureDef.shape = sensorFov
#        self.sensor = self.body.CreateFixture(sensorFixtureDef)

    def construct(self, world, cfg):
        # Initialize the control variables
        pass
#        self.max_engine_speed = cfg.max_engine_speed
#        self.max_steer_angle = (b2.b2_pi/180)*cfg.max_steer_angle
#        self.steering_speed = cfg.steering_speed
#        self.acceleration_step = cfg.acceleration_step
#        self.reverse_engine_max_speed = cfg.reverse_engine_max_speed
#        self.reverse_engine_acc_step = cfg.reverse_engine_acc_step
#        self.steer_angle_step = cfg.steer_angle_step
        # Agent body
        bodyDef = b2.b2BodyDef()
        bodyDef.type = b2.b2_dynamicBody
        bodyDef.linearDamping = cfg.linearDamping
        bodyDef.angularDamping = cfg.angularDamping
        bodyDef.position = cfg.position
        self.body = world.CreateBody(bodyDef)
#        self.body.userData = self
        verts = ((1.5,0.0),
                 (3.0,2.5),
                 (2.8,5.5),
                 (1.0,10.0),
                 (-1.0,10.0),
                 (-2.8,5.5),
                 (-3.0,2.5),
                 (-1.5,0.0)
        )
        carShape = b2.b2PolygonShape()
        carShape.vertices = verts
        carShape.vertexCount = len(verts)
#        fixture = self.body.CreateFixture(carShape, 0.1)
        fx = self.body.CreatePolygonFixture(shape=carShape,
                                       friction=cfg.friction,
                                       density=cfg.density,
                                       restitution=cfg.restitution
       )

#        fx = self.body.CreatePolygonFixture(box=(cfg.size.width, cfg.size.height),
#                                       friction=cfg.friction,
#                                       density=cfg.density,
#                                       restitution=cfg.restitution
#        )
        self.flJoint = self.addWheel(self.frontLeftWheel,
                      world,
                      self.body,
                      cfg.position,
                      cfg.wheels.frontLeft
        )
        self.frJoint = self.addWheel(self.frontRightWheel,
                      world,
                      self.body,
                      cfg.position,
                      cfg.wheels.frontRight
        )
        self.addWheel(self.rearLeftWheel,
                      world,
                      self.body,
                      cfg.position,
                      cfg.wheels.rearLeft
        )
        self.addWheel(self.rearRightWheel,
                      world,
                      self.body,
                      cfg.position,
                      cfg.wheels.rearRight
        )


#        self.frontLeftWheel = self.addFrontWheel(world, self.body,
#                                                 cfg.position,
#                                                 cfg.wheels.frontLeft)
#        self.frontRightWheel = self.addFrontWheel(world, self.body,
#                                                  cfg.position,
#                                                  cfg.wheels.frontRight)
#
#        self.rearLeftWheel = self.addRearWheel(world, self.body,
#                                                  cfg.position,
#                                                  cfg.wheels.rearLeft)
#        self.rearRightWheel = self.addRearWheel(world, self.body,
#                                                  cfg.position,
#                                                  cfg.wheels.rearRight)
#        # Create Sensor
#        self.addSensor(world, cfg.sensor)
#        # Create the mind or remain mindless
#        if not cfg.mind: return
#        self.mind = programed.MindProgram()
#        self.mind.configure(cfg.mind, self)

#    def accelerate(self):
#        if self.engineSpeed < self.max_engine_speed:
#            self.engineSpeed += self.acceleration_step
#
#    def releaseAccelerator(self):
#        self.engineSpeed = 0
#
#    def reverse(self):
#        if self.engineSpeed < self.reverse_engine_max_speed:
##            print 'reverse'
#            self.engineSpeed -= self.reverse_engine_acc_step
#
#    def releaseReverse(self):
#        self.engineSpeed = 0
#
#    def brake(self):
#        pass
#        #print "brake"
#
#    def releaseBrake(self):
#        pass
#        #print "releaseBrake"
#
#    def steerLeft(self):
#        self.steeringAngle = -self.max_steer_angle
#
#    def steerRight(self):
#        self.steeringAngle = self.max_steer_angle
#
#    def releaseSteering(self):
#        self.steeringAngle = 0
#

    def update(self):
        # Decision-making time
        if self.mind: self.mind.think()
        # Put our actions into physics
        for wheel in self.wheels:
            wheel.updateFriction()
            wheel.updateDrive(self.controlState)



        lockAngle = 35*DEGTORAD
        turnSpeedPerSec = 160 * DEGTORAD
        turnPerTimeStep = turnSpeedPerSec / 60.0
        desiredAngle = 0

        if self.controlState & WDC_LEFT:
            desiredAngle = lockAngle
        elif self.controlState & WDC_RIGHT:
            desiredAngle = -lockAngle
        angleToTurn = desiredAngle - self.flJoint.angle
        # Some problem with the API, therefore workaround:
        angleToTurn = b2.b2Clamp((angleToTurn,0),
                                 (-turnPerTimeStep,0),
                                 (turnPerTimeStep,0)).x
        newAngle = self.flJoint.angle + angleToTurn
        self.flJoint.limits = (newAngle, newAngle)
        self.frJoint.limits = (newAngle, newAngle)



    def handleCollisionBegin(self, contact, fixtureMe, fixtureOther):
        """ Collision events that do not come from the sensor,
            meaning that the agent has collided with something.
            Can be used to simulate damage, pain, etc.
        """
        # Will check if this is a sensor first
        if fixtureMe.sensor:
#            print '%d, b: %d' % (self.frameNum, id(fixtureOther))
            self.sensorField[fixtureOther] = {}
            return

        if fixtureOther.sensor: return
        obj = fixtureOther.body.userData
        # The other object is an agent
        if isinstance(obj, Agent):
            pass
#            print '%s - %s' % (self.name, obj.name)
        elif isinstance(obj, StaticObject):
            pass
#            print '%s - sObj(%d)' % (self.name, obj.id)
        # Either an error or an unknown body type without user data
        else:
            pass


    def handleCollisionEnd(self, contact, fixtureMe, fixtureOther):
        """ Collision events that do not come from the sensor,
            meaning that the agent has collided with something.
            Can be used to simulate damage, pain, etc.
            It is the *touch* sensor.
        """
        # Will check if this is a sensor first
        if fixtureMe.sensor:
#            print '%d, e: %d' % (self.frameNum, id(fixtureOther))
            self.sensorField.pop(fixtureOther)
            return

        if fixtureOther.sensor: return
        obj = fixtureOther.body.userData
        # The other object is an agent
        if isinstance(obj, Agent):
            pass
#            print '%s - %s' % (self.name, obj.name)
        elif isinstance(obj, StaticObject):
            pass
#            print '%s - sObj(%d)' % (self.name, obj.id)
        # Either an error or an unknown body type without user data
        else:
            pass

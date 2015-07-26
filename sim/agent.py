import Box2D as b2
import numpy as np
from staticobject import StaticObject
#-------------------------------------------------------------------------------
# Agent
#-------------------------------------------------------------------------------
class Agent(object):
    def __init__(self, name):
        self.name = name
        # Control variables
        self.max_engine_speed = 0
        self.reverse_engine_speed = 0
        self.reverse_engine_acc_step = 0
        self.acceleration_step = 0
        self.engineSpeed = 0
        self.steering_speed = 0
        self.max_steer_angle = 0
        self.steer_angle_step = 0
        self.steeringAngle = 0
        # Bodies
        self.body = None
        self.frontLeftWheel = None
        self.frontRightWheel = None
        self.rearLeftWheel = None
        self.rearRightWheel = None
        # Sensors
        self.sensor = None

    def addWheel(self, world, carBody, carPos, cfg):
        wheelDef = b2.b2BodyDef()
        wheelDef.type = b2.b2_dynamicBody
        wheelDef.position = b2.b2Vec2(carPos) + b2.b2Vec2(cfg.position)
        wheel = world.CreateBody(wheelDef)
        # For collision detection, the wheel is part of the car:
        wheel.userData = self
        fx = wheel.CreatePolygonFixture(box=(cfg.size.width, cfg.size.height),
                                       friction=cfg.friction,
                                       density=cfg.density,
                                       restitution=cfg.restitution
        )
        return wheel

    def addFrontWheel(self, world, carBody, carPos, cfg):
        wheel = self.addWheel(world, carBody, carPos, cfg)
        jointDef = b2.b2RevoluteJointDef()
        jointDef.Initialize(carBody, wheel, wheel.worldCenter)
        jointDef.enableMotor = True
        jointDef.maxMotorTorque = cfg.max_motor_torque
        joint = world.CreateJoint(jointDef)
        return wheel

    def addRearWheel(self, world, carBody, carPos, cfg):
        wheel = self.addWheel(world, carBody, carPos, cfg)
        jointDef = b2.b2PrismaticJointDef()
        jointDef.Initialize(self.body,
                            wheel,
                            wheel.worldCenter,
                            b2.b2Vec2(1,0))
        jointDef.enableLimit = True
        jointDef.lowerTranslation = 0
        joint = world.CreateJoint(jointDef)
        return wheel

    def addSensor(self, world, cfg):
        sensorFov = b2.b2PolygonShape()
        # Define sensor shape
        w, h = cfg.fov.width, cfg.fov.height
        fov = np.array([(-0.5*w,-0.5*h),(0.5*w,-0.5*h),
                        (0.5*w,0.5*h),(-0.5*w, 0.5*h)])
        # Move sensor relative to the body
        relpos = np.array([cfg.relpos.x, cfg.relpos.y])
        sensorFov.vertices = (fov+relpos).tolist()
        sensorFixtureDef = b2.b2FixtureDef()
        sensorFixtureDef.isSensor = True
        sensorFixtureDef.shape = sensorFov
        self.sensor = self.body.CreateFixture(sensorFixtureDef)

    def construct(self, world, cfg):
        # Initialize the control variables
        self.max_engine_speed = cfg.max_engine_speed
        self.max_steer_angle = (b2.b2_pi/180)*cfg.max_steer_angle
        self.steering_speed = cfg.steering_speed
        self.acceleration_step = cfg.acceleration_step
        self.reverse_engine_max_speed = cfg.reverse_engine_max_speed
        self.reverse_engine_acc_step = cfg.reverse_engine_acc_step
        self.steer_angle_step = cfg.steer_angle_step
        # Agent body
        bodyDef = b2.b2BodyDef()
        bodyDef.type = b2.b2_dynamicBody
        bodyDef.linearDamping = cfg.linearDamping
        bodyDef.angularDamping = cfg.angularDamping
        bodyDef.position = cfg.position
        self.body = world.CreateBody(bodyDef)
        self.body.userData = self
        fx = self.body.CreatePolygonFixture(box=(cfg.size.width, cfg.size.height),
                                       friction=cfg.friction,
                                       density=cfg.density,
                                       restitution=cfg.restitution
        )
        self.frontLeftWheel = self.addFrontWheel(world, self.body,
                                                 cfg.position,
                                                 cfg.wheels.frontLeft)
        self.frontRightWheel = self.addFrontWheel(world, self.body,
                                                  cfg.position,
                                                  cfg.wheels.frontRight)

        self.rearLeftWheel = self.addRearWheel(world, self.body,
                                                  cfg.position,
                                                  cfg.wheels.rearLeft)
        self.rearRightWheel = self.addRearWheel(world, self.body,
                                                  cfg.position,
                                                  cfg.wheels.rearRight)
        # Create Sensor
        self.addSensor(world, cfg.sensor)

    def accelerate(self):
        if self.engineSpeed < self.max_engine_speed:
            self.engineSpeed += self.acceleration_step

    def releaseAccelerator(self):
        self.engineSpeed = 0

    def reverse(self):
        if self.engineSpeed < self.reverse_engine_max_speed:
            self.engineSpeed -= self.reverse_engine_acc_step

    def releaseReverse(self):
        self.engineSpeed = 0

    def brake(self):
        pass
        #print "brake"

    def releaseBrake(self):
        pass
        #print "releaseBrake"

    def steerLeft(self):
        self.steeringAngle = -self.max_steer_angle

    def steerRight(self):
        self.steeringAngle = self.max_steer_angle

    def releaseSteering(self):
        self.steeringAngle = 0

    def killOrthogonalVelocity(self, body):
        """ This function applies a "friction" in a direction orthogonal to the
            body's axis. """
        localPoint = b2.b2Vec2(0,0)
        velocity = body.GetLinearVelocityFromLocalPoint(localPoint)
        sidewaysAxis = body.transform.R.col2
        sidewaysAxis *= b2.b2Dot(velocity, sidewaysAxis)
        body.linearVelocity = sidewaysAxis

    def update(self):
        self.killOrthogonalVelocity(self.frontLeftWheel)
        self.killOrthogonalVelocity(self.frontRightWheel)
        self.killOrthogonalVelocity(self.rearLeftWheel)
        self.killOrthogonalVelocity(self.rearRightWheel)
        # Driving
        lforce = self.frontLeftWheel.transform.R.col2 * self.engineSpeed
        self.frontLeftWheel.ApplyForceToCenter(lforce, True)
        rforce = self.frontRightWheel.transform.R.col2 * self.engineSpeed
        self.frontRightWheel.ApplyForceToCenter(rforce, True)
        # Steering
        lj = self.frontLeftWheel.joints[0].joint
        mspeed = self.steeringAngle - lj.angle
        lj.motorSpeed = mspeed * self.steering_speed

        rj = self.frontRightWheel.joints[0].joint
        mspeed = self.steeringAngle - rj.angle
        rj.motorSpeed = mspeed * self.steering_speed

    def handleCollision(self, contact, fixtureMe, fixtureOther):
        """ Collision events that do not come from the sensor,
            meaning that the agent has collided with something.
            Can be used to simulate damage, pain, etc.
        """
        # Will check if this is a sensor first
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

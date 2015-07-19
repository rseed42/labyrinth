import sys
from conf import msg
try:
    import Box2D as b2
except ImportError:
    sys.stderr.write(msg.import_box2d_fail+msg.newline)
    sys.exit(1)
try:
    import bunch
except ImportError:
    sys.stderr.write(msg.import_bunch_fail+msg.newline)
    sys.exit(1)

from loader import MapLoader
#-------------------------------------------------------------------------------
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
#class AgentBody(object):
#    def __init__(self, world):
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

class Simulation(object):
    def __init__(self):
        # Non-interactive simulation:
        self.running = False
        # Solver params
        self.timestep = 0
        self.velocityIterations = 0
        self.positionIterations = 0
        # World parameters
        self.worldLowerBound = None
        self.worldUpperBound = None
#        worldAABB = b2.b2AABB()
#        worldAABB.lowerBound = (-10, -10)
#        worldAABB.upperBound = (10, 10)
#        worldAABB.lowerBound = (-40, -40)
#        worldAABB.upperBound = (40, 40)

#        gravity = (0,0)
#        doSleep = True
#        self.world = b2.b2World(worldAABB, gravity, doSleep)
        # Add maze
#        self.mapLoader = MapLoader()
#        self.mapLoader.load(MAZE_FILENAME)
#        scale = 1.
#        trans = (-40, -40)
#        for wall in self.mapLoader.walls:
#            for brick in wall:
#                brickBodyDef = b2.b2BodyDef()
#                brickBodyDef.position = (brick[0]*scale+trans[0],
#                                         brick[1]*scale+trans[1])
#                brickBody = self.world.CreateBody(brickBodyDef)
#                brickShapeDef = b2.b2PolygonDef()
#                brickShapeDef.SetAsBox(0.5*scale, 0.5*scale)
#                brickBody.CreateShape(brickShapeDef)

        # Add ground
#        groundBodyDef = b2.b2BodyDef()
#        groundBodyDef.position = (0,-9)
#        self.groundBody = self.world.CreateBody(groundBodyDef)
#        groundShapeDef = b2.b2PolygonDef()
#        groundShapeDef.SetAsBox(8, 0.25)
#        self.groundBody.CreateShape(groundShapeDef)
#        self.groundBody.angle = 8*(b2.b2_pi/180)
#        # Static shapes
#        obstacleBodyDef = b2.b2BodyDef()
#        obstacleBodyDef.position = (0,4)
#        obstacleBody = self.world.CreateBody(obstacleBodyDef)
#        obstacleShapeDef = b2.b2PolygonDef()
#        verts = ((-5,-0.2), (1,-0.2), (3,3), (2.8,3.3), (2, 1.8), (-5,0.2))
#        obstacleShapeDef.setVertices(verts)
#        obstacleBody.CreateShape(obstacleShapeDef)

        # Add dynamic bodies
#        bodyDef = b2.b2BodyDef()
#        bodyDef.position = (0,0)
#
#        self.body = self.world.CreateBody(bodyDef)
#        shapeDef = b2.b2PolygonDef()
#        shapeDef.SetAsBox(0.5,0.5)
#        shapeDef.density = 1
#        shapeDef.friction = 0.08
#
#        self.body.CreateShape(shapeDef)
#        self.body.SetMassFromShapes()
#        self.body.SetLinearVelocity((2, 10))
#        self.body.SetAngularVelocity(3)

#        self.agentBody = AgentBody(self.world)

    def configure(self, cfg_file):
        """ Configuration before the map is loaded
        """
        import json
        fp = file(cfg_file, 'r')
        self.cfg = bunch.bunchify(json.load(fp))
        fp.close()
        # Solver configuration
        self.timestep = self.cfg.solver.timestep
        self.velocityIterations = self.cfg.solver.velocityIterations
        self.positionIterations = self.cfg.solver.positionIterations
        # World configuration ---remove, this should come from the map ---
        self.worldLowerBound = tuple(self.cfg.world.lowerBound)
        self.worldUpperBound = tuple(self.cfg.world.upperBound)



    def load(self):
        # Build the world
        worldAABB = b2.b2AABB()
        worldAABB.lowerBound = self.worldLowerBound
        worldAABB.upperBound = self.worldUpperBound
        self.world = b2.b2World(worldAABB, (0,0), True)
        # Load the map and populate the world
        return True

    def start(self):
        """ Used in non-interactive simulations"""
        self.running = True
        self.run()

    def run(self):
        """ Used in non-interactive simulations"""
        while self.running:
            self.step()

    def step(self):
        """ Update with a given step size """
#        self.world.Step(TIME_STEP, VEL_ITER, POS_ITER)
        self.world.Step(self.timestep,
                        self.velocityIterations,
                        self.positionIterations)
#        self.agentBody.update()
#        print self.body.position, self.body.angle, self.body.linearVelocity
#        pass

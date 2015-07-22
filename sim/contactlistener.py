"""
 Main Listener.
"""
import sys
from conf import msg
try:
    import Box2D as b2
except ImportError:
    sys.stderr.write(msg.import_box2d_fail+msg.newline)
    sys.exit(1)
#-------------------------------------------------------------------------------
# Contact Listener for the agent
#-------------------------------------------------------------------------------
class WorldContactListener(b2.b2ContactListener):
    def __init__(self, agent):
        b2.b2ContactListener.__init__(self)
        self.agent = agent
    def BeginContact(self, contact):
        pass
#        print hex(id(contact.fixtureA)), hex(id(contact.fixtureB))
#        if not contact.touching: return
#        sensor = contact.fixtureA
#        fix = contact.fixtureB
#        if contact.fixtureA == self.agent.sensor:
#            print '-S-'
#        if contact.fixtureB == self.agent.sensor:
#            print '-S-'
        # Needs to be touching
#        print fix.body.worldCenter

    def EndContact(self, contact):
        pass
    def PreSolve(self, contact, oldManifold):
        pass
    def PostSolve(self, contact, impulse):
        pass

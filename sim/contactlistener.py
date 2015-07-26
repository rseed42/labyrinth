"""
 Global contact listener for the world
"""
import sys
import Box2D as b2
from staticobject import StaticObject
from agent import Agent
#-------------------------------------------------------------------------------
# Contact Listener for the agent
#-------------------------------------------------------------------------------
class WorldContactListener(b2.b2ContactListener):
    def __init__(self):
        super(WorldContactListener, self).__init__()

    def BeginContact(self, contact):
        # Extra careful here. UserData contains a pointer to an agent object.
        for fix in (contact.fixtureA, contact.fixtureB):
            agent = fix.body.userData
            if not isinstance(agent, Agent): continue
            otherFix = contact.fixtureB
            if otherFix == fix: otherFix = contact.fixtureA
            agent.handleCollisionBegin(contact, fix, otherFix)

    def EndContact(self, contact):
        # Extra careful here. UserData contains a pointer to an agent object.
        for fix in (contact.fixtureA, contact.fixtureB):
            agent = fix.body.userData
            if not isinstance(agent, Agent): continue
            otherFix = contact.fixtureB
            if otherFix == fix: otherFix = contact.fixtureA
            agent.handleCollisionEnd(contact, fix, otherFix)

    def PreSolve(self, contact, oldManifold):
        pass

    def PostSolve(self, contact, impulse):
        pass

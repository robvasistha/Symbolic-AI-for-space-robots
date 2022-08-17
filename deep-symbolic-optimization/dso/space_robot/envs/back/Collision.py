from panda3d.core import loadPrcFileData
from panda3d.core import LPoint3f
loadPrcFileData("", "window-type none" ) # Make sure we don't need a graphics engine (Will also prevent X errors / Display errors when starting on linux without X server)
loadPrcFileData("", "audio-library-name null" ) # Prevent ALSA errors
import direct.directbase.DirectStart
from direct.showbase.DirectObject import DirectObject
import numpy as np
from panda3d.core import CollisionTraverser
from panda3d.core import CollisionSphere, CollisionNode, CollisionBox, CollisionCapsule
from panda3d.core import CollisionHandlerEvent
from panda3d.core import CollisionHandlerQueue

class Collision(DirectObject):
    def __init__(self, dof):
        # prevent computer using 100% CPU
        # make node at the origin of the system
        self.basenode = render.attachNewNode('base node')
        self.basenode.setPos(0, 0, 0)
        self.dummynode = self.basenode.attachNewNode('dummy')
        self.dof = dof

    def rad_2_deg(self, x):
        return x * (180 / 3.14159265359)

    def node_form(self, state, ff, l, payload):
        self.dummynode = self.basenode.attachNewNode('dummy')

        self.PayloadNode = self.dummynode.attachNewNode('Payload')
        self.PayloadNode.setHpr(self.rad_2_deg(payload[3]), self.rad_2_deg(payload[4]), self.rad_2_deg(payload[5]))
        self.PayloadNode.setPos(payload[0], payload[1], payload[2])

        x, y, z, roll, pitch, yaw = state[0], state[1], state[2], self.rad_2_deg(state[3]), self.rad_2_deg(
            state[4]), self.rad_2_deg(state[5])

        # make node for the CoM of the base spacecraft. This is positioned given the state.
        self.basecom = self.dummynode.attachNewNode('base com')
        self.basecom.setPos(x, y, z)
        self.basecom.setHpr(yaw, roll, pitch)
        self.basecom.setColor(0.12, 0.49, 0.76, 0.5)

        # create node for link 1. This should be attached to the base.
        self.link1node = self.basecom.attachNewNode("Link 1")
        self.link1node.setZ(ff[2] / 2)
        self.link1node.setHpr( self.rad_2_deg(state[6]),0,0)
        self.link1node.setColor(1, 1, 1, 1)

        self.link2node = self.link1node.attachNewNode("Link 2")
        self.link2node.setZ(np.amax(l[0]))
        self.link2node.setHpr(0, self.rad_2_deg(state[7]), 0)

        self.link3node = self.link2node.attachNewNode("Link 3")
        self.link3node.setZ(np.amax(l[1]))
        self.link3node.setHpr(0, self.rad_2_deg(state[8]), 0)

        if self.dof == 4:
            self.eenode = self.link3node.attachNewNode("End Effector")
            self.eenode.setZ(np.amax(l[2]))
            self.eenode.setHpr(self.rad_2_deg(state[9]),0,0)

        if self.dof == 5:
            self.link4node = self.link3node.attachNewNode("Link 4")
            self.link4node.setZ(np.max(l[2]))
            self.link4node.setHpr(0, self.rad_2_deg(state[9]),0)

            self.eenode = self.link4node.attachNewNode("End Effector")
            self.eenode.setZ(np.amax(l[3]))
            self.eenode.setHpr(self.rad_2_deg(state[10]), 0, 0)

        if self.dof == 6:
            self.link4node = self.link3node.attachNewNode("Link 4")
            self.link4node.setZ(np.amax(l[2]))
            self.link4node.setHpr(0, self.rad_2_deg(state[9]), 0)

            self.link5node = self.link4node.attachNewNode("Link 5")
            self.link5node.setZ(np.amax(l[3]))
            self.link5node.setHpr(0, self.rad_2_deg(state[10]), 0)

            self.eenode = self.link5node.attachNewNode("End Effector")
            self.eenode.setZ(np.amax(l[4]))
            self.eenode.setHpr(self.rad_2_deg(state[11]), 0, 0)


    def contact(self, base_dim, links, payload_size):
        self.collhandevent = CollisionHandlerEvent()
        self.collhandqueue = CollisionHandlerQueue()
        self.collhandevent.addInPattern("into-%in")
        base.cTrav = CollisionTraverser()

        colliderNode1 = CollisionNode("payload")
        position = LPoint3f(0,0,0)
        colliderNode1.addSolid(CollisionSphere(position, payload_size[0]/2))
        payload_collision = self.PayloadNode.attachNewNode(colliderNode1)
        #payload_collision.show()
        base.cTrav.addCollider(payload_collision, self.collhandevent)

        colliderNode2 = CollisionNode("gripper")
        position = LPoint3f(0,0,0)
        colliderNode2.addSolid(CollisionSphere(0,0,0.15,0.075))
        end_effector_collision = self.eenode.attachNewNode(colliderNode2)
        #end_effector_collision.show()

        self.accept("into-" + "gripper", self.collide)

        colliderNode3 = CollisionNode("base")
        position = LPoint3f(-base_dim[0]/2,-base_dim[1]/2,-base_dim[2]/2)
        colliderNode3.addSolid(CollisionBox(position, (base_dim[0]/2, base_dim[1]/2, base_dim[2]/2)))
        self.basecom.attachNewNode(colliderNode3)
        #base_collision.show()

        self.accept("into-" + "base", self.collide1)

        colliderNode4 = CollisionNode("link 1")
        colliderNode4.addSolid(CollisionCapsule(0,0,0.05,0,0,links[1].max()-0.05,0.04))
        link_1_collision = self.link1node.attachNewNode(colliderNode4)
        #link_1_collision.show()

        self.accept("into-" + "link 1", self.collide1)

        colliderNode5 = CollisionNode("link 2")
        colliderNode5.addSolid(CollisionCapsule(0,0,0.05,0,0,links[2].max()-0.05,0.04))
        link_2_collision = self.link2node.attachNewNode(colliderNode5)
        #link_2_collision.show()

        self.accept("into-" + "link 2", self.collide1)

        colliderNode6 = CollisionNode("link 3")
        colliderNode6.addSolid(CollisionCapsule(0,0,0.05,0,0,links[3].max()-0.05,0.04))
        link_3_collision = self.link3node.attachNewNode(colliderNode6)
        #link_3_collision.show()

        self.accept("into-" + "link 3", self.collide1)

        if self.dof == 5:
            colliderNode7 = CollisionNode("link 4")
            colliderNode7.addSolid(CollisionCapsule(0, 0, 0.05, 0, 0, links[4].max() - 0.05, 0.04))
            link_4_collision = self.link4node.attachNewNode(colliderNode7)

            self.accept("into-" + "link 4", self.collide1)

            if self.dof == 6:
                colliderNode8 = CollisionNode("link 5")
                colliderNode8.addSolid(CollisionCapsule(0, 0, 0.05, 0, 0, links[5].max() - 0.05, 0.04))
                link_5_collision = self.link5node.attachNewNode(colliderNode8)
                link_5_collision.show()

                self.accept("into-" + "link 5", self.collide1)

        """ FIX - sometimes the traverser was called when no nodes were present this lead to an error. The fix is to 
            set the collision to none if this happens and wait for the next time step to detect them.        
        """

        try:
            base.cTrav.traverse(self.dummynode)
        except:
            self.done = False
            self.complete = False

        """ FIX - this double clear and the second taskMgr.step stopped the zero time episode. This was when the
            system thought it was captured immediately after a previous successful episode.
        """

        base.cTrav.clearColliders()
        taskMgr.step()

        return self.done, self.complete

    # need to work out which object is causing the collision. Adding the link and base node to the collision traverser
    # allows them to be detected but does not yet give info on what has collided.
    def collide(self, event):
        # print('payload captured')
        self.complete = True
        self.done = True

    def collide1(self, event):
        # print('base collision')
        self.complete = False
        self.done = True

    def check_collision(self, state, base_dim, links, payload_size, payload_position):
        self.dummynode.removeNode()
        self.node_form(state, base_dim, links, payload_position)
        done = self.contact(base_dim, links, payload_size)
        return done

    def distance(self, old_dist):
        """ FIX - occassionally the dist went to nan. This takes the distance from the step before."""
        try:
            dist = (self.eenode.getPos(self.dummynode)-self.PayloadNode.getPos(self.dummynode)).length()
        except:
            dist = old_dist
        return dist

    def reset(self):
        self.dummynode.removeNode()
        self.done = False
        self.complete = False

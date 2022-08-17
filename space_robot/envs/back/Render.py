from panda3d.core import loadPrcFileData
loadPrcFileData("", "window-type none" ) # Make sure we don't need a graphics engine (Will also prevent X errors / Display errors when starting on linux without X server)
import direct.directbase.DirectStart
from direct.gui.DirectGui import *
from pandac.PandaModules import *
from direct.showbase.DirectObject import DirectObject
from space_robot.envs.back.Mission import *
from direct.interval.IntervalGlobal import *

class Render(DirectObject):
    def __init__(self, dof):
        base.openMainWindow(type='onscreen')
        self.basenode = render.attachNewNode('base node')
        self.basenode.setPos(0,0,0)
        self.initLighting()
        # make node at the origin of the system
        self.mission = Mission(dof)
        self.dummynode = self.basenode.attachNewNode('dummy')
        imageObject = OnscreenImage(image='space_robot/envs/back/stars.jpg', pos=(0, 10, 0), parent=base.cam, scale=5)
        # Note the inclusion of the "parent" keyword.
        # The scale is likely not correct; I leave it to you to find a proper value for it!
        imageObject.setBin("background", 0)
        imageObject.setDepthWrite(False)
        self.dof = dof

    def rad_2_deg(self, x):
        return x*(180/3.14159265359)

    def form(self, state, ff, l, payload, payload_size, capture):
        self.dummynode = self.basenode.attachNewNode('dummy')
        target = self.makepayload(payload_size)
        self.PayloadNode = self.dummynode.attachNewNode('payload')
        self.PayloadNode.setHpr(rad_2_deg(payload[3]), rad_2_deg(payload[4]), rad_2_deg(payload[5]))
        self.PayloadNode.setPos(payload[0], payload[1], payload[2])
        payload = self.PayloadNode.attachNewNode(target)
        if capture == 1:
            self.PayloadNode.setColor(0, 230, 64, 1)

        x, y, z, roll, pitch, yaw = state[0], state[1], state[2], self.rad_2_deg(state[3]), self.rad_2_deg(
            state[4]), self.rad_2_deg(state[5])

        # make node for the CoM of the base spacecraft. This is positioned given the state.
        self.basecom = self.dummynode.attachNewNode('base com')
        self.basecom.setPos(x, y, z)
        self.basecom.setHpr(yaw, roll, pitch)
        self.basecom.setColor(0.12, 0.49, 0.76, 0.5)

        # make the geometry of the base spacecraft based on the form factor input. Add this to the scene at the
        # location given by the state
        base = loader.loadModel("space_robot/envs/back/satellite_body.glb")
        base.setScale(0.4*ff[0], 0.4*ff[1], 0.4*ff[2])
        base.reparentTo(self.basecom)

        # create node for link 1. This should be attached to the base.
        self.link1node = self.basecom.attachNewNode("Link 1")
        self.link1node.setZ((ff[2]/2)+0.05)
        self.link1node.setHpr(self.rad_2_deg(state[self.dof]), 0, 0)
        self.link1node.setColor(1, 1, 1, 1)

        link1 = self.makelink(l[0])
        LinkOne = self.link1node.attachNewNode(link1)

        self.link2node = self.link1node.attachNewNode("Link 2")
        self.link2node.setZ(np.amax(l[0]))
        self.link2node.setHpr(0, self.rad_2_deg(state[self.dof+1]), 0)

        link2 = self.makelink(l[1])
        LinkTwo = self.link2node.attachNewNode(link2)

        self.link3node = self.link2node.attachNewNode("Link 3")
        self.link3node.setZ(np.amax(l[1]))
        self.link3node.setHpr(0, self.rad_2_deg(state[self.dof+2]), 0)

        link3 = self.makelink(l[2])
        LinkThree = self.link3node.attachNewNode(link3)

        if self.dof == 4:
            self.eenode = self.link3node.attachNewNode("End Effector")
            self.eenode.setZ(np.amax(l[2]))
            self.eenode.setHpr(self.rad_2_deg(state[self.dof+3]), 0, 0)

            ee = loader.loadModel("space_robot/envs/back/end_effector")
            ee.setScale(0.04)
            ee.reparentTo(self.eenode)

        if self.dof == 5:
            self.link4node = self.link3node.attachNewNode("Link 4")
            self.link4node.setZ(np.amax(l[2]))
            self.link4node.setHpr(0, self.rad_2_deg(state[self.dof+3]), 0)

            link4 = self.makelink(l[3])
            LinkFour = self.link4node.attachNewNode(link4)

            self.eenode = self.link4node.attachNewNode("End Effector")
            self.eenode.setZ(np.amax(l[3]))
            self.eenode.setHpr(self.rad_2_deg(state[self.dof+4]), 0, 0)

            ee = loader.loadModel("space_robot/envs/back/end_effector")
            ee.setScale(0.04)
            ee.reparentTo(self.eenode)

        if self.dof == 6:
            self.link4node = self.link3node.attachNewNode("Link 4")
            self.link4node.setZ(np.amax(l[2]))
            self.link4node.setHpr(0, self.rad_2_deg(state[self.dof+3]), 0)

            link4 = self.makelink(l[3])
            LinkFour = self.link4node.attachNewNode(link4)

            self.link5node = self.link4node.attachNewNode("Link 5")
            self.link5node.setZ(np.amax(l[3]))
            self.link5node.setHpr(0, self.rad_2_deg(state[self.dof+4]), 0)

            link5 = self.makelink(l[4])
            LinkFive = self.link5node.attachNewNode(link5)

            self.eenode = self.link5node.attachNewNode("End Effector")
            self.eenode.setZ(np.amax(l[4]))
            self.eenode.setHpr(self.rad_2_deg(state[self.dof+5]), 0, 0)

            ee = loader.loadModel("space_robot/envs/back/end_effector")
            ee.setScale(0.04)
            ee.reparentTo(self.eenode)


    def initLighting(self):
        panda3d.core.load_prc_file_data('', 'framebuffer-srgb true')
        ambientLight=AmbientLight('ambient')
        ambientLight.setColor(Vec4(.3,.3,.3,1))
        ambientLightNP=render.attachNewNode(ambientLight)
        render.setLight(ambientLightNP)
        # render.set_shader_auto()

        # directionalLight=DirectionalLight("directional 1")
        # directionalLight.setColor(Vec4(1,1,1,1))
        # directionalLightNP=render.attachNewNode(directionalLight)
        # directionalLightNP.setHpr(0,0,0)
        # render.setLight(directionalLightNP)

        directionalLight=DirectionalLight("directional 2")
        directionalLight.setColor(Vec4(1,1,1,1))
        directionalLightNP=render.attachNewNode(directionalLight)
        directionalLightNP.setPos(-10,0,0)
        directionalLightNP.setHpr(45,0,0)
        render.setLight(directionalLightNP)

        # plight = PointLight('plight')
        # plight.setColor((0.2, 0.2, 0.2, 1))
        # plnp = render.attachNewNode(plight)
        # plnp.setPos(0, -10, 0)
        # render.setLight(plnp)

        #base.useDrive()
        base.disableMouse()
        base.camera.setPos(0, -7, 2)
        base.camera.setHpr(0, -10, 0)

    def makebase(self, ff):
        fx = ff[0] / 2
        fy = ff[1] / 2
        fz = ff[2] / 2
        vertices = [(-fx, -fy, -fz),
                    (-fx, -fy, fz),
                    (-fx, fy, -fz),
                    (-fx, fy, fz),
                    (fx, -fy, -fz),
                    (fx, -fy, fz),
                    (fx, fy, -fz),
                    (fx, fy, fz)]

        windings = [((0, 0, -1), [0, 2, 6, 4]),
                    ((0, 0, 1), [5, 7, 3, 1]),
                    ((0, -1, 0), [4, 5, 1, 0]),
                    ((0, 1, 0), [2, 3, 7, 6]),
                    ((-1, 0, 0), [0, 1, 3, 2]),
                    ((1, 0, 0), [6, 7, 5, 4])]

        vdata = GeomVertexData('vertdata', GeomVertexFormat.getV3n3(), Geom.UHStatic)

        vertex = GeomVertexWriter(vdata, 'vertex')
        normal = GeomVertexWriter(vdata, 'normal')

        for (normalCoords, windingEntries) in windings:
            for x in windingEntries:
                vertex.addData3f(*(vertices[x]))
                normal.addData3f(*normalCoords)

        prim = GeomTrifans(Geom.UHStatic)

        for i in range(0, len(windings)):
            prim.addConsecutiveVertices(i * 4, 4)
            prim.closePrimitive()

        geom = Geom(vdata)
        geom.addPrimitive(prim)

        node = GeomNode('gnode')
        node.addGeom(geom)

        return node

    def makelink(self, l):
        fx = 0.02
        fy = 0.02
        fz = np.amax(l)
        vertices = [(-fx, -fy, 0),
         (-fx, -fy, fz),
         (-fx, fy, 0),
         (-fx, fy, fz),
         (fx, -fy, 0),
         (fx, -fy, fz),
         (fx, fy, 0),
         (fx, fy, fz)]


        windings = [((0, 0, -1), [0, 2, 6, 4]),
                    ((0, 0, 1), [5, 7, 3, 1]),
                    ((0, -1, 0), [4, 5, 1, 0]),
                    ((0, 1, 0), [2, 3, 7, 6]),
                    ((-1, 0, 0), [0, 1, 3, 2]),
                    ((1, 0, 0), [6, 7, 5, 4])]

        vdata = GeomVertexData('vertdata', GeomVertexFormat.getV3n3(), Geom.UHStatic)

        vertex = GeomVertexWriter(vdata, 'vertex')
        normal = GeomVertexWriter(vdata, 'normal')

        for (normalCoords, windingEntries) in windings:
            for x in windingEntries:
                vertex.addData3f(*(vertices[x]))
                normal.addData3f(*normalCoords)

        prim = GeomTrifans(Geom.UHStatic)

        for i in range(0, len(windings)):
            prim.addConsecutiveVertices(i * 4, 4)
            prim.closePrimitive()

        geom = Geom(vdata)
        geom.addPrimitive(prim)

        node = GeomNode('gnode')
        node.addGeom(geom)

        return node

    def makepayload(self, payload):
        fx, fy, fz = payload[0]/2 ,payload[1]/2 , payload[2]/2

        vertices = [(-fx, -fy, -fz),
                    (-fx, -fy, fz),
                    (-fx, fy, -fz),
                    (-fx, fy, fz),
                    (fx, -fy, -fz),
                    (fx, -fy, fz),
                    (fx, fy, -fz),
                    (fx, fy, fz)]

        windings = [((0, 0, -1), [0, 2, 6, 4]),
                    ((0, 0, 1), [5, 7, 3, 1]),
                    ((0, -1, 0), [4, 5, 1, 0]),
                    ((0, 1, 0), [2, 3, 7, 6]),
                    ((-1, 0, 0), [0, 1, 3, 2]),
                    ((1, 0, 0), [6, 7, 5, 4])]

        vdata = GeomVertexData('vertdata', GeomVertexFormat.getV3n3(), Geom.UHStatic)

        vertex = GeomVertexWriter(vdata, 'vertex')
        normal = GeomVertexWriter(vdata, 'normal')

        for (normalCoords, windingEntries) in windings:
            for x in windingEntries:
                vertex.addData3f(*(vertices[x]))
                normal.addData3f(*normalCoords)

        prim = GeomTrifans(Geom.UHStatic)

        for i in range(0, len(windings)):
            prim.addConsecutiveVertices(i * 4, 4)
            prim.closePrimitive()

        geom = Geom(vdata)
        geom.addPrimitive(prim)

        node = GeomNode('gnode')
        node.addGeom(geom)
        return node

    def render(self, state, ff, l, payload, payload_size, capture):
        #print(state)
        self.dummynode.removeNode()
        self.form(state, ff, l, payload, payload_size, capture)
        taskMgr.step()

    def final_render(self, state, ff, l, payload, payload_size):
        #print(state)
        self.dummynode.removeNode()
        self.form(state, ff, l, payload, payload_size)
        taskMgr.run()

    def reset(self):
        self.dummynode.removeNode()

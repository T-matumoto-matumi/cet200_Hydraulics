import agx
import agxSDK
import agxOSG
import agxRender
import agxCollide

from agxPythonModules.utils.environment import application
from agxPythonModules.utils.numpy_utils import wrap_vector_as_numpy_array

import math
import numpy as np

import collections


class LidarSensor2D:
    """
    A 2D lidar sensor simulated using the depth buffer of the visuals
    """

    def __init__(
        self,
        width: int,
        height: int,
        eye: agx.Vec3,
        center: agx.Vec3,
        up: agx.Vec3,
        fovy: float = 64.0,
        near: float = 0.1,
        far: float = 1000.0,
        eye_coordinates: bool = True,
    ):
        """
        Utility class for using agxOSG::SimpleDepthBufferLidar as a 2D lidar sensor,
        that is a camera that renders to a depth buffer and can transform the buffer
        to values in the world or eye coordinates.

        :param width: The width resolution
        :param height: The height resolution
        :param eye: The position in the world that the camera is placed at
        :param center: The position in the world that the camera aims at
        :param up: The "up" direction of the camera
        :param fovy: The field of view of the camera
        :param near: The near rendering plane of the camera
        :param far: The far rendering plane of the camera
        :eye_coordinates: Decides if the scan should be in eye (camera coordinates) or world coordinates
        """
        self._lidar = agxOSG.SimpleDepthBufferLidar(
            width, height, eyeCoordinates=eye_coordinates
        )
        self._lidar.setViewMatrixAsLookAt(eye, center, up)
        self._lidar.setProjectionMatrixAsPerspective(fovy, width / height, near, far)
        application().addRenderTarget(self._lidar)

    def scan(self):
        scan = self._lidar.getScan()
        pts = wrap_vector_as_numpy_array(scan, np.float32)
        return pts.copy().reshape(-1, 4)


class LidarSensor1D(agxSDK.StepEventListener):
    """
    A 1D lidar simulated using agxCollide::Line and collision detection.
    """

    def __init__(
        self,
        sim: agxSDK.Simulation,
        world_position: agx.Vec3,
        world_direction: agx.Vec3,
        num_side_rays: int,
        rad_range_side: int,
        max_length: float,
        rb_origin: agx.RigidBody = None,
        draw_lines: bool = False,
    ):
        """
        Creates the lidar sensor. The Lidar sensor is placed at the given world_position and
        directed towards the specified world_direction. It always has at least one ray in the world direction.

        :param sim: Simulation that the Lines and Listeners are added to
        :param world_position: World position of the lidar
        :param world_direction: World direction of the lidar
        :param num_side_rays: Number of rays created on either side of the middle ray. Can be 0.
        :param rad_range_side: The range within the side rays are created.
        :param max_length: The maximum length of the lidar rays
        :param rb_origin: Rigidbody to lock the lidar body to. Will lock to world if None
        :param draw_lines: debug rendering of the rays
        """
        super().__init__(agxSDK.StepEventListener.PRE_COLLIDE)

        geom = agxCollide.Geometry(agxCollide.Box(0.1, 0.1, 0.1))
        geom.setName("lidar_geom")
        geom.setSensor(True)
        lidar_body = agx.RigidBody(geom)
        lidar_body.setMotionControl(agx.RigidBody.KINEMATICS)

        lidar_body.setRotation(agx.Quat(agx.Vec3().Z_AXIS(), world_direction))
        lidar_body.setPosition(world_position)

        self._relative_transform = None
        if rb_origin is not None:
            self._relative_transform = (
                lidar_body.getTransform() * rb_origin.getTransform().inverse()
            )

        rays_dict = collections.OrderedDict()

        delta = 0
        start = 0
        if num_side_rays > 0:
            delta = rad_range_side / num_side_rays
            start = -rad_range_side

        for i in range(2 * num_side_rays + 1):
            angle = start + i * delta
            z = max_length * math.cos(angle)
            y = max_length * math.sin(angle)
            ray = agxCollide.Geometry(agxCollide.Line(agx.Vec3(0), agx.Vec3(0, y, z)))
            ray.setSensor(True)
            ray.addGroup("LidarGeom")
            ray.setName("Ray")
            ray.setEnableCollisions(geom, False)
            lidar_body.add(ray)
            rays_dict[ray.getUuid()] = [ray, max_length]

        self.cel = LidarContactSensor(lidar_body, rays_dict, max_length)
        sim.add(self.cel)
        sim.add(self)

        sim.add(lidar_body)
        self._lidar_body = lidar_body
        self._rb_origin = rb_origin
        self._rays_dict = rays_dict
        self._max_length = max_length
        self._draw_lines = draw_lines

        render_manager = sim.getRenderManager()
        if render_manager and self._draw_lines:
            render_manager.disableFlags(agxRender.RENDER_GEOMETRIES)

    def preCollide(self, t):
        if self._draw_lines:
            pos = self._lidar_body.getPosition()
            for k, v in self._rays_dict.items():
                color = v[1] / self._max_length
                ray = v[0].getShapes()[0].asLine()
                f = v[0].getFrame()
                d = f.transformPointToWorld(ray.getSecondPoint()) - pos
                agxRender.RenderSingleton.instance().add(
                    pos,
                    pos + v[1] * d.normal(),
                    0.025,
                    agx.Vec4f(1 - color**0.4, color**0.4, 0, 1),
                )

        # clear rays
        for k in self._rays_dict.keys():
            self._rays_dict[k][1] = self._max_length

        if self._relative_transform:
            self._lidar_body.setTransform(
                self._relative_transform * self._rb_origin.getTransform()
            )

    def get_distances(self):
        d = []
        for _, v in self._rays_dict.items():
            d.append(v[1])
        return d


class LidarSensor3DNoGPU(agxSDK.StepEventListener):
    """
    A 3D lidar simulated with agxCollide::Line sensor geometries.

    This does not use the depth buffer or GPU rendering. Each beam is a collision
    line and the closest contact point along that beam is stored as the measured
    distance.
    """

    def __init__(
        self,
        sim: agxSDK.Simulation,
        world_position: agx.Vec3,
        world_direction: agx.Vec3,
        horizontal_rays: int,
        vertical_rays: int,
        horizontal_fov: float,
        vertical_fov: float,
        max_length: float,
        rb_origin: agx.RigidBody = None,
        rb_origin_frame=None,
        draw_lines: bool = False,
    ):
        """
        Creates a CPU-only 3D lidar.

        :param sim: Simulation that the lidar body and listeners are added to
        :param world_position: World position of the lidar
        :param world_direction: Forward direction of the lidar
        :param horizontal_rays: Number of rays across the horizontal field of view
        :param vertical_rays: Number of rays across the vertical field of view
        :param horizontal_fov: Total horizontal field of view in radians
        :param vertical_fov: Total vertical field of view in radians
        :param max_length: Maximum length of each lidar ray
        :param rb_origin: Rigidbody to follow. The lidar is locked to world if None
        :param rb_origin_frame: Local lidar frame on rb_origin. The lidar rays use
                                +X forward, +Y left and +Z up in this frame.
        :param draw_lines: Enables debug rendering of rays
        """
        super().__init__(agxSDK.StepEventListener.PRE_COLLIDE)

        if horizontal_rays < 1:
            raise ValueError("horizontal_rays must be at least 1")
        if vertical_rays < 1:
            raise ValueError("vertical_rays must be at least 1")

        geom = agxCollide.Geometry(agxCollide.Box(0.1, 0.1, 0.1))
        geom.setName("lidar_3d_geom")
        geom.setSensor(True)
        lidar_body = agx.RigidBody(geom)
        lidar_body.setMotionControl(agx.RigidBody.KINEMATICS)
        if rb_origin is not None and rb_origin_frame is not None:
            lidar_body.setTransform(rb_origin_frame * rb_origin.getTransform())
        else:
            lidar_body.setRotation(agx.Quat(agx.Vec3().X_AXIS(), world_direction))
            lidar_body.setPosition(world_position)

        self._relative_transform = None
        if rb_origin is not None:
            self._relative_transform = (
                lidar_body.getTransform() * rb_origin.getTransform().inverse()
            )

        self._horizontal_rays = horizontal_rays
        self._vertical_rays = vertical_rays
        self._max_length = max_length
        self._draw_lines = draw_lines
        self._lidar_body = lidar_body
        self._rb_origin = rb_origin
        self._rays_dict = collections.OrderedDict()
        self._ray_directions = []

        for v in range(vertical_rays):
            pitch = self._angle_at_index(v, vertical_rays, vertical_fov)
            for h in range(horizontal_rays):
                yaw = self._angle_at_index(h, horizontal_rays, horizontal_fov)
                direction = self._direction_from_angles(yaw, pitch)
                end_point = max_length * direction
                ray = agxCollide.Geometry(agxCollide.Line(agx.Vec3(0), end_point))
                ray.setSensor(True)
                ray.addGroup("LidarGeom")
                ray.setName("Ray3D")
                ray.setEnableCollisions(geom, False)
                lidar_body.add(ray)
                self._rays_dict[ray.getUuid()] = [ray, max_length]
                self._ray_directions.append(direction)

        self.cel = LidarContactSensor(lidar_body, self._rays_dict, max_length)
        sim.add(self.cel)
        sim.add(self)
        sim.add(lidar_body)

        render_manager = sim.getRenderManager()
        if render_manager and self._draw_lines:
            render_manager.disableFlags(agxRender.RENDER_GEOMETRIES)

    @property
    def rigid_body(self):
        return self._lidar_body

    @property
    def max_length(self):
        return self._max_length

    @property
    def horizontal_rays(self):
        return self._horizontal_rays

    @property
    def vertical_rays(self):
        return self._vertical_rays

    @staticmethod
    def _angle_at_index(index, count, fov):
        if count == 1:
            return 0.0
        return -0.5 * fov + fov * index / (count - 1)

    @staticmethod
    def _direction_from_angles(yaw, pitch):
        cos_pitch = math.cos(pitch)
        return agx.Vec3(
            cos_pitch * math.cos(yaw),
            cos_pitch * math.sin(yaw),
            math.sin(pitch),
        ).normal()

    def preCollide(self, t):
        if self._draw_lines:
            pos = self._lidar_body.getPosition()
            for _, v in self._rays_dict.items():
                color = v[1] / self._max_length
                ray = v[0].getShapes()[0].asLine()
                f = v[0].getFrame()
                direction = f.transformPointToWorld(ray.getSecondPoint()) - pos
                agxRender.RenderSingleton.instance().add(
                    pos,
                    pos + v[1] * direction.normal(),
                    0.0125,
                    agx.Vec4f(1 - color**0.4, color**0.4, 0, 1),
                )

        for k in self._rays_dict.keys():
            self._rays_dict[k][1] = self._max_length

        if self._relative_transform is not None:
            self._lidar_body.setTransform(
                self._relative_transform * self._rb_origin.getTransform()
            )

    def get_distances(self):
        distances = [v[1] for _, v in self._rays_dict.items()]
        return np.array(distances, dtype=np.float32).reshape(
            self._vertical_rays, self._horizontal_rays
        )

    def get_point_cloud(self, world_coordinates=True, include_misses=True):
        points = []
        frame = self._lidar_body.getFrame()
        for direction, (_, ray_data) in zip(
            self._ray_directions, self._rays_dict.items()
        ):
            distance = ray_data[1]
            if not include_misses and distance >= self._max_length:
                continue
            point = distance * direction
            if world_coordinates:
                point = frame.transformPointToWorld(point)
            points.append([point.x(), point.y(), point.z()])
        return np.array(points, dtype=np.float32)


LidarSensor3D = LidarSensor3DNoGPU


class LidarContactSensor(agxSDK.ContactEventListener):
    """
    Contact event sensor with a filter to only trigger on contacts where one geometry
    belongs to the lidar_body. The distance between the contact point and the lidar body
    is saved if it is closer than the previous registered distance.
    """

    def __init__(self, lidar_body, rays_dict, max_length):
        super().__init__(
            agxSDK.ContactEventListener.IMPACT + agxSDK.ContactEventListener.CONTACT
        )

        self.setFilter(agxSDK.RigidBodyFilter(lidar_body))
        self.lidar_body = lidar_body
        self.rays_dict = rays_dict
        self.max_length = max_length

    def contact(self, t, gc):
        return self.handle(t, gc)

    def impact(self, t, gc):
        return self.handle(t, gc)

    def handle(self, t, gc):
        g0 = gc.geometry(0)
        g1 = gc.geometry(1)

        point = None
        g = None
        if g0.getUuid() in self.rays_dict:
            point = gc.points()[0].getPoint()
            g = g0
        elif g1.getUuid() in self.rays_dict:
            point = gc.points()[0].getPoint()
            g = g1
        else:
            # Something is in contact with lidar_geom
            return agxSDK.ContactEventListener.REMOVE_CONTACT_IMMEDIATELY
        distance = (self.lidar_body.getPosition() - point).length()

        if distance < self.rays_dict[g.getUuid()][1]:
            self.rays_dict[g.getUuid()][1] = distance

        return agxSDK.ContactEventListener.REMOVE_CONTACT_IMMEDIATELY

#!/usr/bin/env python3
"""
Scene Spawner Node — drops a few props (cubes, pyramids, cylinders) into Gazebo.

Purely cosmetic / interaction targets for the teleop station: with the gripper on
you can knock them around or try to pick them up. Everything is driven by the
`scene_objects:` block in teleop.yaml so it needs no rebuild to retune.

    scene_objects:
      enabled: true      # master TRUE/FALSE switch — false = empty arm-only scene
      static: false      # true = bolted down; false = the arm can push them
      objects: [ ... ]   # list of props (see teleop.yaml for the schema)

The node self-gates on `enabled`: if false it logs and exits without spawning,
so it is safe to include unconditionally from every launch file. When enabled it
waits for Gazebo's /gazebo/spawn_sdf_model service and spawns each prop once.

Geometry:
  box      size [sx, sy, sz]            -> SDF <box>
  cylinder size [radius, length]        -> SDF <cylinder> (axis = world Z)
  pyramid  size [base_x, base_y, height]-> meshes/pyramid.stl scaled (box collision)

Each prop's `pose` z is an OFFSET above its natural resting height, so pose
[x, y, 0] always sits flush on the ground plane regardless of size.
"""

import math
import os

import rospy
import rospkg
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnModel


def _material(color):
    r, g, b = (list(color) + [0.5, 0.5, 0.5])[:3]
    a = color[3] if len(color) > 3 else 1.0
    return (
        "<material>"
        f"<ambient>{r} {g} {b} {a}</ambient>"
        f"<diffuse>{r} {g} {b} {a}</diffuse>"
        "<specular>0.1 0.1 0.1 1</specular>"
        "</material>"
    )


def _inertial(mass, ixx, iyy, izz, z=0.0):
    return (
        "<inertial>"
        f"<pose>0 0 {z} 0 0 0</pose>"
        f"<mass>{mass}</mass>"
        f"<inertia><ixx>{ixx}</ixx><ixy>0</ixy><ixz>0</ixz>"
        f"<iyy>{iyy}</iyy><iyz>0</iyz><izz>{izz}</izz></inertia>"
        "</inertial>"
    )


def _box_inertia(mass, sx, sy, sz):
    return (
        mass / 12.0 * (sy * sy + sz * sz),
        mass / 12.0 * (sx * sx + sz * sz),
        mass / 12.0 * (sx * sx + sy * sy),
    )


class SceneSpawner:
    def __init__(self):
        rospy.init_node("scene_spawner", anonymous=False)

        self.enabled = bool(rospy.get_param("scene_objects/enabled", False))
        self.static = bool(rospy.get_param("scene_objects/static", False))
        self.objects = rospy.get_param("scene_objects/objects", [])

        self.mesh_dir = os.path.join(
            rospkg.RosPack().get_path("z1_teleop"), "meshes"
        )

    # --- SDF builders -----------------------------------------------------
    def _link_body(self, geometry_xml, inertial_xml, color):
        collision = geometry_xml if isinstance(geometry_xml, str) else geometry_xml[0]
        visual = geometry_xml if isinstance(geometry_xml, str) else geometry_xml[1]
        return (
            "<link name='link'>"
            + inertial_xml
            + f"<collision name='collision'><geometry>{collision}</geometry>"
            + "<surface><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction></surface>"
            + "</collision>"
            + f"<visual name='visual'><geometry>{visual}</geometry>{_material(color)}</visual>"
            + "</link>"
        )

    def _build_sdf(self, name, otype, size, color, mass):
        """Return (sdf_xml, rest_z) or (None, 0) for an unknown type."""
        if otype == "box":
            sx, sy, sz = size
            box = f"<box><size>{sx} {sy} {sz}</size></box>"
            ixx, iyy, izz = _box_inertia(mass, sx, sy, sz)
            body = self._link_body(box, _inertial(mass, ixx, iyy, izz), color)
            rest_z = sz / 2.0

        elif otype == "cylinder":
            radius, length = size
            cyl = f"<cylinder><radius>{radius}</radius><length>{length}</length></cylinder>"
            ixx = iyy = mass / 12.0 * (3 * radius * radius + length * length)
            izz = mass / 2.0 * radius * radius
            body = self._link_body(cyl, _inertial(mass, ixx, iyy, izz), color)
            rest_z = length / 2.0

        elif otype == "pyramid":
            sx, sy, sz = size
            mesh_path = os.path.join(self.mesh_dir, "pyramid.stl")
            visual = (
                "<mesh>"
                f"<uri>file://{mesh_path}</uri>"
                f"<scale>{sx} {sy} {sz}</scale>"
                "</mesh>"
            )
            # Mesh base sits at z=0, apex at z=sz; approximate collision with a
            # box of the bounding volume, lifted so its base aligns with the mesh.
            collision = (
                f"<box><size>{sx} {sy} {sz}</size></box>"
            )
            ixx, iyy, izz = _box_inertia(mass, sx, sy, sz)
            # collision/inertia centred at half-height (mesh origin is the base)
            link = (
                "<link name='link'>"
                + _inertial(mass, ixx, iyy, izz, z=sz / 2.0)
                + f"<collision name='collision'><pose>0 0 {sz / 2.0} 0 0 0</pose>"
                + f"<geometry>{collision}</geometry>"
                + "<surface><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction></surface>"
                + "</collision>"
                + f"<visual name='visual'><geometry>{visual}</geometry>{_material(color)}</visual>"
                + "</link>"
            )
            body = link
            rest_z = 0.0

        else:
            return None, 0.0

        static = "true" if self.static else "false"
        sdf = (
            "<sdf version='1.6'>"
            f"<model name='{name}'><static>{static}</static>{body}</model>"
            "</sdf>"
        )
        return sdf, rest_z

    # --- main -------------------------------------------------------------
    def run(self):
        if not self.enabled:
            rospy.loginfo("[scene_spawner] scene_objects/enabled=false — "
                          "spawning no props (arm-only scene).")
            return
        if not self.objects:
            rospy.logwarn("[scene_spawner] enabled but scene_objects/objects is empty.")
            return

        rospy.loginfo("[scene_spawner] waiting for /gazebo/spawn_sdf_model ...")
        try:
            rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=60.0)
        except rospy.ROSException:
            rospy.logerr("[scene_spawner] /gazebo/spawn_sdf_model not available; "
                         "is Gazebo running? Skipping prop spawn.")
            return
        spawn = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

        spawned = 0
        for i, obj in enumerate(self.objects):
            name = str(obj.get("name", f"prop_{i}"))
            otype = str(obj.get("type", "box")).lower()
            size = [float(v) for v in obj.get("size", [0.05, 0.05, 0.05])]
            pose = [float(v) for v in obj.get("pose", [0.4, 0.0, 0.0])]
            color = [float(v) for v in obj.get("color", [0.7, 0.7, 0.7])]
            mass = float(obj.get("mass", 0.15))
            yaw = math.radians(float(obj.get("yaw", 0.0)))

            sdf, rest_z = self._build_sdf(name, otype, size, color, mass)
            if sdf is None:
                rospy.logwarn("[scene_spawner] '%s' has unknown type '%s' "
                              "(box|cylinder|pyramid); skipping.", name, otype)
                continue

            p = Pose()
            p.position.x = pose[0]
            p.position.y = pose[1]
            p.position.z = rest_z + pose[2]
            p.orientation.z = math.sin(yaw / 2.0)
            p.orientation.w = math.cos(yaw / 2.0)

            try:
                resp = spawn(model_name=name, model_xml=sdf,
                             robot_namespace="", initial_pose=p,
                             reference_frame="world")
                if resp.success:
                    spawned += 1
                    rospy.loginfo("[scene_spawner] spawned %s (%s).", name, otype)
                else:
                    rospy.logwarn("[scene_spawner] spawn of %s failed: %s",
                                  name, resp.status_message)
            except rospy.ServiceException as exc:
                rospy.logwarn("[scene_spawner] spawn of %s raised: %s", name, exc)

        rospy.loginfo("[scene_spawner] done — %d/%d props spawned.",
                      spawned, len(self.objects))


if __name__ == "__main__":
    try:
        SceneSpawner().run()
    except rospy.ROSInterruptException:
        pass

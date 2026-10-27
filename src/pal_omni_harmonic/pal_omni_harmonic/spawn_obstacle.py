#!/usr/bin/env python3
import subprocess
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Point

CYLINDER_SDF_TEMPLATE = """<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="obstacle">
    <static>true</static>
    <link name="link">
      <pose>0 0 0.5 0 0 0</pose>
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
            <length>1.0</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
            <length>1.0</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.85 0.15 0.15 1</ambient>
          <diffuse>0.85 0.15 0.15 1</diffuse>
          <specular>0.2 0.2 0.2 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


class ObstacleSpawner(Node):
    def __init__(self, ox: float, oy: float, radius: float):
        super().__init__('spawn_obstacle')
        self.ox = ox
        self.oy = oy
        self.radius = radius

        # 1. Remove any existing 'obstacle' model in Gazebo Harmonic
        remove_cmd = [
            'gz', 'service', '-s', '/world/empty_harmonic/remove',
            '--reqtype', 'gz.msgs.Entity',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '1000',
            '--req', 'name: "obstacle" type: MODEL'
        ]
        subprocess.run(remove_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        # 2. Spawn the new cylindrical obstacle with dynamically scaled radius
        sdf_string = CYLINDER_SDF_TEMPLATE.format(radius=self.radius)
        spawn_cmd = [
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-world', 'empty_harmonic',
            '-string', sdf_string,
            '-name', 'obstacle',
            '-x', str(self.ox),
            '-y', str(self.oy),
            '-z', '0.0',
        ]
        subprocess.run(spawn_cmd, check=False)

        # 3. Publish obstacle (ox, oy, r) on /obstacle with TRANSIENT_LOCAL durability
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(Point, '/obstacle', qos)

        msg = Point()
        msg.x = float(self.ox)
        msg.y = float(self.oy)
        msg.z = float(self.radius)  # z field carries radius r
        self.pub.publish(msg)

        self.get_logger().info(
            f"Spawned obstacle at ({self.ox:.2f}, {self.oy:.2f}) with radius r={self.radius:.2f} m. "
            f"Publishing on /obstacle (Ctrl+C to exit after controller starts, or leave running)."
        )


def parse_args():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    if not args:
        raw = input("Enter obstacle 'ox,oy,radius' (e.g. 1,2,0.3): ").strip()
    else:
        raw = " ".join(args)
    parts = raw.replace(',', ' ').split()
    if len(parts) < 3:
        print("Usage: ros2 run pal_omni_harmonic spawn_obstacle <ox>,<oy>,<radius>")
        sys.exit(1)
    return float(parts[0]), float(parts[1]), float(parts[2])


def main():
    ox, oy, radius = parse_args()
    rclpy.init()
    node = ObstacleSpawner(ox, oy, radius)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
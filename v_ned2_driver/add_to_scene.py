import sys
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
import time

class AddObjectToScene(Node):
    def __init__(self):
        super().__init__('add_object_to_scene')
        
        # Create publisher for planning scene
        self.scene_publisher = self.create_publisher(
            PlanningScene,
            '/planning_scene',
            10
        )
        
        # Wait for publisher to be ready
        time.sleep(1)
        
        self.get_logger().info('Adding box to planning scene...')
        self.add_box_to_scene()
        
    def add_box_to_scene(self):
        # Create a collision object message
        collision_object = CollisionObject()
        collision_object.header.frame_id = 'world'
        collision_object.id = 'my_cylinder'
        
        # Define the cylinder primitive
        cylinder = SolidPrimitive()
        cylinder.type = SolidPrimitive.CYLINDER
        cylinder.dimensions = [0.01, 0.04]  # height (1cm), radius (4cm diameter = 2cm radius, but this is diameter so 0.04)
        
        # Define the pose of the cylinder
        cylinder_pose = Pose()
        cylinder_pose.position.x = 0.5
        cylinder_pose.position.y = 0.0
        cylinder_pose.position.z = 0.1
        cylinder_pose.orientation.w = 1.0
        
        # Add the primitive and pose to the collision object
        collision_object.primitives.append(cylinder)
        collision_object.primitive_poses.append(cylinder_pose)
        collision_object.operation = CollisionObject.ADD
        
        # Create planning scene message
        planning_scene = PlanningScene()
        planning_scene.world.collision_objects.append(collision_object)
        planning_scene.is_diff = True
        
        # Publish the planning scene
        self.scene_publisher.publish(planning_scene)
        self.get_logger().info(f"Added cylinder '{collision_object.id}' to the planning scene.")
        
        # Keep node alive briefly to ensure message is sent
        time.sleep(2)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = AddObjectToScene()
        rclpy.spin_once(node, timeout_sec=3.0)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
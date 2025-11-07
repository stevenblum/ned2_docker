import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPlanningScene
import time

class PickCylinder(Node):
    def __init__(self):
        super().__init__('pick_cylinder')
        
        # Initialize MoveItPy
        self.moveit = MoveItPy(node_name="moveit_py_pick")
        self.ned2_arm = self.moveit.get_planning_component("ned2_arm")
        
        # Create service client to get planning scene
        self.scene_client = self.create_client(GetPlanningScene, '/get_planning_scene')
        
        self.get_logger().info('Picking up cylinder...')
        self.pick_cylinder()
        
    def get_object_pose(self, object_id):
        """Get the pose of an object from the planning scene"""
        request = GetPlanningScene.Request()
        request.components.components = request.components.WORLD_OBJECT_GEOMETRY
        
        # Wait for service
        self.scene_client.wait_for_service(timeout_sec=5.0)
        future = self.scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            scene = future.result().scene
            for obj in scene.world.collision_objects:
                if obj.id == object_id:
                    self.get_logger().info(f'Found object: {object_id}')
                    return obj.primitive_poses[0]
        
        self.get_logger().error(f'Object {object_id} not found in planning scene!')
        return None
        
    def pick_cylinder(self):
        # Get cylinder pose from planning scene
        cylinder_pose = self.get_object_pose('my_cylinder')
        
        if cylinder_pose is None:
            return
        
        # Define the grasp pose (above the cylinder)
        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = "world"
        grasp_pose.pose.position.x = cylinder_pose.position.x
        grasp_pose.pose.position.y = cylinder_pose.position.y
        grasp_pose.pose.position.z = cylinder_pose.position.z + 0.05  # 5cm above object
        grasp_pose.pose.orientation.w = 1.0
        
        self.get_logger().info(f'Grasp pose: x={grasp_pose.pose.position.x}, y={grasp_pose.pose.position.y}, z={grasp_pose.pose.position.z}')
        
        # Set the target pose
        self.ned2_arm.set_goal_state(pose_stamped_msg=grasp_pose, pose_link="end_effector_link")
        
        # Plan to the grasp pose
        self.get_logger().info('Planning to grasp pose...')
        plan_result = self.ned2_arm.plan()
        
        if plan_result:
            self.get_logger().info('Executing plan...')
            self.ned2_arm.execute()
            time.sleep(2)
            
            # TODO: Close gripper here
            self.get_logger().info('Close gripper now!')
            time.sleep(1)
            
            # Move up
            grasp_pose.pose.position.z = cylinder_pose.position.z + 0.2
            self.ned2_arm.set_goal_state(pose_stamped_msg=grasp_pose, pose_link="end_effector_link")
            plan_result = self.ned2_arm.plan()
            if plan_result:
                self.ned2_arm.execute()
                self.get_logger().info('Picked up cylinder!')
        else:
            self.get_logger().error('Planning failed!')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = PickCylinder()
        rclpy.spin_once(node, timeout_sec=10.0)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from moveit_wrapper.moveitapi import MoveItApi
from geometry_msgs.msg import Point, Quaternion, Pose
from std_srvs.srv import Empty
from rclpy.action import ActionClient
from franka_msgs.msg import GraspEpsilon
from franka_msgs.action import Grasp


class Tossing(Node):
    def __init__(self):
        super().__init__(node_name="tossing")
        self.path = None
        self.cb = ReentrantCallbackGroup()
        self.get_logger().warn("Started node")

        # Creating tf listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_parent_frame = "panda_link0"

        # Moveit Wrapper Object
        self.moveit = MoveItApi(
            self,
            base_frame="panda_link0",
            end_effector_frame="panda_hand_tcp",
            group_name="panda_manipulator",
            joint_state_topic="joint_states",
            robot_model_name="panda",
        )

        # Creating throw service
        self.throw_service = self.create_service(Empty, "throw", self.throwing_callback)

        # Creating grasp client
        self.grasp_client = ActionClient(self, Grasp, "panda_gripper/grasp", callback_group=ReentrantCallbackGroup())

    async def throwing_callback(self, request, responce):
        """Callback function for the throwing service

        Loads waypoints from simulation to plan and execute the motion to throw

         Args:
           request (Empty): triggers the throwing action
           response (Empty): None

        Returns:
           Empty
        """
        posesList = []
        # x_pose = [3.0701956e-01, 3.8798738e-01]
        # y_pose = [0.0, -2.0939697e-02]
        # z_pose = [4.8526955e-01, 4.9219847e-01]
        # qx = [1.0, 9.9955380e-01]
        # qy = [0.0, 7.2671432e-04]
        # qz = [0.0, -2.9859692e-02]
        # qw = [0.0, -2.7706963e-04]
        # x_pose = [0.3972437083721161]
        # y_pose = [-0.004268326330929995]
        # z_pose = [0.4899594187736511]
        # qx = [0.9999591112136841]
        # qy = [-3.8243761082412675e-05]
        # qz = [-0.009043239988386631]
        # qw = [-6.348253918986302e-06]
        x_pose = [0.35856181383132935,0.39047911763191223]
        y_pose = [-0.014218512922525406,-0.028397340327501297]
        z_pose = [0.4894610643386841,0.4926537275314331]
        qx = [0.9983216524124146,0.9997369647026062]
        qy = [0.00011710412218235433,0.000902213912922889]
        qz = [-0.05790982022881508,-0.02291547693312168]
        qw = [-0.0005903120036236942,-0.0003065155469812453]
        for i in range(len(x_pose)):
            posesList.append(Pose(position=Point(x=x_pose[i],
                                                 y=y_pose[i],
                                                 z=z_pose[i]),
                                  orientation=Quaternion(x=qx[i],
                                                         y=qy[i],
                                                         z=qz[i],
                                                         w=qw[i])))
            self.get_logger().warn(f"{posesList[i]}")
        planned_traj = await self.moveit.create_cartesian_path(waypoints=posesList,
                                                               max_velocity_scaling_factor=1.0,
                                                               max_acceleration_scaling_factor=1.0
                                                               )
        self.moveit.execute_trajectory(planned_traj.trajectory)
        epsilon=GraspEpsilon(inner=0.01,outer=0.01)
        gripper_command = Grasp.Goal(width=0.06,force=50.0,speed=0.1,epsilon=epsilon)
        goal_handle = await self.grasp_client.send_goal_async(gripper_command)
        return responce


def main(args=None):
    rclpy.init(args=args)
    node = Tossing()
    rclpy.spin(node)
    rclpy.shutdown()

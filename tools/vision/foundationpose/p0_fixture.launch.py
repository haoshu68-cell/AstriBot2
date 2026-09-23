"""Launch only the C++ FoundationPose backend for isolated offline validation."""
import os

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    node = ComposableNode(
        name='foundationpose', namespace='p0',
        package='isaac_ros_foundationpose',
        plugin='nvidia::isaac_ros::foundationpose::FoundationPoseNode',
        parameters=[{
            'mesh_file_path': os.environ['P0_MESH'],
            'refine_iterations': 1,
            # Historical replay is explicitly offline. No runtime freshness
            # policy is being relaxed in the host project.
            'discard_old_messages': False,
            'refine_model_file_path': '/models/refine_model.onnx',
            'refine_engine_file_path': '/engines/refine.plan',
            'refine_input_tensor_names': ['input_tensor1', 'input_tensor2'],
            'refine_input_binding_names': ['input1', 'input2'],
            'refine_output_tensor_names': ['output_tensor1', 'output_tensor2'],
            'refine_output_binding_names': ['output1', 'output2'],
            'score_model_file_path': '/models/score_model.onnx',
            'score_engine_file_path': '/engines/score.plan',
            'score_input_tensor_names': ['input_tensor1', 'input_tensor2'],
            'score_input_binding_names': ['input1', 'input2'],
            'score_output_tensor_names': ['output_tensor'],
            'score_output_binding_names': ['output1'],
        }])
    return LaunchDescription([ComposableNodeContainer(
        name='p0_foundationpose_container', namespace='',
        package='rclcpp_components', executable='component_container_mt',
        composable_node_descriptions=[node], output='screen')])

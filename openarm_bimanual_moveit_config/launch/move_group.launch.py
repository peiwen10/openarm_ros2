# Copyright 2025 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def resolve_arm_config(arm_type_str):
    if any(x in arm_type_str for x in ("1.0", "10", "1_0")):
        return "openarm_v1.0"
    if any(x in arm_type_str for x in ("2.0", "20", "2_0")):
        return "openarm_v2.0"
    raise ValueError(f"Invalid arm_type: {arm_type_str}")


def move_group_spawner(context: LaunchContext, arm_type):
    arm_type_str = context.perform_substitution(arm_type)
    config_dir = resolve_arm_config(arm_type_str)
    moveit_config = (
        MoveItConfigsBuilder(
            "openarm", package_name="openarm_bimanual_moveit_config")
        .robot_description_semantic(file_path=f"config/{config_dir}/openarm_bimanual.srdf")
        .joint_limits(file_path=f"config/{config_dir}/joint_limits.yaml")
        .robot_description_kinematics(file_path=f"config/{config_dir}/kinematics.yaml")
        .trajectory_execution(file_path=f"config/{config_dir}/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )
    # Load the MoveIt Task Constructor execution capability so planned task
    # solutions can be executed (required by openarm_mtc).
    moveit_config.move_group_capabilities["capabilities"] = \
        "move_group/ExecuteTaskSolutionCapability"
    return generate_move_group_launch(moveit_config).entities


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("arm_type", default_value="v20"),
        OpaqueFunction(function=move_group_spawner, args=[
                       LaunchConfiguration("arm_type")])
    ])

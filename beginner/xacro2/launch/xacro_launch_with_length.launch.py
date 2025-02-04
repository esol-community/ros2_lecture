#
#  Copyright (C) 2023 eSOL Co.,Ltd.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are
#  met:
#
#  1. Redistributions of source code must retain the above copyright notice,
#  this list of conditions and the following disclaimer.
#
#  2. Redistributions in binary form must reproduce the above copyright notice,
#  this list of conditions and the following disclaimer in the documentation
#  and/or other materials provided with the distribution.
#
#  THIS SOFTWARE IS PROVIDED BY THE FREEBSD PROJECT ``AS IS'' AND ANY EXPRESS
#  OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
#  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
#  NO EVENT SHALL THE FREEBSD PROJECT OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
#  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
#  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
#  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
#  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    ld = LaunchDescription()

    default_model_path = str(get_package_share_path('xacro2')) + '/xacro/launch_arg.xacro'
    rviz_config_path = str(get_package_share_path('urdf_launch')) + '/config/urdf.rviz'

    # launchファイルのパラメータ設定
    model_arg = DeclareLaunchArgument(name='model', default_value=default_model_path,
                                    description='Absolute path to robot urdf file')
    length_arg = DeclareLaunchArgument(name='length_value', default_value="1.0",
                                    description='length between center and robot')
    
    # robot_state_publisherのパラメータ設定
    # launchファイルのパラメータ"length_value"の値を、xacroのパラメータ"length"に設定
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model'), ' length:=', LaunchConfiguration('length_value')]),
        value_type=str
    )

    # robot_state_publisherとrviz2をLaunchDescriptionに追加
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    ld.add_action(model_arg)
    ld.add_action(length_arg)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz_node)

    return ld

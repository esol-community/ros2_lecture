#
#  Copyright (C) 2024 eSOL Co.,Ltd. All rights reserved.
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
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessStart
from launch.substitutions import FindExecutable, LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()

    # Launchファイルで使える設定項目を、launchシステム内の型で保持する
    param_talker_parameter = LaunchConfiguration('param_talker_parameter')

    # 変更するパラメータ名を引数で受け取れるようにする
    param_talker_parameter_arg = DeclareLaunchArgument(
        'param_talker_parameter',
        default_value='"Default parameter from launch file"'
    )

    # ROS 2パラメータを利用するノードを起動する
    # launchファイルから、起動時のパラメータを与えておく
    param_talker_node = Node(
        package="ros2_launch_2",
        executable="param_talker",
        name='param_talker',
        parameters=[
                {"param_talker_parameter": "earth"}
        ]
    )

    # パラメータを変更するアクションを作成する
    change_parameter_action = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set',
            ' /param_talker',
            ' param_talker_parameter ',
            param_talker_parameter
        ]],
        shell=True
    )

    # ノードが起動するイベントを受けてパラメータを変更するアクションを定義する
    parameter_change_action = RegisterEventHandler(
        OnProcessStart(
            target_action=param_talker_node,
            on_start=[
                TimerAction(
                    period=2.0,
                    actions=[
                        change_parameter_action,
                        LogInfo(msg='Change the parameter of the node')
                    ],
                )
            ]
        )
    )

    # アクションを追加する
    ld.add_action(param_talker_parameter_arg)
    ld.add_action(param_talker_node)
    ld.add_action(parameter_change_action)

    return ld

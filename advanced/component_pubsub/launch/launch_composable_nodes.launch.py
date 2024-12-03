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
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # component containerのアクションを作成
    action_component_container = ComposableNodeContainer(
        name="component_pubsub_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        # componentで実装されたノードを指定
        composable_node_descriptions=[
            ComposableNode(
                name="simple_talker_component",
                package="component_pubsub",
                plugin="component_pubsub::SimpleTalker",
                remappings=[("/chatter", "/chatter_remapped")],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),  # プロセス内通信を有効にする
            ComposableNode(
                name="simple_listener_component",
                package="component_pubsub",
                plugin="component_pubsub::SimpleListener",
                remappings=[("/chatter", "/chatter_remapped")],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),  # プロセス内通信を有効にする
        ],
        output="screen",
    )

    # 2秒後に、別の名前でtalkerとlistenerを起動
    action_delayed_nodes = TimerAction(
        period=2.0,
        actions=[
            # Componentを読み込むActionの定義
            LoadComposableNodes(
                # component container の名前を指定して読み込む
                target_container="component_pubsub_container",
                composable_node_descriptions=[
                    # 起動させるComponentの定義
                    ComposableNode(
                        package="component_pubsub",
                        plugin="component_pubsub::SimpleTalker",
                        name="simple_talker_component",
                        namespace="delayed_nodes",
                        remappings=[("/chatter", "/chatter_remapped")],
                        extra_arguments=[{"use_intra_process_comms": True}],
                    ),
                    ComposableNode(
                        name="simple_listener_component",
                        package="component_pubsub",
                        plugin="component_pubsub::SimpleListener",
                        namespace="delayed_nodes",
                        remappings=[("/chatter", "/chatter_remapped")],
                        extra_arguments=[{"use_intra_process_comms": True}],
                    ),
                ],
            ),
        ],
    )

    ld = LaunchDescription()
    ld.add_action(action_component_container)
    ld.add_action(action_delayed_nodes)

    return ld

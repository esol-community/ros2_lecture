#
#  Copyright (C) 2023 eSOL Co.,Ltd. All rights reserved.
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
from launch_ros.actions import Node

def generate_launch_description():
    # launchの構成を示すLaunchDescription型の変数の定義
    ld = LaunchDescription()

    # subscriber nodeを、"listener_renamed1"という名前で定義
    sub_node1 = Node(
        package='pub_sub_comm',
        executable='listener',
        name='listener_renamed1',
        namespace='namespace_app1',                 # namespace_app1というnamespaceを追加
        remappings=[('chatter', 'chatter_app1')]    # chatterトピックをchatter_app1トピックにremap
    )

    # subscriber nodeを、"listener_renamed2"という名前で定義
    sub_node2 = Node(
        package='pub_sub_comm',
        executable='listener',
        name='listener_renamed2',
        namespace='namespace_app2',                 # namespace_app2というnamespaceを追加
        remappings=[('chatter', 'chatter_app2')]    # chatterトピックをchatter_app2トピックにremap
    )

    # LaunchDescriptionに、起動したいノードを追加する
    ld.add_action(sub_node1)
    ld.add_action(sub_node2)

    # launch構成を返すようにする
    return ld

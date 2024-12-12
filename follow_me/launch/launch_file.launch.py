from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Node 1: Depuis le package `follow_me
    node1 = Node(
        package='follow_me',
        executable='follow',  # Nom de l'exécutable du nœud
        name='test_follow',  # Nom unique pour ce nœud
        output='screen'         # Permet d'afficher la sortie sur le terminal
    )

    # Node 2: Depuis le package `autonomous_nav`
    node2 = Node(
        package='autonomous_nav2',
        executable='autonomous_nav2',    # Nom de l'exécutable du nœud
        name='autonomous_nav2',
        output='screen'
    )

    # Retourne les deux nœuds dans la description
    return LaunchDescription([
        node1,
        node2,
    ])

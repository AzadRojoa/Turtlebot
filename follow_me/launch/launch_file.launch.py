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
    
    # Node 3: Depuis le package `go_dock`
    node2 = Node(
        package='go_dock',
        executable='go_dock',    # Nom de l'exécutable du nœud
        name='go_dock',
        output='screen'
    )

    # Retourne les deux nœuds dans la description
    return LaunchDescription([
        node1,
        node2,
    ])

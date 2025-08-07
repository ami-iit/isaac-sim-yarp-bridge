import omni.graph.core as og
import dataclasses


@dataclasses.dataclass
class Camera:
    prefix: str
    suffix: str
    type: str
    target: str


class Settings:
    def __init__(self):
        self.graph_path = "/action_graph"
        self.robot_path = "/World/ergoCubSN002/ergoCubSN002"
        self.articulation_root = self.robot_path + "/root_link"
        self.topic_prefix = "/ergocub"
        self.imus = {
            "waist_imu_0": self.robot_path + "/waist_imu_0/waist_imu_0",
            "head_imu_0": self.robot_path + "/head_imu_0/head_imu_0",
        }
        realsense_prefix = "/World/ergoCubSN002/ergoCubSN002/realsense/"
        self.cameras = [
            Camera(
                prefix="realsense",
                suffix="rgb",
                type="rgb",
                target=realsense_prefix
                + "rsd455/RSD455/Camera_OmniVision_OV9782_Color",
            ),
            Camera(
                prefix="realsense",
                suffix="depth",
                type="depth",
                target=realsense_prefix + "rsd455/RSD455/Camera_Pseudo_Depth",
            ),
        ]


def merge_actions(action_list):
    output = {}
    for d in action_list:
        for key, value in d.items():
            if key in output:
                for val in value:
                    output[key].append(val)
            else:
                output[key] = value
    return output


def add_basic_nodes(graph_keys):
    return {
        graph_keys.CREATE_NODES: [
            ("tick", "omni.graph.action.OnPlaybackTick"),
            ("sim_time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("ros2_context", "isaacsim.ros2.bridge.ROS2Context"),
        ],
    }


def add_ros2_clock_publisher(graph_keys):
    return {
        graph_keys.CREATE_NODES: [
            ("ros2_clock_publisher", "isaacsim.ros2.bridge.ROS2PublishClock"),
        ],
        graph_keys.CONNECT: [
            ("tick.outputs:tick", "ros2_clock_publisher.inputs:execIn"),
            ("ros2_context.outputs:context", "ros2_clock_publisher.inputs:context"),
            (
                "sim_time.outputs:simulationTime",
                "ros2_clock_publisher.inputs:timeStamp",
            ),
        ],
    }


# When creating nodes, first you specify the name of the node, and then the type of
# node. If instead of the type, you pass a dict with a series of other actions, you
# are creating a compound node instead (basically a subgraph). The inputs and
# outputs of a compound node are defined by "promoting" inputs and outputs of the
# internal nodes. If multiple internal nodes need to be connected to the same
# input/outputs, this connection is done explicitly after creating the compound,
# using the compound name.


def add_ros2_joint_compound(graph_keys, settings):
    return {
        graph_keys.CREATE_NODES: [
            (
                "ros2_joint_compound",
                {
                    graph_keys.CREATE_NODES: [
                        (
                            "ros2_joint_publisher",
                            "isaacsim.ros2.bridge.ROS2PublishJointState",
                        ),
                        (
                            "ros2_joint_subscriber",
                            "isaacsim.ros2.bridge.ROS2SubscribeJointState",
                        ),
                        (
                            "articulation_controller",
                            "isaacsim.core.nodes.IsaacArticulationController",
                        ),
                    ],
                    graph_keys.SET_VALUES: [
                        (
                            "ros2_joint_publisher.inputs:targetPrim",
                            settings.articulation_root,
                        ),
                        (
                            "ros2_joint_publisher.inputs:topicName",
                            settings.topic_prefix + "/joint_state",
                        ),
                        (
                            "ros2_joint_subscriber.inputs:topicName",
                            settings.topic_prefix + "/joint_state/input",
                        ),
                        (
                            "articulation_controller.inputs:targetPrim",
                            settings.articulation_root,
                        ),
                    ],
                    graph_keys.PROMOTE_ATTRIBUTES: [
                        ("ros2_joint_publisher.inputs:execIn", "inputs:execIn"),
                        ("ros2_joint_publisher.inputs:context", "inputs:context"),
                        ("ros2_joint_publisher.inputs:timeStamp", "inputs:timeStamp"),
                    ],
                    graph_keys.CONNECT: [
                        (
                            "ros2_joint_subscriber.outputs:effortCommand",
                            "articulation_controller.inputs:effortCommand",
                        ),
                        (
                            "ros2_joint_subscriber.outputs:jointNames",
                            "articulation_controller.inputs:jointNames",
                        ),
                        (
                            "ros2_joint_subscriber.outputs:positionCommand",
                            "articulation_controller.inputs:positionCommand",
                        ),
                        (
                            "ros2_joint_subscriber.outputs:velocityCommand",
                            "articulation_controller.inputs:velocityCommand",
                        ),
                    ],
                },
            )
        ],
        graph_keys.CONNECT: [
            ("tick.outputs:tick", "ros2_joint_compound.inputs:execIn"),
            (
                "ros2_context.outputs:context",
                "ros2_joint_compound.inputs:context",
            ),
            (
                "sim_time.outputs:simulationTime",
                "ros2_joint_compound.inputs:timeStamp",
            ),
            (
                "ros2_joint_compound.inputs:execIn",
                "ros2_joint_subscriber.inputs:execIn",
            ),
            (
                "ros2_joint_compound.inputs:context",
                "ros2_joint_subscriber.inputs:context",
            ),
            (
                "ros2_joint_compound.inputs:execIn",
                "articulation_controller.inputs:execIn",
            ),
        ],
    }


def create_imu_subcompound(graph_keys, settings, imu_name, imu_target):
    read_node_name = "read_" + imu_name
    publish_node_name = "publish_" + imu_name
    return {
        graph_keys.CREATE_NODES: [
            (read_node_name, "isaacsim.sensors.physics.IsaacReadIMU"),
            (publish_node_name, "isaacsim.ros2.bridge.ROS2PublishImu"),
        ],
        graph_keys.SET_VALUES: [
            (
                read_node_name + ".inputs:imuPrim",
                imu_target,
            ),
            (
                publish_node_name + ".inputs:topicName",
                settings.topic_prefix + "/IMU/" + imu_name,
            ),
        ],
        graph_keys.PROMOTE_ATTRIBUTES: [
            (read_node_name + ".inputs:execIn", "inputs:execIn"),
            (publish_node_name + ".inputs:context", "inputs:context"),
        ],
        graph_keys.CONNECT: [
            (
                read_node_name + ".outputs:execOut",
                publish_node_name + ".inputs:execIn",
            ),
            (
                read_node_name + ".outputs:angVel",
                publish_node_name + ".inputs:angularVelocity",
            ),
            (
                read_node_name + ".outputs:linAcc",
                publish_node_name + ".inputs:linearAcceleration",
            ),
            (
                read_node_name + ".outputs:orientation",
                publish_node_name + ".inputs:orientation",
            ),
            (
                read_node_name + ".outputs:sensorTime",
                publish_node_name + ".inputs:timeStamp",
            ),
        ],
    }


def create_imu_compounds(graph_keys, settings):
    if len(settings.imus) == 0:
        return

    imu_compounds = []
    first_compound = None
    connections = []
    for key, value in settings.imus.items():
        compound_name = key + "_compound"
        imu_compounds.append(
            (compound_name, create_imu_subcompound(graph_keys, settings, key, value))
        )
        if not first_compound:
            # Only the first compound gets the attributes promoted
            first_compound = compound_name
        else:
            # Add the connections to the inner compound inputs for the internal inputs
            # that have not been promoted
            connections.append(
                ("ros2_imus_compound.inputs:execIn", compound_name + ".inputs:execIn")
            )
            connections.append(
                ("ros2_imus_compound.inputs:context", compound_name + ".inputs:context")
            )

    connections.append(("tick.outputs:tick", "ros2_imus_compound.inputs:execIn"))
    connections.append(
        ("ros2_context.outputs:context", "ros2_imus_compound.inputs:context")
    )

    return {
        graph_keys.CREATE_NODES: [
            (
                "ros2_imus_compound",
                {
                    graph_keys.CREATE_NODES: imu_compounds,
                    graph_keys.PROMOTE_ATTRIBUTES: [
                        (first_compound + ".inputs:execIn", "inputs:execIn"),
                        (first_compound + ".inputs:context", "inputs:context"),
                    ],
                },
            )
        ],
        graph_keys.CONNECT: connections,
    }


# For the IMU we have a function that creates the nodes for a single IMU in
# a compound, and another function that creates all the IMU subcompounds
# and a compound to include them all.
# Fot the cameras instead, we have some additional connections to do after
# the creation of the subcompound (since we have two nodes that needs both
# execIn and context). Hence, the first function creates the nodes AND the
# subcompound, while the second creates the full compound merging all actions.


def create_camera_subcompound(
    graph_keys,
    topic_prefix,
    camera_prefix,
    camera_suffix,
    camera_target,
    camera_type,
    promoted,
):
    camera_prefix_name = camera_prefix.replace("/", "_")
    render_node_name = "render_" + camera_prefix_name + "_" + camera_suffix
    publish_node_name = "publish_" + camera_prefix_name + "_" + camera_suffix
    publish_info_node_name = "publish_info_" + camera_prefix_name + "_" + camera_suffix
    compound_name = camera_prefix_name + "_" + camera_suffix + "_compound"
    topic_name = topic_prefix + "/" + camera_prefix + "/" + camera_suffix
    topic_info_name = topic_name + "/info"

    compound_actions = {
        graph_keys.CREATE_NODES: [
            (
                render_node_name,
                "isaacsim.core.nodes.IsaacCreateRenderProduct",
            ),
            (publish_node_name, "isaacsim.ros2.bridge.ROS2CameraHelper"),
            (
                publish_info_node_name,
                "isaacsim.ros2.bridge.ROS2CameraInfoHelper",
            ),
        ],
        graph_keys.SET_VALUES: [
            (
                render_node_name + ".inputs:cameraPrim",
                camera_target,
            ),
            (
                publish_node_name + ".inputs:topicName",
                topic_name,
            ),
            (publish_node_name + ".inputs:type", camera_type),
            (
                publish_info_node_name + ".inputs:topicName",
                topic_info_name,
            ),
        ],
        graph_keys.PROMOTE_ATTRIBUTES: [
            (render_node_name + ".inputs:execIn", "inputs:execIn"),
            (publish_node_name + ".inputs:context", "inputs:context"),
        ],
        graph_keys.CONNECT: [
            (
                render_node_name + ".outputs:execOut",
                publish_node_name + ".inputs:execIn",
            ),
            (
                render_node_name + ".outputs:execOut",
                publish_info_node_name + ".inputs:execIn",
            ),
            (
                render_node_name + ".outputs:renderProductPath",
                publish_node_name + ".inputs:renderProductPath",
            ),
            (
                render_node_name + ".outputs:renderProductPath",
                publish_info_node_name + ".inputs:renderProductPath",
            ),
        ],
    }

    output = {
        graph_keys.CREATE_NODES: [(compound_name, compound_actions)],
        # Since we can promote only one "context" input, we do the connection
        # here explicitly
        graph_keys.CONNECT: [
            (
                compound_name + ".inputs:context",
                publish_info_node_name + ".inputs:context",
            ),
        ],
    }

    if promoted:
        # If the sucompound is promoted, promote the inputs
        output[graph_keys.PROMOTE_ATTRIBUTES] = [
            (compound_name + ".inputs:execIn", "inputs:execIn"),
            (compound_name + ".inputs:context", "inputs:context"),
        ]

    return output, compound_name


def create_camera_compounds(graph_keys, settings):
    if len(settings.cameras) == 0:
        return

    compound_name = "ros2_cameras_compound"
    subcompound_actions = []
    promote = True
    connections = []
    for camera in settings.cameras:
        actions, subcompound_name = create_camera_subcompound(
            graph_keys,
            settings.topic_prefix,
            camera.prefix,
            camera.suffix,
            camera.target,
            camera.type,
            promote,
        )
        subcompound_actions.append(actions)

        if not promote:
            connections.append(
                (compound_name + ".inputs:execIn", subcompound_name + ".inputs:execIn")
            )
            connections.append(
                (
                    compound_name + ".inputs:context",
                    subcompound_name + ".inputs:context",
                )
            )

        promote = False  # We promote only the first

    connections.append(("tick.outputs:tick", compound_name + ".inputs:execIn"))
    connections.append(
        ("ros2_context.outputs:context", compound_name + ".inputs:context")
    )

    return {
        graph_keys.CREATE_NODES: [(compound_name, merge_actions(subcompound_actions))],
        graph_keys.CONNECT: connections,
    }


keys = og.Controller.Keys
s = Settings()
cmds = {}
cmds_dicts = [
    add_basic_nodes(keys),
    add_ros2_clock_publisher(keys),
    add_ros2_joint_compound(keys, s),
    create_imu_compounds(keys, s),
    create_camera_compounds(keys, s),
]
cmds = merge_actions(cmds_dicts)

print(cmds)

(graph, nodes, prims, name_to_object_map) = og.Controller.edit(
    {"graph_path": s.graph_path, "evaluator_name": "execution"},
    cmds,
)

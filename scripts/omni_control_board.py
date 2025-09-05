# This script is executed the first time the script node computes, or the next time
# it computes after this script is modified or the 'Reset' button is pressed.
#
# The following callback functions may be defined in this script:
#     setup(db): Called immediately after this script is executed
#     compute(db): Called every time the node computes (should always be defined)
#     cleanup(db): Called when the node is deleted or the reset button is pressed
# Available variables:
#    db: og.Database The node interface - attributes are exposed in a namespace like db.inputs.foo and db.outputs.bar.
#                    Use db.log_error, db.log_warning to report problems in the compute function.
#    og: The omni.graph.core module

# Expects two inputs:
# - domain_id [int]: The ROS2 domain ID (optional)
# - useDomainIDEnvVar [bool]: Define whether to get the domain ID from an env var or not (optional)

import dataclasses
import os

import omni.graph.core as og
import rcl_interfaces
import rclpy
from rcl_interfaces.srv import GetParameters as GetParametersSrv
from rcl_interfaces.srv import SetParameters as SetParametersSrv
from rclpy.context import Context as ROS2Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node as ROS2Node


@dataclasses.dataclass
class ControlBoardSettings:
    node_name: str
    node_set_parameters_service_name: str
    node_get_parameters_service_name: str
    node_timeout: float


# TODO: change this
settings = ControlBoardSettings(
    node_name="isaac_sim_control_board_state",
    node_set_parameters_service_name="ergocub/controlboard/set_parameters",
    node_get_parameters_service_name="ergocub/controlboard/get_parameters",
    node_timeout=0.1,
)


class ControlBoardState:
    control_modes: list[int]
    position_pid_gains: list[float]

    def __init__(self):
        # TODO specify a sort of initialization
        self.control_modes = [1] * 23
        self.position_pid_gains = [1.0, 2.0, 3.0]


class ControlBoardNode(ROS2Node):
    def __init__(
        self,
        node_name: str,
        set_service_name: str,
        get_service_name: str,
        context,
        state,
    ):
        super().__init__(node_name, context=context)
        self.set_srv = self.create_service(
            SetParametersSrv, set_service_name, self.callback_set_parameters
        )
        self.get_srv = self.create_service(
            GetParametersSrv, get_service_name, self.callback_get_parameters
        )
        self.state = state

    def callback_set_parameters(self, request, response):
        # TODO edit the state
        print(f"Received request: {request}")
        results = []
        for p in request.parameters:
            value = rclpy.parameter.parameter_value_to_python(p.value)
            print(f"Name: {p.name} Value {value}")
            r = rcl_interfaces.msg.SetParametersResult()
            r.successful = True
            r.reason = "accepted"
            results.append(r)
        response.results = results
        return response

    def callback_get_parameters(self, request, response):
        # TODO implement
        return response


class ControlBoardData:
    def __init__(self):
        self.state = None
        self.context = None
        self.node = None
        self.executor = None
        self.initialized = False


def choose_domain_id(db) -> int:
    """
    Replicates ROS2Context selection logic:
      - if use_env_var and ROS_DOMAIN_ID exists: use it
      - otherwise use domain_id_input
    """
    domain_id_input = (
        db.inputs.domain_id
        if hasattr(db.inputs, "domain_id") and db.inputs.domain_id is not None
        else 0
    )
    use_env_var = (
        db.inputs.useDomainIDEnvVar
        if hasattr(db.inputs, "useDomainIDEnvVar")
        and db.inputs.useDomainIDEnvVar is not None
        else True
    )

    if use_env_var:
        s = os.environ.get("ROS_DOMAIN_ID")
        if s is not None:
            try:
                return int(s)
            except Exception:
                pass
    return int(domain_id_input)



def setup(db: og.Database):
    domain_id = choose_domain_id(db=db)
    db.per_instance_state.state = ControlBoardState()
    db.per_instance_state.context = ROS2Context()
    db.per_instance_state.context.init(domain_id=domain_id)
    db.per_instance_state.node = ControlBoardNode(
        node_name=settings.node_name,
        set_service_name=settings.node_set_parameters_service_name,
        get_service_name=settings.node_get_parameters_service_name,
        context=db.per_instance_state.context,
        state=db.per_instance_state.state,
    )
    db.per_instance_state.executor = SingleThreadedExecutor(
        context=db.per_instance_state.context
    )
    db.per_instance_state.executor.add_node(db.per_instance_state.node)
    db.per_instance_state.initialized = True


def cleanup(db: og.Database):
    if (
        not hasattr(db.per_instance_state, "initialized")
        or not db.per_instance_state.initialized
    ):
        return

    db.per_instance_state.executor.shutdown()
    db.per_instance_state.node.destroy_node()
    db.per_instance_state.context.destroy()

    db.per_instance_state.initialized = False


def compute(db: og.Database):
    if (
        not hasattr(db.per_instance_state, "initialized")
        or not db.per_instance_state.initialized
    ):
        setup(db)

    state = db.per_instance_state

    if rclpy.ok(context=state.context):
        state.executor.spin_once(timeout_sec=settings.node_timeout)

    return True


def internal_state():
    return ControlBoardData()

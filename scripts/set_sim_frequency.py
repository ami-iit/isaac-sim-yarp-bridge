import isaacsim.core.utils.stage as stage_utils
from pxr import PhysxSchema, Sdf, UsdPhysics

# Desired frequency in Hz
freq = 100

stage = stage_utils.get_current_stage()

# Add a physics scene prim to stage
scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/World/physicsScene"))

# Add PhysxSceneAPI
PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/World/physicsScene"))

physxSceneAPI = PhysxSchema.PhysxSceneAPI.Get(stage, "/World/physicsScene")
# Number of physics steps per second
physxSceneAPI.CreateTimeStepsPerSecondAttr(freq)

layer = stage.GetRootLayer()
# Number of rendering updates per second
# This frequency also affects the "Playback Tick" of the OmniGraph
layer.timeCodesPerSecond = freq

print(f"Simulator frequency changed to {freq}Hz.")

import isaacsim.core.utils.stage as stage_utils
from pxr import PhysxSchema, Sdf, UsdPhysics

# Desired frequency in Hz
freq = 100

stage = stage_utils.get_current_stage()

physics_scene_name = "/World/physicsScene"
# Add a physics scene prim to stage
UsdPhysics.Scene.Define(stage, Sdf.Path(physics_scene_name))

# Add PhysxSceneAPI
PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath(physics_scene_name))

physx_scene_api = PhysxSchema.PhysxSceneAPI.Get(stage, physics_scene_name)
# Number of physics steps per second
physx_scene_api.CreateTimeStepsPerSecondAttr(freq)

layer = stage.GetRootLayer()
# Number of rendering updates per second
# This frequency also affects the "Playback Tick" of the OmniGraph
layer.timeCodesPerSecond = freq

print(f"Simulator frequency changed to {freq}Hz.")

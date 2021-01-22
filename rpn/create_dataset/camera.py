import pybullet as p
import numpy as np

def front_view_matrix():
  return pose_ypr([0, 0, 0], 1.3, 180, -45)

def overhead_view_matrix():
  return pose_ypr([0, 0, 0], 1.3, 180, -90)

def pose_ypr(target_pos, distance, yaw, pitch, roll=0, up_axis=2):
  assert(len(target_pos) == 3)
  return p.computeViewMatrixFromYawPitchRoll(
    cameraTargetPosition=target_pos,
    distance=distance,
    yaw=yaw,
    pitch=pitch,
    roll=roll,
    upAxisIndex=up_axis
  )

def capture(
    height=240,
    width=320,
    viewMatrix=overhead_view_matrix(),
    fov=60,
    near=0.01,
    far=100.,
    renderer=p.ER_BULLET_HARDWARE_OPENGL
  ):
  aspect = float(width) / float(height)
  viewMatrix = list(viewMatrix)
  projectionMatrix = list(p.computeProjectionMatrixFOV(fov=fov, aspect=aspect, nearVal=near, farVal=far))
  width, height, rgb, depth, seg = p.getCameraImage(
    width=width,
    height=height,
    viewMatrix=viewMatrix,
    projectionMatrix=projectionMatrix,
    renderer=renderer,
  )
  rgb = np.reshape(rgb, (height, width, 4))
  depth = np.reshape(depth, (height, width))
  seg = np.reshape(seg, (height, width))
  return rgb, depth, seg

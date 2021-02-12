import json
import pickle
from camera import *
import matplotlib.pyplot as plt
import pybullet as p
import torch
from utils import *

def show():
  rgba, _, _ = capture()
  plt.imshow(rgba)
  plt.show()

def views_to_tensor():
  images = []
  for viewMatrix in [front_view_matrix(), overhead_view_matrix()]:
    rgb, _, _ = capture(viewMatrix=viewMatrix, renderer=p.ER_BULLET_HARDWARE_OPENGL)
    _, depth, segmentation = capture(viewMatrix=viewMatrix, renderer=p.ER_TINY_RENDERER)
    images.extend([np.expand_dims(img, 2) if len(img.shape) <= 2 else img for img in (rgb, depth, segmentation)])
  images = np.concatenate(images, -1)
  return images

def save_timestep(output_dir, episode_index, action_index):
  prefix = '{}/{}_{}'.format(output_dir, episode_index, action_index)

  # Restore via `p.restoreState('file.bullet')`
  p.saveBullet(prefix + '_state.bullet')

  # Collect tensor of visual data
  tensor = views_to_tensor()
  # Restore via `torch.load(file.pyt)`
  torch.save(tensor, prefix + '_views.pyt')

  # Collect ground-truth pose information
  pose_info = { body: get_pose(body) for body in get_bodies() }

  # Restore via `pickle.load(file.pkl)`
  json.dump(pose_info, open(prefix + '_poses.json', 'w'))

def save_episode(episode_info, output_dir, episode_index):
  prefix = '{}/{}'.format(output_dir, episode_index)
  json.dump(episode_info, open(prefix + '.json', 'w'))

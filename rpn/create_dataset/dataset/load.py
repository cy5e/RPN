import sys
sys.path.append('..')

import argparse
import matplotlib.pyplot as plt
import json
import pickle
import torch
import pybullet as p
import shutil
import os
import numpy as np

from utils import *
from init import *

def main(
    input_dir,
  ):

  # Import load objects used for data dumping
  #shutil.copyfile(input_dir + '/init_for_loading.py', 'init_for_loading.py')

  # Load dataset metadata
  metadata = json.load(open(input_dir + '/metadata.json', 'r'))
  num_episodes, episode_length, seed, all_actions = metadata['num_episodes'], metadata['episode_length'], metadata['seed'], metadata['actions']
  print('metadata:\n{}'.format(metadata))

  for episode_index in range(num_episodes):
    prefix = '{}/{}'.format(input_dir, episode_index)

    # Check to see if this episode was logged, skip if not.
    if not os.path.exists(prefix + '.json'):
      continue

    episode_info = json.load(open(prefix + '.json', 'r'))
    print(episode_info)

    world_states = episode_info['world_states']
    action_seq = episode_info['action_seq']
    python_random_state = episode_info['python_random_state']

    print(action_seq)
    for timestep, world_state in enumerate(world_states):
      print(world_state)

    for action_index in range(len(world_states) - 1):
      prefix = '{}/{}_{}'.format(input_dir, episode_index, action_index)

      # Check to see if this episode was logged, skip if not.
      if not os.path.exists(prefix + '_poses.json'):
        continue

      """
      `poses`: ground-truth poses of objects in scene, contained in dictionary mapping index to pose.
      Ex.)
        {
          0: ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
          1: ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
          ...
        }
      """
      poses = json.load(open(prefix + '_poses.json', 'r'))
      print('poses:\n{}'.format(poses))

      # Reload world state!
      #with pb_session(use_gui=False):
      #  world = load_objects(python_random_state)
      #  p.restoreState(fileName=prefix + '_state.bullet')

      """
      `image_tensor`: tensor of images for given episode and timestep.
      Shape: (240, 320, 12)
      Images are always 240 x 320.
      First 6 and last 6 channels correspond to different angles (default and overhead, respectively).
      First 4 channels (of 6) correspond to RGBA, then 1 channel depth, then 1 channel segmentation mask.
      REMEMBER: call .astype(np.int32) on RGBA images.
      """
      image_tensor = torch.load(prefix + '_views.pyt')
      print('image_tensor:\n{}'.format(image_tensor.shape))
      #plt.imshow(image_tensor[:, :, :4].astype(np.int32))
      #plt.show()

def parse_args():
  parser = argparse.ArgumentParser(description='Load data for Kitchen3D')
  parser.add_argument('--input_dir', default='dataset', type=str)
  return vars(parser.parse_args())

if __name__ == '__main__':
  args = parse_args()
  main(**args)

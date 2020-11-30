import sys
sys.path.append('..')
sys.path.append('../..')

from rpn.env_utils import World, URDFS, pb_session, world_saved

import argparse
import matplotlib.pyplot as plt
import json
import pickle
import torch
import pybullet as p
import shutil
import os
import numpy as np

def main(
    input_dir,
  ):

  # Import load objects used for data dumping
  shutil.copyfile(input_dir + '/init_for_loading.py', 'init_for_loading.py')
  from init_for_loading import load_objects

  # Load dataset metadata
  # Looks like: {'num_episodes': 3, 'episode_length': 10, 'num_containers': 2, 'num_ingredients': 4, 'num_cookwares': 2, 'num_objects': 2}
  metadata = json.load(open(input_dir + '/metadata.json', 'r'))
  num_episodes, episode_length = metadata['num_episodes'], metadata['episode_length']
  num_containers, num_ingredients, num_cookwares, num_objects = metadata['num_containers'], metadata['num_ingredients'], metadata['num_cookware'], metadata['num_objects']
  #print('metadata:\n{}'.format(metadata))

  print('Number of episodes: {}'.format(num_episodes))
  print('Episode lengths: {}'.format(episode_length))

  for episode_index in range(num_episodes):
    prefix = '{}/{}'.format(input_dir, episode_index)

    # Check to see if this episode was logged, skip if not.
    if not os.path.exists(prefix + '_ids.json'):
      continue

    """
    `ids`: list of ids in current episode.
    """
    ids = json.load(open(prefix + '_ids.json', 'r'))
    #print('ids:\n{}'.format(ids))

    """
    `action_seq`: list of actions (+ associated objects) executed in current episode.

    Ex.)
      [
        ('pick+place', (pot/1, stove/0)),
        ('pick+place', (plate/0, sink/0)),
        ('activate', (stove/0,)),
      ]
    """
    action_seq = pickle.load(open(prefix + '_actions.pkl', 'rb'))
    #print('action_seq:\n{}'.format(action_seq))

    """
    `random_state`: random state at start of episode.
    """
    random_state = pickle.load(open(prefix + '_random_state.pkl', 'rb'))

    for action_index in range(episode_length + 1):
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
      #print('poses:\n{}'.format(poses))

      """
      with pb_session(use_gui=False):
        world = load_objects(num_tools, num_objects, random_state)
        p.restoreState(fileName=prefix + '_state.bullet')
      """

      """
      `image_tensor`: tensor of images for given episode and timestep.

      Shape: (240, 320, 12)
      Images are always 240 x 320.
      First 6 and last 6 channels correspond to different angles (default and overhead, respectively).
      First 4 channels (of 6) correspond to RGBA, then 1 channel depth, then 1 channel segmentation mask.

      REMEMBER: call .astype(np.int32) on RGBA images.
      """
      image_tensor = torch.load(prefix + '_views.pyt')
      #print('image_tensor:\n{}'.format(image_tensor))
      #plt.imshow(image_tensor[:, :, :4].astype(np.int32))
      #plt.show()

def parse_args():
  parser = argparse.ArgumentParser(description='Load data for Kitchen3D')
  parser.add_argument('--input_dir', default='dataset', type=str)
  return vars(parser.parse_args())

if __name__ == '__main__':
  args = parse_args()
  main(**args)

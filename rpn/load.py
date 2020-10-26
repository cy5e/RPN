import sys
sys.path.append('..')

from rpn.env_utils import World, URDFS, pb_session, world_saved

import argparse
import json
import pickle
import torch
import pybullet as p
import shutil

def main(
    input_dir,
  ):

  # Import load objects used for data dumping
  shutil.copyfile(input_dir + 'load_objects.py', 'load_objects.py')
  from load_objects import load_objects

  # Load dataset metadata
  metadata = json.load(open(input_dir + '/metadata.json', 'r'))
  num_episodes, episode_length = metadata['num_episodes'], metadata['episode_length']

  for episode_index in range(num_episodes):
    prefix = '{}/{}'.format(input_dir, episode_index)

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

    """
    `action_seq`: list of actions (+ associated objects) executed in current episode.

    Ex.)
      [
        ('pick+place', (pot/1, stove/0)),
        ('pick+place', (plate/0, sink/0)),
        ('activate', (stove/0,)),
      ]
    """
    ids = json.load(open(prefix + '_ids.json', 'r'))

    for action_index in range(episode_length + 1):
      prefix = '{}/{}_{}'.format(input_dir, episode_index, action_index)
      with pb_session(use_gui=False):
        world = load_objects()
        p.restoreState(fileName=prefix + '_state.bullet')

      """
      `image_tensor`: tensor of images for given episode and timestep.

      Shape: (240, 320, 12)
      Images are always 240 x 320.
      First 6 and last 6 channels correspond to different angles (default and overhead, respectively).
      First 4 channels (of 6) correspond to RGBA, then 1 channel depth, then 1 channel segmentation mask.
      """
      image_tensor = torch.load(prefix + '_views.pyt')

      """
      `poses`: ground-truth poses of objects in scene, contained in dictionary mapping index to pose.

      Ex.)
        {
          0: ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
          1: ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
          ...
        }
      """
      poses = pickle.load(open(prefix + '_pose.pkl', 'rb'))

def parse_args():
    parser = argparse.ArgumentParser(description='Load data for Kitchen3D')
    parser.add_argument('--input_dir', default='experiment/', type=str)
    return vars(parser.parse_args())

if __name__ == '__main__':
  args = parse_args()
  main(**args)

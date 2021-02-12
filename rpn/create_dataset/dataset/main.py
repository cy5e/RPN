from __future__ import print_function

from init import load_objects, load_urdf
from objects import *
from planner import Planner
from utils import *
from actions import *
from serialization import *
from camera import *
from textures import load_textures

from collections import defaultdict
import argparse
import itertools
import json
import matplotlib.pyplot as plt
import os
import pickle
import pybullet as p
import random
import shutil
import time


def main(
    num_episodes=100, episode_length=3,
    seed=0, display=False, output_dir='dataset',
  ):
  # Create output directory
  os.makedirs(output_dir, exist_ok=True)

  # Reproducibility
  random.seed(seed)
  np.random.seed(seed)

  # Dump dataset metadata
  metadata = {
    'num_episodes': num_episodes,
    'episode_length': episode_length,
    'seed': seed,
    'actions': all_actions(),
  }

  json.dump(metadata, open(output_dir + '/metadata.json', 'w'))

  # Save dataset dumping script
  shutil.copyfile('init.py', output_dir + '/init.py')
  shutil.copyfile('load.py', output_dir + '/load.py')
  shutil.copyfile('main.py', output_dir + '/main.py')

  # Generate `num_episodes` number of datapoints
  for episode_index in range(num_episodes):
    with pb_session(use_gui=False):
      p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

      # Set GUI camera location
      p.resetDebugVisualizerCamera(
        cameraDistance=0.8,
        cameraYaw=0,
        cameraPitch=-89,
        cameraTargetPosition=[0, 0, 0]
      )

      python_random_state = random.getstate()

      # Load environment with objects
      world = load_objects(python_random_state)

      # Sample random action sequence
      action_seq = valid_action_sequence(world['types'], episode_length)

      if display:
        show()

      planner = Planner(world['types'])

      try:
        # Print action sequence
        print('Executing:')
        for action in action_seq:
          action_type, action_objs = action
          action_objs = tuple(world['types'][action_obj] for action_obj in action_objs)
          print(action_type, action_objs)

        # Save world state
        world_state = world.copy()
        save_timestep(output_dir, episode_index, 0)
        world_states = [world_state]

        # Execute action sequence
        for action_index in range(len(action_seq)):
          execute_action(action_seq, action_index, world, planner, load_textures())
          if display:
            show()

          # Save world state
          world_state = world.copy()
          save_timestep(output_dir, episode_index, action_index + 1)
          world_states.append(world_state)

        # If we're successful, save the episode!
        save_episode({
          'python_random_state': python_random_state,
          'world_states': world_states,
          'action_seq': action_seq,
        }, output_dir, episode_index)
      except Exception as e:
        print('Episode {} failed: {}'.format(episode_index, str(e)))

def parse_args():
  parser = argparse.ArgumentParser(description='Random data generation for Kitchen3D')
  parser.add_argument('--display', default=False, type=bool)
  parser.add_argument('--episode_length', default=7, type=int)
  parser.add_argument('--num_episodes', default=2, type=int)
  parser.add_argument('--output_dir', default='dataset', type=str)
  parser.add_argument('--seed', default=0, type=int)
  return vars(parser.parse_args())

if __name__ == '__main__':
  args = parse_args()
  main(**args)


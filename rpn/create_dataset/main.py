from __future__ import print_function

import sys
"""
sys.path.append('..')
sys.path.append('../..')
"""

from init import load_objects
from objects import *
from planner import Planner
from utils import *

from collections import defaultdict
import argparse
import itertools
import json
import matplotlib.pyplot as plt
import numpy as np
import os
import pickle
import pybullet as p
import random
import shutil
import time
import torch

from camera import capture, front_view_matrix, overhead_view_matrix

def valid_action_sequence(world, episode_length):
  env_table = [id for id, type in world.items() if type == 'table'][0]
  env_ingredients = [id for id, type in world.items() if type in INGREDIENTS]
  env_cookware = [id for id, type in world.items() if type in COOKWARE]
  env_containers = [id for id, type in world.items() if type in CONTAINERS]
  env_sinks = [id for id, type in world.items() if type == 'sink']
  env_stoves = [id for id, type in world.items() if type == 'stove']
  env_knives = [id for id, type in world.items() if type in KNIVES]

  actions = []
  while len(actions) < episode_length:
    ingredient = random.sample(env_ingredients, 1)[0]
    env_ingredients.remove(ingredient)

    peel = True if np.random.random() < 0.5 else False
    wash = True if np.random.random() < 0.5 else False
    cook = True if np.random.random() < 0.5 else False

    stove = random.sample(env_stoves, 1)[0]
    cookware = random.sample(env_cookware, 1)[0]
    sink = random.sample(env_sinks, 1)[0]
    container = random.sample(env_containers, 1)[0]

    if peel and len(env_knives) > 0:
      knife = random.sample(env_knives, 1)[0]
      actions.append(('peel', (ingredient, knife)))

    if wash:
      actions.append(('pick+place', (ingredient, sink)))
      actions.append(('activate', (sink,)))
      actions.append(('deactivate', (sink,)))

    if cook:
      actions.append(('pick+place', (cookware, stove)))
      actions.append(('pick+place', (ingredient, cookware)))
      actions.append(('activate', (stove,)))
      actions.append(('deactivate', (stove,)))

    actions.append(('pick+place', (ingredient, container)))

    if cook:
      actions.append(('pick+place', (cookware, env_table)))

  return actions

def execute_pick_and_place(planner, subj, dest, time_step=0.001):
  with world_saved():
    pick_plan, pick_pose = planner.plan('pick', (subj,))
    pick_command = ('pick', (subj, pick_pose), pick_plan)
  pick_plan.refine().execute(time_step=time_step)

  with world_saved():
    place_plan, place_pose = planner.plan('place', (subj, dest))
    place_command = ('place', (subj, dest, place_pose), place_plan)
  place_plan.refine().execute(time_step=time_step)

STOVE_ACTIVE_COLOR = [0.8, 0, 0, 1]
STOVE_INACTIVE_COLOR = [1, 1, 1, 1]
COOK_COLOR = [0.396, 0.263, 0.129, 1]

SINK_ACTIVE_COLOR = [0, 0, 0.8, 1]
SINK_INACTIVE_COLOR = [1, 1, 1, 1]
CLEAN_COLOR = [0.1, 0.1, 0.8, 0.5]

PEELED_SIZE = [0.8, 0.8, 0.8]
PEELED_COLOR = [0.396, 0.263, 0.129, 1]

def execute_activate(id, world):
  object_type = world[id]
  if object_type == 'sink':
    color = SINK_ACTIVE_COLOR
  elif object_type == 'stove':
    color = STOVE_ACTIVE_COLOR
  p.changeVisualShape(id, -1, rgbaColor=color)

def execute_deactivate(id, world):
  object_type = world[id]
  if object_type == 'sink':
    color = SINK_INACTIVE_COLOR
  elif object_type == 'stove':
    color = STOVE_INACTIVE_COLOR
  p.changeVisualShape(id, -1, rgbaColor=color)

def execute_peel(obj, knife, world):
  color = PEELED_COLOR
  p.changeVisualShape(id, -1, rgbaColor=color)

def execute_action(action, world, planner):
  action_type, ids = action[0], action[1]
  if action_type == 'pick+place':
    execute_pick_and_place(planner, ids[0], ids[1])
  elif action_type == 'activate':
    execute_activate(ids[0], world)
  elif action_type == 'deactivate':
    execute_deactivate(ids[0], world)
  elif action_type == 'peel':
    execute_peel(ids[0], ids[1], world)
  else:
    raise 'Invalid action.'

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

def save_timestep(world, output_dir, episode_index, action_index):
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

def save_episode(world, action_seq, output_dir, episode_index, random_state):
  prefix = '{}/{}'.format(output_dir, episode_index)

  # Restore via `pickle.load(file.pkl)`
  json.dump(world, open(prefix + '_ids.json', 'w'))

  # Restore via `pickle.load(file.pkl)`
  pickle.dump(action_seq, open(prefix + '_actions.pkl', 'wb'))

  # Restore via `pickle.load(file.pkl)`
  pickle.dump(random_state, open(prefix + '_random_state.pkl', 'wb'))

def main(
    num_episodes=100,
    episode_length=3,
    num_ingredients=3,
    num_containers=3,
    num_cookware=3,
    num_objects=2,
    seed=0,
    display=False,
    output_dir='dataset',
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
    'num_containers': num_containers,
    'num_ingredients': num_ingredients,
    'num_cookware': num_cookware,
    'num_objects': num_objects,
  }

  json.dump(metadata, open(output_dir + '/metadata.json', 'w'))

  # Save dataset dumping script
  shutil.copyfile('init.py', output_dir + '/init_for_loading.py')

  # Generate `num_episodes` number of datapoints
  for episode_index in range(num_episodes):
    with pb_session(use_gui=False):
      p.resetDebugVisualizerCamera(
        cameraDistance=0.8,
        cameraYaw=0,
        cameraPitch=-89,
        cameraTargetPosition=[0, 0, 0]
      )
      random_state = random.getstate()

      # Load environment with objects
      world = load_objects(num_ingredients, num_cookware, num_containers, num_objects, random_state)

      # Sample random action sequence
      action_seq = valid_action_sequence(world, episode_length)

      # Save action sequence for episode
      save_episode(world, action_seq, output_dir, episode_index, random_state)

      if display:
        show()

      save_timestep(world, output_dir, episode_index, 0)

      planner = Planner(world)

      # Execute action sequence
      print('Executing: {}'.format(action_seq))
      for action_index, action in enumerate(action_seq):
        print(action)
        execute_action(action, world, planner)
        save_timestep(world, output_dir, episode_index, action_index + 1)
        if display:
          show()

def parse_args():
  parser = argparse.ArgumentParser(description='Random data generation for Kitchen3D')
  parser.add_argument('--display', default=True, type=bool)
  parser.add_argument('--episode_length', default=7, type=int)
  parser.add_argument('--num_episodes', default=1, type=int)
  parser.add_argument('--num_ingredients', default=4, type=int)
  parser.add_argument('--num_cookware', default=2, type=int)
  parser.add_argument('--num_objects', default=2, type=int)
  parser.add_argument('--num_containers', default=2, type=int)
  parser.add_argument('--output_dir', default='dataset', type=str)
  parser.add_argument('--seed', default=0, type=int)
  return vars(parser.parse_args())

if __name__ == '__main__':
  args = parse_args()
  main(**args)

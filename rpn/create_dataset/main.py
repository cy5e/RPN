from __future__ import print_function

import sys
sys.path.append('..')
sys.path.append('../..')

from third_party.pybullet.utils.pybullet_tools.utils import WorldSaver, connect, dump_world, dump_body, get_pose, set_pose, Pose, \
    Point, set_camera, stable_z, create_box, create_cylinder, create_plane, HideOutput, load_model, \
    BLOCK_URDF, BLOCK1_URDF, get_configuration, SINK_URDF, STOVE_URDF, load_model, get_body_name, \
    disconnect, DRAKE_IIWA_URDF, PLATE_URDF, get_bodies, user_input, SHROOM_URDF, sample_placement, \
    get_movable_joints, pairwise_collision, stable_z, sample_placement_region, step_simulation
from third_party.pybullet.utils.pybullet_tools.kuka_primitives import Command

from third_party.pybullet.utils.pybullet_tools.perception import Camera
from bullet_envs import TaskEnvCook

from rpn.plan_utils import *
from rpn.env_utils import World, URDFS, pb_session, world_saved
from init import load_objects

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

FRUITS = ['apple', 'pear', 'lemon', 'banana', 'peach', 'orange', 'plum', 'strawberry']
VEGGIES = ['cabbage', 'tomato', 'pumpkin']
INGREDIENTS = FRUITS + VEGGIES

COOKWARES = ['pot', 'pan']
CONTAINERS = ['plate']
ACTIVATABLE = ['stove', 'sink']
GRASPABLE = INGREDIENTS + COOKWARES + CONTAINERS

def valid_pick_actions(env):
  def all_pick_actions(objs):
    for subject in objs:
      for dest in objs:
        yield ('pick+place', (subject, dest))

  def is_intuitively_valid(action):
    _, (subject, dest) = action
    subject_type = subject._name.split('/')[0]
    dest_type = dest._name.split('/')[0]
    if subject._name == dest._name:
      return False
    if subject_type in ['table', 'stove', 'sink']:
      return False
    if subject_type in ['pot', 'pan'] and dest_type in ['pot', 'pan']:
      return False
    if dest_type not in ['table', 'stove', 'sink', 'plate', 'pot', 'pan']:
      return False
    return True

  def is_valid(action):
    _, (subject, dest) = action
    return env.applicable('pick', env.objects.id(subject._name)) \
        and env.applicable('place', env.objects.id(subject._name), env.objects.id(dest._name)) \
        and is_intuitively_valid(action)

  return filter(is_valid, all_pick_actions(env.objects.tolist()))


def valid_activate_actions(env):
  def all_activate_actions(objs):
    for obj in objs:
      yield ('activate', (obj,))

  def is_valid(action):
    _, (obj,) = action
    return env.applicable('activate', env.objects.id(obj._name))

  return filter(is_valid, all_activate_actions(env.objects.tolist()))


def valid_actions(env):
  return itertools.chain(
    valid_pick_actions(env),
    valid_activate_actions(env)
  )


def valid_actions_given_state(env, state):
  def is_valid(action):
    if action[0] == 'activate' and action[1][0]._name in state['activated_objs']:
      return False
    return True
  return filter(is_valid, valid_actions(env))



def valid_action_sequence(env, episode_length):
  actions = []
  state = { 'activated_objs': set() }
  attempts = 0

  while len(actions) < episode_length:
    next_actions = list(valid_actions_given_state(env, state))

    if len(next_actions) == 0:
      action = None
    else:
      action = random.sample(next_actions, 1)[0]

    if action:
      actions.append(action)

      if action[0] == 'activate':
        state['activated_objs'].add(action[1][0]._name)
    else:
      attempts += 1
      actions = []
      state = { 'activated_objs': set() }

      if attempts >= 100:
        raise Exception('Can\'t create valid action sequence.')

  return actions


def valid_action_sequence(env, episode_length):
  env_table = None
  env_ingredients = []
  env_cookwares = []
  env_containers = []
  env_sinks = []
  env_stoves = []

  for obj in env.objects.tolist():
    id = env.objects.id(obj._name)
    if env.objects.type(id) in ['table']:
      env_table = id
    if env.objects.type(id) in INGREDIENTS:
      env_ingredients.append(id)
    elif env.objects.type(id) in COOKWARES:
      env_cookwares.append(id)
    elif env.objects.type(id) in CONTAINERS:
      env_containers.append(id)
    elif env.objects.type(id) in ['sink']:
      env_sinks.append(id)
    elif env.objects.type(id) in ['stove']:
      env_stoves.append(id)

  actions = []
  while len(actions) < episode_length:
    ingredient = random.sample(env_ingredients, 1)[0]

    wash = True if np.random.random() < 0.5 else False
    cook = True if np.random.random() < 0.5 else False

    sink = random.sample(env_sinks, 1)[0]
    stove = random.sample(env_stoves, 1)[0]
    cookware = random.sample(env_cookwares, 1)[0]
    container = random.sample(env_containers, 1)[0]

    env_ingredients.remove(ingredient)

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


def execute_action(action, planner, env):
  world = env.objects
  ids = action[1] # tuple(world.id(obj._name) for obj in action[1])
  if action[0] == 'pick+place':
    with world_saved():
      print(world.name(ids[0]))
      plan = planner.plan('pick', (ids[0],))
      pick_plan, pick_pose = plan
      pick_command = ('pick', (ids[0], pick_pose), pick_plan)
    with world_saved():
      plan = planner.plan('place', ids)
      place_plan, place_pose = plan
      place_command = ('place', (ids[0], ids[1], place_pose), place_plan)
    env.execute_command(pick_command)
    env.execute_command(place_command)
  elif action[0] == 'activate':
    dummy_cont_args = ((0, 0, 0), (0, 0, 0, 0))
    activate_command = ('activate', (world.id(ids[0]), dummy_cont_args), Command([]))
    env.execute_command(activate_command)
  elif action[0] == 'deactivate':
    dummy_cont_args = ((0, 0, 0), (0, 0, 0, 0))
    deactivate_command = ('deactivate', (world.id(ids[0]), dummy_cont_args), Command([]))
    env.execute_command(deactivate_command)
  else:
    raise 'Invalid action.'


def basic_view(camera):
  camera.set_pose_ypr([0, 0, 0], 1.3, 180, -45)


def overhead_view(camera):
  camera.set_pose_ypr([0, 0, 0], 1.3, 180, -90)


def show(camera, env):
  overhead_view(camera)
  rgba = camera.capture_raw()[0]
  plt.imshow(rgba)
  plt.show()

  rgba = camera.capture_raw()[2]
  plt.imshow(rgba)
  plt.show()


def show_angles(camera):
  for set_view in [basic_view, overhead_view]:
    set_view(camera)
    modalities = camera.capture_raw()
    for modal in modalities:
      plt.imshow(modal)
      plt.show()


def views_to_tensor(camera):
  orig_renderer = camera._renderer
  imgs = []
  for set_view in [basic_view, overhead_view]:
    set_view(camera)

    camera._renderer = p.ER_TINY_RENDERER
    _, depth, segmentation = camera.capture_raw()

    camera._renderer = p.ER_BULLET_HARDWARE_OPENGL
    rgb, _, _ = camera.capture_raw()

    imgs.extend([np.expand_dims(img, 2) if len(img.shape) <= 2 else img for img in (rgb, depth, segmentation)])
  camera._renderer = orig_renderer
  imgs = np.concatenate(imgs, -1)
  return imgs


def save_timestep(env, camera, output_dir, episode_index, action_index):
  world = env.objects
  prefix = '{}/{}_{}'.format(output_dir, episode_index, action_index)

  # Restore via `p.restoreState('file.bullet')`
  p.saveBullet(prefix + '_state.bullet')

  # Collect tensor of visual data
  tensor = views_to_tensor(camera)
  # Restore via `torch.load(file.pyt)`
  torch.save(tensor, prefix + '_views.pyt')

  # Collect ground-truth pose information
  pose_info = { body: get_pose(body) for body in get_bodies() }
  # Restore via `pickle.load(file.pkl)`
  json.dump(pose_info, open(prefix + '_poses.json', 'w'))


def save_episode(env, action_seq, output_dir, episode_index, random_state):
  world = env.objects
  prefix = '{}/{}'.format(output_dir, episode_index)

  # Save mapping of ids to object names (+ object type, implicitly)
  ids = { world.id(obj._name): obj._name for obj in world.tolist() }
  # Restore via `pickle.load(file.pkl)`
  json.dump(ids, open(prefix + '_ids.json', 'w'))

  # Restore via `pickle.load(file.pkl)`
  pickle.dump(action_seq, open(prefix + '_actions.pkl', 'wb'))

  # Restore via `pickle.load(file.pkl)`
  pickle.dump(random_state, open(prefix + '_random_state.pkl', 'wb'))


def main(
    num_episodes=100,
    episode_length=3,
    num_ingredients=3,
    num_containers=3,
    num_cookwares=3,
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

  # Set up camera
  camera = Camera(240, 320)

  # Dump dataset metadata
  metadata = {
    'num_episodes': num_episodes,
    'episode_length': episode_length,
    'num_containers': num_containers,
    'num_ingredients': num_ingredients,
    'num_cookwares': num_cookwares,
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

      #p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)

      random_state = random.getstate()

      # Load environment with objects
      env = TaskEnvCook(load_objects(num_ingredients, num_cookwares, num_containers, num_objects, random_state))

      env.reset()
      world = env.objects
      objs = world.tolist()
      planner = ActionPlanner(world)

      #p.changeVisualShape(world._robot, -1, rgbaColor=[0.5, 0.5, 0.5, 0])

      # Sample random action sequence
      action_seq = valid_action_sequence(env, episode_length)

      # Save action sequence for episode
      save_episode(env, action_seq, output_dir, episode_index, random_state)

      if display:
        show(camera, env)

      save_timestep(env, camera, output_dir, episode_index, 0)

      # Execute action sequence
      print('Executing: {}'.format(action_seq))
      continue_action_seq = True
      for action_index, action in enumerate(action_seq):
        if continue_action_seq:
          try:
            print(action)
            execute_action(action, planner, env)
            save_timestep(env, camera, output_dir, episode_index, action_index + 1)
            if display:
              show(camera, env)
          except Exception as e:
            print('Action failed: {}'.format(e))
            continue_action_seq = False


def parse_args():
  parser = argparse.ArgumentParser(description='Random data generation for Kitchen3D')
  parser.add_argument('--display', default=False, type=bool)
  parser.add_argument('--episode_length', default=3, type=int)
  parser.add_argument('--num_episodes', default=100, type=int)
  parser.add_argument('--num_ingredients', default=4, type=int)
  parser.add_argument('--num_cookwares', default=2, type=int)
  parser.add_argument('--num_objects', default=2, type=int)
  parser.add_argument('--num_containers', default=2, type=int)
  parser.add_argument('--output_dir', default='dataset', type=str)
  parser.add_argument('--seed', default=0, type=int)
  return vars(parser.parse_args())


if __name__ == '__main__':
  args = parse_args()
  main(**args)

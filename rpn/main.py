from __future__ import print_function

import sys
sys.path.append('..')

from third_party.pybullet.utils.pybullet_tools.utils import WorldSaver, connect, dump_world, dump_body, get_pose, set_pose, Pose, \
    Point, set_camera, stable_z, create_box, create_cylinder, create_plane, HideOutput, load_model, \
    BLOCK_URDF, BLOCK1_URDF, get_configuration, SINK_URDF, STOVE_URDF, load_model, get_body_name, \
    disconnect, DRAKE_IIWA_URDF, PLATE_URDF, get_bodies, user_input, HideOutput, SHROOM_URDF, sample_placement, \
    get_movable_joints, pairwise_collision, stable_z, sample_placement_region, step_simulation
from third_party.pybullet.utils.pybullet_tools.kuka_primitives import Command

from third_party.pybullet.utils.pybullet_tools.perception import Camera
from bullet_envs import TaskEnvCook

from rpn.plan_utils import *
from rpn.env_utils import World, URDFS, pb_session, world_saved

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



def valid_pick_actions(env):
  def all_pick_actions(objs):
    for subject in objs:
      for dest in objs:
        yield ('pick+place', (subject, dest))

  def intuitively_valid(action):
    _, (subject, dest) = action
    subject_type = subject._name.split('/')[0]
    dest_type = dest._name.split('/')[0]
    if subject._name == dest._name:
      return False
    if subject_type in ['table', 'sink', 'stove']:
      return False
    if subject_type == 'cabbage':
      return dest_type == 'pot'
    if dest_type not in ['sink', 'stove']:
      return False

  def is_valid(action):
    _, (subject, dest) = action
    return env.applicable('pick', env.objects.id(subject._name)) and \
      env.applicable('place', env.objects.id(subject._name), env.objects.id(dest._name)) and \
      intuitively_valid(action)

  return filter(is_valid, all_pick_actions(env.objects.tolist()))


def valid_activate_actions(env):
  def all_activate_actions(objs):
    for obj in objs:
      yield ('activate', (obj,))

  """
  def is_valid(action):
    _, (obj,) = action
    obj_type = obj._name.split('/')[0]
    return obj_type in ['sink', 'stove']
  """

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
  while len(actions) < episode_length:
    action = random.sample(list(valid_actions_given_state(env, state)), 1)[0]
    if action:
      actions.append(action)
      if action[0] == 'activate':
        state['activated_objs'].add(action[1][0]._name)
    else:
      actions = []
      state = { 'activated_objs': set() }
  return actions


def valid_action_sequences(env, episode_length, num_episodes):
  return [valid_action_sequence(env, episode_length) for _ in range(num_episodes)]


def execute_action(action, planner, env):
  """
  world = env.objects
  ids = tuple(world.id(obj._name) for obj in action[1])
  if action[0] == 'pick+place':
    planner.plan('pick', ids[:1])
    planner.plan('place', ids)
  elif action[0] == 'activate':
    planner.plan('activate', ids)
  else:
    raise 'Invalid action.'
  """
  world = env.objects
  ids = tuple(world.id(obj._name) for obj in action[1])
  if action[0] == 'pick+place':
    with world_saved():
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
  else:
    raise 'Invalid action.'


def basic_view(camera):
  camera.set_pose_ypr([0, 0, 0], 1.5, 180, -45)


def overhead_view(camera):
  camera.set_pose_ypr([0, 0, 0], 1.5, 180, -90)


def show(camera):
  basic_view(camera)
  rgb = camera.capture_raw()[0]
  plt.imshow(rgb)
  plt.show()


def show_angles(camera):
  for set_view in [basic_view, overhead_view]:
    set_view(camera)
    modalities = camera.capture_raw()
    for modal in modalities:
      plt.imshow(modal)
      plt.show()


def views_to_tensor(camera):
  basic_view(camera)
  imgs = []
  for set_view in [basic_view, overhead_view]:
    set_view(camera)
    modalities = camera.capture_raw()
    imgs.extend([np.expand_dims(modal, 2) if len(modal.shape) <= 2 else modal for modal in modalities])
  return np.concatenate(imgs, -1)


def load_objects(): #num_metatools, num_tools, num_objects):
  with HideOutput():
    world = World(['table', 'stove', 'sink', 'plate', 'pot', 'cabbage'])
    world.load_robot(URDFS['ph_gripper'])

    """
    path, type_name, fixed=False, n_copy=1, init_pose=None, randomly_place_on=None, color=None

    metatools = [
      { 'path': URDFS['stove'], 'type_name': 'stove', 'fixed': True, 'globalScaling': 0.8 }
      { 'path': URDFS['sink'], 'type_name': 'sink', 'fixed': True }
    ]

    tools = [
      { 'path': URDFS['pot'], 'type_name': 'pot', 'fixed': False }
    ]

    objs = [
      { 'path': URDFS['cabbage'], 'type_name': 'cabbage', 'fixed': False }
      { 'path': URDFS['plate'], 'type_name': 'plate', 'fixed': False }
    ]

    for i in range(num_metatools):
      metatool = metatools[i % len(metatools)]
      world.load_object(**metatool)

    for i in range(num_tools):
      tool = random.sample(tools, 1)
      world.load_object(**tool)

    for i in range(num_objects):
      obj = random.sample(objects, 1)
      world.load_object(**obj)
    """

    world.load_object(URDFS['short_floor'], 'table', fixed=True, globalScaling=0.6)
    world.load_object(URDFS['stove'], 'stove', fixed=True, globalScaling=0.8)
    world.load_object(URDFS['sink'], 'sink', fixed=True)
    world.load_object(URDFS['plate'], 'plate', fixed=False)
    world.load_object(URDFS['pot'], 'pot', fixed=False)
    world.load_object(URDFS['pot'], 'pot', fixed=False)
    world.load_object(URDFS['cabbage'], 'cabbage', fixed=False)

    world.place(world.id('stove/0'), world.id('table/0'), [[0.4, 0, 0], [0, 0, 0, 1]])
    world.place(world.id('sink/0'), world.id('table/0'), [[-0.2, -0.4, 0], [0, 0, 0, 1]])
    world.place(world.id('plate/0'), world.id('table/0'),   [[-0.4, 0, 0], [0, 0, 0, 1]])

    world.random_place(world.id('pot/0'), world.id('table/0'))
    world.random_place(world.id('pot/1'), world.id('table/0'))
    world.random_place(world.id('cabbage/0'), world.id('table/0'), np.array([[-0.3, -0.2], [0.3, 0.2]]))
  return world


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


def save_episode(env, action_seq, output_dir, episode_index):
  world = env.objects
  prefix = '{}/{}'.format(output_dir, episode_index)

  # Save mapping of ids to object names (+ object type, implicitly)
  ids = { world.id(obj._name): obj._name for obj in world.tolist() }
  # Restore via `pickle.load(file.pkl)`
  json.dump(ids, open(prefix + '_ids.json', 'w'))

  # Restore via `pickle.load(file.pkl)`
  pickle.dump(action_seq, open(prefix + '_actions.pkl', 'wb'))


def main(
    num_episodes=1000,
    episode_length=3,
    seed=0,
    display=False,
    output_dir='experiment/',
  ):
  # Create output directory
  os.makedirs(output_dir, exist_ok=True)

  # Reproducibility
  np.random.seed(seed)

  # Set up camera
  camera = Camera(240, 320)

  # Dump dataset metadata
  metadata = { 'num_episodes': num_episodes, 'episode_length': episode_length }
  json.dump(metadata, open(output_dir + '/metadata.json', 'w'))

  # Save dataset dumping script
  shutil.copyfile('main.py', output_dir + '/load_objects.py')

  # Generate `num_episodes` number of datapoints
  for episode_index in range(num_episodes):
    with pb_session(use_gui=False):
      # Load environment with objects
      env = TaskEnvCook(load_objects())

      env.reset()
      world = env.objects
      objs = world.tolist()
      planner = ActionPlanner(world)

      # Sample random action sequence
      action_seq = valid_action_sequence(env, episode_length)

      # Save action sequence for episode
      save_episode(env, action_seq, output_dir, episode_index)

      if display:
        show(camera)

      save_timestep(env, camera, output_dir, episode_index, 0)
      # Execute action sequence
      print('Executing: {}'.format(action_seq))
      for action_index, action in enumerate(action_seq):
        execute_action(action, planner, env)
        save_timestep(env, camera, output_dir, episode_index, action_index + 1)

        if display:
          show(camera)


def parse_args():
  parser = argparse.ArgumentParser(description='Random data generation for Kitchen3D')
  parser.add_argument('--num_episodes', default=2, type=int)
  parser.add_argument('--episode_length', default=3, type=int)
  parser.add_argument('--display', default=False, type=bool)
  parser.add_argument('--output_dir', default='experiment/', type=str)
  parser.add_argument('--seed', default=0, type=int)
  return vars(parser.parse_args())


if __name__ == '__main__':
  args = parse_args()
  main(**args)

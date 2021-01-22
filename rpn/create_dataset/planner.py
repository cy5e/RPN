from __future__ import print_function

import numpy as np

import sys
import time

from kuka_primitives import BodyPose, BodyConf, Command, get_grasp_gen, get_stable_gen, get_ik_fn, get_free_motion_gen, get_holding_motion_gen, get_movable_collision_test
from utils import get_pose, set_pose, set_default_camera, get_configuration, HideOutput, get_movable_joints, is_center_stable, end_effector_from_body, multiply, invert, world_saved

def follow_instructions(instruction, world, teleport, resolution_factor=0.05):
  planner = ActionPlanner(world, resolution_factor=resolution_factor)
  for inst in instruction:
    if inst[0] == 'on':
      pick_args = [world.id(inst[1])]
      place_args = [world.id(inst[1]), world.id(inst[2])]
      with world_saved():
        pick_plan, pick_pose = planner.plan('pick', pick_args, teleport=teleport)
      yield 'pick', pick_args + [pick_pose], pick_plan
      with world_saved():
        place_plan, place_pose = planner.plan('place', place_args, teleport=teleport)
      yield 'place', place_args + [place_pose], place_plan
    else:
      raise NotImplementedError('%s is not implemented' % inst[0])

def goal_to_motion_plan(goals, env, teleport, motion_resolution=0.05):
  world = env.objects
  planner = ActionPlanner(world, resolution_factor=motion_resolution)
  if len(goals) > 2 or len(goals) == 0:
    yield None, None, None, 'MP'

  # exactly one goal should be actionable
  actionable_predicates = ['on', 'activated']
  actionable_goals = [g for g in goals if g.predicate in actionable_predicates]
  # print(actionable_goals)
  if len(actionable_goals) != 1:
    yield None, None, None, 'MP'
  goal = actionable_goals[0]

  dummy_cont_args = ((0, 0, 0), (0, 0, 0, 0))
  if goal.predicate == 'on':
    if not env.applicable('pick', world.id(goal.name1)) or not env.applicable('place', world.id(goal.name1), world.id(goal.name2)):
      yield None, None, None, 'MP'

    pick_args = [world.id(goal.name1)]
    place_args = [world.id(goal.name1), world.id(goal.name2)]
    with world_saved():
      plan = planner.plan('pick', pick_args, teleport=teleport)
      if plan is None:
        yield None, None, None, 'MP'
      pick_plan, pick_pose = plan
    yield 'pick', pick_args + [pick_pose], pick_plan, -1
    with world_saved():
      plan = planner.plan('place', place_args, teleport=teleport)
      if plan is None:
        yield None, None, None, 'MP'
      place_plan, place_pose = plan
    yield 'place', place_args + [place_pose], place_plan, -1
  elif goal.predicate == 'activated':
    if not env.applicable('activate', world.id(goal.name1)):
      yield None, None, None, 'MP'
    yield 'activate', [world.id(goal.name1), dummy_cont_args], Command([]), -1
  else:
    raise NotImplementedError('%s is not implemented' % goal.predicate)

def plan_pick(robot, target, grasp_gen, fixed, teleport, resolutions=None):
  ik_fn = get_ik_fn(robot, fixed=fixed, teleport=teleport, resolutions=resolutions)
  free_motion_fn = get_free_motion_gen(robot, fixed=([target] + fixed), teleport=teleport, resolutions=resolutions)
  pose0 = BodyPose(target)
  conf0 = BodyConf(robot)
  for grasp, in grasp_gen(target):
    with world_saved():
      result1 = ik_fn(target, pose0, grasp)
      if result1 is None:
        continue
      conf1, path2 = result1
      pose0.assign()
      result2 = free_motion_fn(conf0, conf1)
      if result2 is None:
        continue
      path1, = result2
      return Command(path1.body_paths + path2.body_paths), grasp
  return None

def plan_place(robot, holding, target, place_gen, grasp, fixed, teleport, conf0=None, resolutions=None):
  ik_fn = get_ik_fn(robot, fixed=fixed, teleport=teleport, resolutions=resolutions)
  holding_motion_fn = get_holding_motion_gen(robot, fixed=fixed, teleport=teleport, resolutions=resolutions)

  if conf0 is None:
    conf0 = BodyConf(robot)

  # TODO: recompute grasp
  for pose_place, in place_gen(holding, target):
    with world_saved():
      result1 = ik_fn(holding, pose_place, grasp)
      if result1 is None:
        continue
      # reuse grasping approach_q here since the planned path is symmetric
      approach_q, path2 = result1
      place_path = path2.reverse()
      result2 = holding_motion_fn(conf0, approach_q, holding, grasp)
      if result2 is None:
        continue
      approach_path, = result2

      # compute placing pose WRT to the target object
      gripper_pose_world = end_effector_from_body(pose_place.pose, grasp.grasp_pose)
      gripper_pose_target = multiply(invert(gripper_pose_world), get_pose(target))

      return Command(approach_path.body_paths + place_path.body_paths), gripper_pose_target
  return None

def get(type, world):
  for id, type_i in sorted(world.items()):
    if type_i == type:
      return id

class Planner(object):
  def __init__(self, world, resolution_factor=0.05, bottom_percent=0.0):
    self.world = world
    self._state = {}
    self._resolution_factor = resolution_factor
    self._bottom_percent = bottom_percent

  def plan(self, action_name, object_args, cont_args=None, teleport=False):
    robot = get('robot', self.world)
    fixed_ids = list(self.world.keys())
    ids = list(self.world.keys())

    resolutions = np.ones(len(get_movable_joints(robot))) * self._resolution_factor
    if action_name == 'pick':
      grasp_gen = get_grasp_gen(robot, 'top')
      plan = plan_pick(robot, object_args[0], grasp_gen, fixed_ids, teleport=teleport, resolutions=resolutions)
      if plan is None:
        return None
      pick, grasp = plan
      self._state['grasp'] = grasp
      return pick, grasp.grasp_pose
    elif action_name == 'place':
      place_gen = get_stable_gen(fixed=ids, bottom_percent=self._bottom_percent)
      plan = plan_place(robot, object_args[0], object_args[1], place_gen, self._state.pop('grasp'), fixed_ids, teleport=False, resolutions=resolutions)
      if plan is None:
        return None
      place, place_pose_target = plan
      return place, place_pose_target
    else:
      raise NotImplementedError('Unimplemented action %s' % action_name)

from collections import defaultdict
from objects import * # URDFS, ALL, ENV_OBJECTS, FRUITS, VEGGIES, INGREDIENTS, PEELABLE, COOKWARE, CONTAINERS, ACTIVATABLE, GRASPABLE, OBJECTS, KNIVES
from utils import *
import numpy as np
import pybullet as p
import random

METATOOL_LOCATIONS = [
  [[0.35, -0.45, 0], [0, 0, 0, 1]], # Top left
  [[-0.3, -0.45, 0], [0, 0, 0, 1]], # Top right
  [[0.3, 0.45, 0], [0, 0, 0, 1]],   # Bottom left
  [[-0.3, 0.45, 0], [0, 0, 0, 1]],  # Bottom right
]

def load_model(path, fixed=True, **kwargs):
  if path.endswith('.urdf'):
    body = p.loadURDF(path, useFixedBase=fixed, flags=0, physicsClientId=CLIENT, **kwargs)
  elif path.endswith('.sdf'):
    body = p.loadSDF(path, physicsClientId=CLIENT, **kwargs)
  elif path.endswith('.xml'):
    body = p.loadMJCF(path, physicsClientId=CLIENT, **kwargs)
  elif path.endswith('.bullet'):
    body = p.loadBullet(path, physicsClientId=CLIENT, **kwargs)
  else:
    raise ValueError(path)
  return body

def setup():
  robot = load_model(URDFS['ph_gripper'])
  table = load_model(URDFS['table'], fixed=True, globalScaling=0.6)
  return { robot: 'robot', table: 'table' }

def place_metatools(table):
  sampled_locations = random.sample(METATOOL_LOCATIONS, 2)

  stove = load_model(URDFS['stove'], fixed=True, globalScaling=0.8)
  place(stove, table, sampled_locations[0])

  sink = load_model(URDFS['sink'], fixed=True)
  place(sink, table, sampled_locations[1])

  return { stove: 'stove', sink: 'sink' }

def get(type, world):
  for id, type_i in sorted(world.items()):
    if type_i == type:
      return id

def place(body, surface, pose):
  z = stable_z(body, surface)
  pose[0][-1] = z
  (point, quat) = pose
  p.resetBasePositionAndOrientation(body, point, quat, physicsClientId=CLIENT)

def random_place(body, surface, fixed=(), region=None, max_attempt=10):
  for _ in range(max_attempt):
    if region is None:
      pose = sample_placement(body, surface, percent=0.6)
    else:
      pose = sample_placement_region(body, surface, region=region, percent=0.3)
    set_pose(body, pose)
    if (pose is None) or any(pairwise_collision(body, b) for b in fixed):
      continue
    return pose
  return False

def sample_and_place_objects(world, types_and_counts):
  table = get('table', world)
  objs = {}
  for population, count in types_and_counts:
    for _ in range(count):
      type_name = random.sample(population, 1)[0]
      obj = load_model(URDFS[type_name], fixed=False)
      random_place(obj, table, world)
      objs.update({ obj: type_name })
  return objs

def load_objects(num_ingredients, num_cookware, num_containers, num_objects, random_state):
  random.setstate(random_state)
  types_and_counts = [
    (COOKWARE, num_cookware),
    (CONTAINERS, num_containers),
    (INGREDIENTS, num_ingredients),
    (OBJECTS, num_objects),
  ]
  with HideOutput():
    world = setup()
    table = get('table', world)
    world.update(place_metatools(table))
    world.update(sample_and_place_objects(world, types_and_counts))
  return world


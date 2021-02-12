from collections import defaultdict
from objects import *
from utils import *
import pybullet as p
import random
import numpy as np

METATOOL_LOCATIONS = [
  [[ 0.35, -0.45, 0], [0, 0, 0, 1]], # Top left
  [[-0.30, -0.45, 0], [0, 0, 0, 1]], # Top right
  [[ 0.30,  0.45, 0], [0, 0, 0, 1]], # Bottom left
  [[-0.30,  0.45, 0], [0, 0, 0, 1]], # Bottom right
]

def load_model(path, fixed=True, **kwargs):
  try:
    if path.endswith('.urdf'):
      return p.loadURDF(path, useFixedBase=fixed, flags=0, physicsClientId=CLIENT, **kwargs)
    elif path.endswith('.sdf'):
      return p.loadSDF(path, physicsClientId=CLIENT, **kwargs)
    elif path.endswith('.xml'):
      return p.loadMJCF(path, physicsClientId=CLIENT, **kwargs)
    elif path.endswith('.bullet'):
      return p.loadBullet(path, physicsClientId=CLIENT, **kwargs)
    else:
      raise ValueError(path)
  except Exception as e:
    raise Exception('Failed with path {}. {}'.format(path, str(e)))

def load_urdf(urdf_type, **kwargs):
  return load_model(URDFS[urdf_type], **kwargs)

def setup_env():
  robot = load_urdf('ph_gripper')
  table = load_urdf('table', fixed=True, globalScaling=0.6)
  return { robot: 'robot', table: 'table' }, { robot: 1.0, table: 0.6 }

def place_metatools(table):
  sampled_locations = random.sample(METATOOL_LOCATIONS, 3)

  stove = load_urdf('stove', fixed=True, globalScaling=0.8)
  place(stove, table, sampled_locations[0])

  sink = load_urdf('sink', fixed=True)
  place(sink, table, sampled_locations[1])

  drying_rack = load_urdf('drying_rack', fixed=True)
  place(drying_rack, table, sampled_locations[2])

  return { stove: 'stove', sink: 'sink', drying_rack: 'drying_rack' }, { stove: 0.8, sink: 1.0, drying_rack: 1.0 }

def get(type, types):
  for id, type_i in sorted(types.items()):
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

def sample_scene():
  scene_type = random.choice(['produce', 'toast', 'pour'])

  if scene_type == 'produce':
    return [
      (COOKWARE, 2),
      (CONTAINERS, 2),
      #(PRODUCE, 3),
      (['orange', 'apple'], 3),
      (['knife'], 1),
    ]
  elif scene_type == 'toast':
    return [
      (COOKWARE, 1),
      (CONTAINERS, 2),
      (PRODUCE, 2),
      (['bread'], 2),
      (['toaster'], 1),
    ]
  elif scene_type == 'pour':
    return [
      (FILLED_DRINKS, 2),
      (RECEPTICLES, 2),
      (PRODUCE, 2),
      (COOKWARE, 1),
      (CONTAINERS, 1),
    ]
  else:
    raise Exception('Should not have happened.')

def sample_and_place_objects(types):
  types_and_counts = sample_scene()
  table = get('table', types)
  types = {}
  for population, count in types_and_counts:
    for _ in range(count):
      type_name = random.sample(population, 1)[0]
      obj = load_urdf(type_name, fixed=False)
      random_place(obj, table, types.keys())
      types.update({ obj: type_name })
  return types, { x: 1.0 for x in types }

def load_objects(python_random_state):
  random.setstate(python_random_state)

  with HideOutput():
    types, scales = setup_env()
    table = get('table', types)

    new_types, new_scales = place_metatools(table)
    types.update(new_types)
    scales.update(new_scales)

    new_types, new_scales = sample_and_place_objects(types)
    types.update(new_types)
    scales.update(new_scales)

  children = defaultdict(list)
  pybullet_ids = {id: id for id in types}

  world = {
    'pybullet_ids': pybullet_ids,
    'types': types,
    'scales': scales,
    'children': children,
    'textures': defaultdict(lambda: [1, 1, 1, 1]),
  }

  return world


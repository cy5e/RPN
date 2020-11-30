from rpn.env_utils import World, pb_session, world_saved
from third_party.pybullet.utils.pybullet_tools.utils import HideOutput
from third_party.pybullet.utils.pybullet_tools.perception import Camera

from objects import *
from collections import defaultdict
import random
import numpy as np

METATOOL_LOCATIONS = [
  [[0.35, -0.45, 0], [0, 0, 0, 1]], # Top left
  [[-0.3, -0.45, 0], [0, 0, 0, 1]], # Top right
  [[0.3, 0.45, 0], [0, 0, 0, 1]],   # Bottom left
  [[-0.3, 0.45, 0], [0, 0, 0, 1]],  # Bottom right
]

def _setup(world):
  world.load_robot(URDFS['ph_gripper'])
  world.load_object(URDFS['table'], 'table', fixed=True, globalScaling=0.6)

def _place_metatools(world):
  sampled_locations = random.sample(METATOOL_LOCATIONS, 2)

  world.load_object(URDFS['stove'], 'stove', fixed=True, globalScaling=0.8)
  world.place(world.id('stove/0'), world.id('table/0'), sampled_locations[0])

  world.load_object(URDFS['sink'], 'sink', fixed=True)
  world.place(world.id('sink/0'), world.id('table/0'), sampled_locations[1])

def _sample_and_place_objects(world, types_and_counts):
  counts = defaultdict(int)
  for population, count in types_and_counts:
    for _ in range(count):
      type_name = random.sample(population, 1)[0]
      obj = { 'path': URDFS[type_name], 'type_name': type_name, 'fixed': False }
      world.load_object(**obj)
      obj_index = str(counts[type_name])
      world.random_place(world.id(type_name + '/' + obj_index), world.id('table/0'))
      counts[type_name] += 1

def load_objects(num_ingredients, num_cookware, num_containers, num_objects, random_state):
  random.setstate(random_state)
  with HideOutput():
    world = World(ALL)

    _setup(world)
    _place_metatools(world)

    types_and_counts = [
      (COOKWARE, num_cookware),
      (CONTAINERS, num_containers),
      (INGREDIENTS, num_ingredients),
      (OBJECTS, num_objects),
    ]

    _sample_and_place_objects(world, types_and_counts)
  return world

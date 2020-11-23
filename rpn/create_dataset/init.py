from rpn.env_utils import World, URDFS, pb_session, world_saved
from third_party.pybullet.utils.pybullet_tools.utils import WorldSaver, connect, dump_world, dump_body, get_pose, set_pose, Pose, \
    Point, set_camera, stable_z, create_box, create_cylinder, create_plane, HideOutput, load_model, \
    BLOCK_URDF, BLOCK1_URDF, get_configuration, SINK_URDF, STOVE_URDF, load_model, get_body_name, \
    disconnect, DRAKE_IIWA_URDF, PLATE_URDF, get_bodies, user_input, SHROOM_URDF, sample_placement, \
    get_movable_joints, pairwise_collision, stable_z, sample_placement_region, step_simulation
from collections import defaultdict
import random
import numpy as np
from third_party.pybullet.utils.pybullet_tools.perception import Camera

FRUITS = ['apple', 'pear', 'lemon', 'banana', 'peach', 'orange', 'plum', 'strawberry']
VEGGIES = ['cabbage', 'tomato', 'pumpkin']
INGREDIENTS = FRUITS + VEGGIES

COOKWARES = ['pot', 'pan']
CONTAINERS = ['plate']
ACTIVATABLE = ['stove', 'sink']
GRASPABLE = INGREDIENTS + COOKWARES + CONTAINERS

def load_objects(num_ingredients, num_cookwares, num_containers, num_objects, random_state):
  random.setstate(random_state)

  with HideOutput():
    world = World([
      'table', 'stove', 'sink', 'pot', 'pan', 'plate', 'banana', 'strawberry', 'apple',
      'lemon', 'peach', 'pear', 'orange', 'plum', 'tomato', 'pumpkin', 'cabbage', 'mug',
      'mug', 'bowl', 'coke_bottle', 'flower', 'knife', 'powerade', 'soap', 'brown_bowl',
      'soda_can', 'coke_can', 'spaghettios', 'steak_knife', 'wine_bottle', 'coffee_mug',
      'drying_rack',
    ])

    world.load_robot(URDFS['ph_gripper'])
    world.load_object(URDFS['short_floor'], 'table', fixed=True, globalScaling=0.6)

    metatool_locations = [
        [[0.35, -0.45, 0], [0, 0, 0, 1]], # Top left
        [[-0.3, -0.45, 0], [0, 0, 0, 1]], # Top right
        [[0.3, 0.45, 0], [0, 0, 0, 1]],   # Bottom left
        [[-0.3, 0.45, 0], [0, 0, 0, 1]],  # Bottom right
    ]

    sampled_metatool_locations = random.sample(metatool_locations, 2)

    world.load_object(URDFS['stove'], 'stove', fixed=True, globalScaling=0.8)
    world.place(world.id('stove/0'), world.id('table/0'), sampled_metatool_locations[0])

    world.load_object(URDFS['sink'], 'sink', fixed=True)
    world.place(world.id('sink/0'), world.id('table/0'), sampled_metatool_locations[1])

    counts = defaultdict(int)

    cookware = [
      { 'path': URDFS['pot'], 'type_name': 'pot', 'fixed': False },
      { 'path': URDFS['pan'], 'type_name': 'pan', 'fixed': False },
    ]

    for i in range(num_cookwares):
      obj = random.sample(cookware, 1)[0]
      type_name = obj['type_name']
      world.load_object(**obj)
      world.random_place(world.id(type_name + '/' + str(counts[type_name])), world.id('table/0'))
      counts[type_name] += 1

    container = [
      { 'path': URDFS['plate'], 'type_name': 'plate', 'fixed': False },
    ]

    for i in range(num_containers):
      obj = random.sample(container, 1)[0]
      type_name = obj['type_name']
      world.load_object(**obj)
      world.random_place(world.id(type_name + '/' + str(counts[type_name])), world.id('table/0'))
      counts[type_name] += 1

    ingredients = [
      { 'path': URDFS['banana'], 'type_name': 'banana', 'fixed': False },
      { 'path': URDFS['strawberry'], 'type_name': 'strawberry', 'fixed': False },
      { 'path': URDFS['apple'], 'type_name': 'apple', 'fixed': False },
      { 'path': URDFS['lemon'], 'type_name': 'lemon', 'fixed': False },
      { 'path': URDFS['peach'], 'type_name': 'peach', 'fixed': False },
      { 'path': URDFS['pear'], 'type_name': 'pear', 'fixed': False },
      { 'path': URDFS['orange'], 'type_name': 'orange', 'fixed': False },
      { 'path': URDFS['plum'], 'type_name': 'plum', 'fixed': False },
      { 'path': URDFS['tomato'], 'type_name': 'tomato', 'fixed': False },
      { 'path': URDFS['pumpkin'], 'type_name': 'pumpkin', 'fixed': False },
      { 'path': URDFS['cabbage'], 'type_name': 'cabbage', 'fixed': False },
    ]

    for i in range(num_ingredients):
      obj = random.sample(ingredients, 1)[0]
      type_name = obj['type_name']
      world.load_object(**obj)
      world.random_place(world.id(type_name + '/' + str(counts[type_name])), world.id('table/0'))
      counts[type_name] += 1

    objects = [
      { 'path': URDFS['mug'], 'type_name': 'mug', 'fixed': False },
      { 'path': URDFS['bowl'], 'type_name': 'bowl', 'fixed': False },
      { 'path': URDFS['coke_bottle'], 'type_name': 'coke_bottle', 'fixed': False },
      { 'path': URDFS['knife'], 'type_name': 'knife', 'fixed': False },
      { 'path': URDFS['powerade'], 'type_name': 'powerade', 'fixed': False },
      { 'path': URDFS['soap'], 'type_name': 'soap', 'fixed': False },
      { 'path': URDFS['brown_bowl'], 'type_name': 'brown_bowl', 'fixed': False },
      { 'path': URDFS['soda_can'], 'type_name': 'soda_can', 'fixed': False },
      { 'path': URDFS['coke_can'], 'type_name': 'coke_can', 'fixed': False },
      { 'path': URDFS['spaghettios'], 'type_name': 'spaghettios', 'fixed': False },
      { 'path': URDFS['steak_knife'], 'type_name': 'steak_knife', 'fixed': False },
      { 'path': URDFS['wine_bottle'], 'type_name': 'wine_bottle', 'fixed': False },
      { 'path': URDFS['coffee_mug'], 'type_name': 'coffee_mug', 'fixed': False },
      { 'path': URDFS['drying_rack'], 'type_name': 'drying_rack', 'fixed': False },
    ]

    for i in range(num_objects):
      obj = random.sample(objects, 1)[0]
      type_name = obj['type_name']
      world.load_object(**obj)
      world.random_place(world.id(type_name + '/' + str(counts[type_name])), world.id('table/0'))
      counts[type_name] += 1

  return world

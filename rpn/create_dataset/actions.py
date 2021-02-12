from collections import defaultdict
import numpy as np
import random
from objects import *
import pybullet as p
from utils import *
from init import load_urdf


def categorize(types, categories):
  objects = defaultdict(set)
  # Get mapping category -> ids in category
  for id, type in types.items():
    for category, category_objects in categories.items():
      if type in category_objects:
        objects[category].add(id)
  return objects

def all_actions():
  return [
    'pour', 'toast', 'peel', 'activate', 'deactivate', 'pick+place',
  ]

def valid_action_sequence(types, episode_length):
  categories = {
    'table': ['table'],
    'food': FOOD,
    'cookware': COOKWARE,
    'containers': CONTAINERS,
    'sinks': ['sink'],
    'stoves': ['stove'],
    'knives': KNIVES,
    'toasters': ['toaster'],
    'drying_racks': ['drying_rack'],
    'filled_drinks': FILLED_DRINKS,
    'recepticles': RECEPTICLES,
    'kitchenware': KITCHENWARE,
  }

  objects = categorize(types, categories)

  table = random.sample(objects['table'], 1)[0]
  sink = random.sample(objects['sinks'], 1)[0]
  stove = random.sample(objects['stoves'], 1)[0]
  drying_rack = random.sample(objects['drying_racks'], 1)[0]

  actions = []

  # While there exist things to be washed, we get a coin flip, and there are still actions
  coin_flip = random.random() < 0.5
  while len(objects['kitchenware']) > 0 and coin_flip and len(actions) < episode_length:
    # Sample kitchenware item
    kitchenware = random.sample(objects['kitchenware'], 1)[0]
    objects['kitchenware'].remove(kitchenware)

    # Wash item
    actions.append(('pick+place', (kitchenware, sink)))
    actions.append(('activate', (sink,)))
    actions.append(('deactivate', (sink,)))
    actions.append(('pick+place', (kitchenware, drying_rack)))
    actions.append(('pick+place', (kitchenware, table)))

    coin_flip = random.random() < 0.5

  while len(actions) < episode_length:
    sample_type = random.choice(['pour', 'prepare food'])

    # Pour liquid
    if sample_type == 'pour':
      if len(objects['filled_drinks']) > 0 and len(objects['recepticles']) > 0:
        # Sample and remove drink
        drink = random.sample(objects['filled_drinks'], 1)[0]
        objects['filled_drinks'].remove(drink)

        # Sample and remove cup
        cup = random.sample(objects['recepticles'], 1)[0]
        objects['recepticles'].remove(cup)

        actions.append(('pour', (drink, cup)))

    # Prepare food item
    elif sample_type == 'prepare food':
      cookware = random.sample(objects['cookware'], 1)[0]
      container = random.sample(objects['containers'], 1)[0]

      food = random.sample(objects['food'], 1)[0]
      food_type = types[food]

      if food_type == 'bread' and len(objects['toasters']) > 0:
        toaster = random.sample(objects['toasters'], 1)[0]
        actions.append(('pick+place', (food, toaster)))
        actions.append(('activate', (toaster,)))
        actions.append(('deactivate', (toaster,)))
        actions.append(('pick+place', (food, container)))

      elif food_type in PRODUCE:
        peel = random.random() < 0.5
        if peel and (food_type in PEELABLE_FOOD) and len(objects['knives']) > 0:
          knife = random.sample(objects['knives'], 1)[0]
          actions.append(('pick+place', (food, knife)))
          actions.append(('eel', (food, knife)))

        wash = random.random() < 0.5
        if wash and not peel and (food_type in WASHABLE_FOOD):
          actions.append(('pick+place', (food, sink)))
          actions.append(('activate', (sink,)))
          actions.append(('deactivate', (sink,)))

        cook = random.random() < 0.5
        if cook and (food_type in COOKABLE_FOOD):
          actions.append(('pick+place', (cookware, stove)))
          actions.append(('pick+place', (food, cookware)))
          actions.append(('activate', (stove,)))
          actions.append(('deactivate', (stove,)))

        actions.append(('pick+place', (food, container)))

        if cook:
          actions.append(('pick+place', (cookware, list(objects['table'])[0])))

        objects['food'].remove(food)
    else:
      raise Exception('Should not happen... (1)')

  return actions

def execute_pick_and_place(subj, dest, world, planner, time_step=0.001):
  pb_subj = world['pybullet_ids'][subj]
  pb_dest = world['pybullet_ids'][dest]

  types = world['types']
  children = world['children']

  if len(children[subj]) > 0:
    object_type = types[subj]
    raise Exception('Trying to move object {} (type: {}) with the following children: {}'.format(subj, object_type, children[subj]))

  with world_saved():
    pick_plan, pick_pose = planner.plan('pick', (pb_subj,))
    pick_command = ('pick', (pb_subj, pick_pose), pick_plan)
  pick_plan.refine().execute(time_step=time_step)

  with world_saved():
    place_plan, place_pose = planner.plan('place', (pb_subj, pb_dest))
    place_command = ('place', (pb_subj, pb_dest, place_pose), place_plan)
  place_plan.refine().execute(time_step=time_step)

  for obj_i, children_i in children.items():
    count = 0
    if subj in children_i:
      children_i.remove(subj)
      count += 1
    if count > 1:
      # Means that object was a child to two parents?
      print('This shouldn\'t happen... (0)')

  children[dest].append(subj)

def dfs(node, graph, f):
  def visit(node, graph, visited, f):
    visited.add(node)
    f(node)
    for child in graph[node]:
      if child not in visited:
        visit(child, graph, visited, f)
  visit(node, graph, set(), f)

def execute_activate(id, world, textures):
  types = world['types']
  children = world['children']
  object_type = types[id]

  if object_type == 'sink':
    # Wash immediate child
    update_texture(id, world, textures, 'SINK_ACTIVE_COLOR')
    for child in children[id]:
      update_texture(child, world, textures, 'WASHED_COLOR')

  elif object_type == 'stove':
    # DFS to find food and cook it
    def change_to_cooked(id):
      if types[id] in COOKABLE_FOOD:
        update_texture(id, world, textures, 'COOKED_COLOR')

    update_texture(id, world, textures, 'STOVE_ACTIVE_COLOR')
    dfs(id, children, change_to_cooked)

  elif object_type == 'toaster':
    # DFS to find food and cook it
    def change_to_cooked(id):
      if types[id] in ['bread']:
        update_texture(id, world, textures, 'TOASTED')

    update_texture(id, world, textures, 'STOVE_ACTIVE_COLOR')
    dfs(id, children, change_to_cooked)

  else:
    raise Exception('Cannot activate object with type {}'.format(object_type))

def execute_deactivate(id, world, textures):
  children = world['children']
  object_type = world['types'][id]

  if object_type == 'sink':
    update_texture(id, world, textures, 'SINK_INACTIVE_COLOR')
  elif object_type == 'stove':
    update_texture(id, world, textures, 'STOVE_INACTIVE_COLOR')
  elif object_type == 'toaster':
    update_texture(id, world, textures, 'STOVE_INACTIVE_COLOR')
  else:
    raise Exception('Cannot deactivate object with type {}'.format(object_type))

def update_texture(id, world, textures, texture_name):
  texture = textures[texture_name]
  if type(texture) == list:
    p.changeVisualShape(world['pybullet_ids'][id], -1, rgbaColor=texture)
  else:
    p.changeVisualShape(world['pybullet_ids'][id], -1, textureUniqueId=texture)
  world['textures'][id] = texture

def fill_recepticle(source, sink, world, textures):
  type_name = world['types'][source]
  if type_name == 'powerade':
    update_texture(sink, world, textures, 'POWERADE')
  elif type_name == 'wine_bottle':
    update_texture(sink, world, textures, 'WINE')
  elif 'coke' in type_name.lower():
    update_texture(sink, world, textures, 'COLA')
  else:
    update_texture(sink, world, textures, 'FILLED_COLOR')

def execute_pour(source, sink, world, textures):
  update_texture(source, world, textures, 'EMPTY_COLOR')
  fill_recepticle(source, sink, world, textures)

def execute_peel(obj, knife, world, textures):
  types = world['types']
  scales = world['scales']

  # Retrieve type and scale information
  type_name = types[obj]
  scale = scales[obj]

  # Save pose and remove object
  pose = p.getBasePositionAndOrientation(world['pybullet_ids'][obj], physicsClientId=CLIENT)
  p.removeBody(world['pybullet_ids'][obj], physicsClientId=CLIENT)

  # Create new object, set pose
  new_obj = load_urdf(type_name, globalScaling=0.7*scale)
  p.resetBasePositionAndOrientation(new_obj, pose[0], pose[1], physicsClientId=CLIENT)

  # Update world state to include new id
  world['pybullet_ids'][obj] = new_obj

  # Maps type to peel color
  def get_peel_texture(type_name, textures):
    if type_name == 'orange':
      return 'LIGHTORANGE'
    elif type_name == 'banana':
      return 'PEELED_BANANA'
    else:
      return 'LIGHTYELLOW'

  # Update texture to be lighter color, as if peeled
  new_texture = get_peel_texture(type_name, textures)
  update_texture(obj, world, textures, new_texture)

def execute_toast(bread, toaster, world, textures):
  # Changes obj color given toaster
  update_texture(bread, world, textures, 'TOASTED')

def execute_action(action_seq, action_index, world, planner, textures):
  action = action_seq[action_index]
  action_type, ids = action[0], action[1]
  if action_type == 'pick+place':
    execute_pick_and_place(ids[0], ids[1], world, planner)
  elif action_type == 'activate':
    execute_activate(ids[0], world, textures)
  elif action_type == 'deactivate':
    execute_deactivate(ids[0], world, textures)
  elif action_type == 'peel':
    execute_peel(ids[0], ids[1], world, textures)
  elif action_type == 'pour':
    execute_pour(ids[0], ids[1], world, textures)
  elif action_type == 'toast':
    execute_toast(ids[0], ids[1], world, textures)
  else:
    raise 'Invalid action.'

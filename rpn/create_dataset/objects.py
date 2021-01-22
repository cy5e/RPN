URDFS = {
  'ph_gripper': 'models/drake/objects/gripper_invisible.urdf',
  'table': 'models/short_floor.urdf',
  # -----------------------------------------------------------
  'plate': 'models/ycb/029_plate/textured.urdf',
  'pot': 'models/dinnerware/pan_tefal.urdf',
  'pan': 'models/skillet/pan_tefal.urdf',
  'cracker_box': 'models/ycb/003_cracker_box/textured.urdf',
  'master_chef_can': 'models/ycb/002_master_chef_can/textured.urdf',
  'banana': 'models/ycb/011_banana/textured.urdf',
  'strawberry': 'models/ycb/012_strawberry/textured.urdf',
  'apple': 'models/ycb/013_apple/textured.urdf',
  'lemon': 'models/ycb/014_lemon/textured.urdf',
  'peach': 'models/ycb/015_peach/textured.urdf',
  'pear': 'models/ycb/016_pear/textured.urdf',
  'orange': 'models/ycb/017_orange/textured.urdf',
  'plum': 'models/ycb/018_plum/textured.urdf',
  'cabbage': 'models/ingredients/cabbage/textured.urdf',
  'tomato': 'models/ingredients/tomato/textured.urdf',
  'pumpkin': 'models/ingredients/pumpkin/textured.urdf',
  'stove': 'models/cooktop/textured.urdf',
  'sink': 'models/sink/tray.urdf',
  # ------------------------- NEW -----------------------------
  'mug': 'models/new/mug/models/textured.urdf',
  'bowl': 'models/new/bowl/models/textured.urdf',
  'coke_bottle': 'models/new/coke_bottle/models/textured.urdf',
  'flower': 'models/new/flower/models/textured.urdf',
  'knife': 'models/new/knife/models/textured.urdf',
  'powerade': 'models/new/powerade/models/textured.urdf',
  'soap': 'models/new/soap/models/textured.urdf',
  'brown_bowl': 'models/new/brown_bowl/models/textured.urdf',
  'soda_can': 'models/new/soda_can/models/textured.urdf',
  'coke_can': 'models/new/coke_can/models/textured.urdf',
  'spaghettios': 'models/new/spaghettios/models/textured.urdf',
  'steak_knife': 'models/new/steak_knife/models/textured.urdf',
  'wine_bottle': 'models/new/wine_bottle/models/textured.urdf',
  'coffee_mug': 'models/new/coffee_mug/models/textured.urdf',
  'drying_rack': 'models/new/drying_rack/models/textured.urdf',
}

ALL = list(URDFS.keys())

ENV_OBJECTS = ['ph_gripper', 'short_floor']

FRUITS = ['apple', 'pear', 'lemon', 'banana', 'peach', 'orange', 'plum', 'strawberry']
VEGGIES = ['cabbage', 'tomato', 'pumpkin']
INGREDIENTS = FRUITS + VEGGIES
PEELABLE = FRUITS + VEGGIES

COOKWARE = ['pot', 'pan']
CONTAINERS = ['plate']
ACTIVATABLE = ['stove', 'sink']

GRASPABLE = INGREDIENTS + COOKWARE + CONTAINERS

OBJECTS = ['mug', 'bowl', 'coke_bottle', 'knife', 'powerade', 'soap', 'brown_bowl', 'soda_can', 'coke_can', 'spaghettios', 'steak_knife', 'wine_bottle', 'coffee_mug', 'drying_rack']
KNIVES = ['knife']

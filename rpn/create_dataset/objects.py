from ordered_set import OrderedSet

URDFS = {
  'ph_gripper': 'models/drake/objects/gripper_invisible.urdf',
  'table': 'models/short_floor.urdf',
  # -----------------------------------------------------------
  'apple': 'models/ycb/013_apple/textured.urdf',
  'banana': 'models/ycb/011_banana/textured.urdf',
  'cabbage': 'models/ingredients/cabbage/textured.urdf',
  'cracker_box': 'models/ycb/003_cracker_box/textured.urdf',
  'lemon': 'models/ycb/014_lemon/textured.urdf',
  'master_chef_can': 'models/ycb/002_master_chef_can/textured.urdf',
  'orange': 'models/ycb/017_orange/textured.urdf',
  'pan': 'models/skillet/pan_tefal.urdf',
  'peach': 'models/ycb/015_peach/textured.urdf',
  'pear': 'models/ycb/016_pear/textured.urdf',
  'plate': 'models/new/plate/textured.urdf',
  'plum': 'models/ycb/018_plum/textured.urdf',
  'pot': 'models/dinnerware/pan_tefal.urdf',
  'pumpkin': 'models/ingredients/pumpkin/textured.urdf',
  'sink': 'models/sink/tray.urdf',
  'stove': 'models/cooktop/textured.urdf',
  'strawberry': 'models/ycb/012_strawberry/textured.urdf',
  'tomato': 'models/ingredients/tomato/textured.urdf',
  # ------------------------- NEW -----------------------------
  #'flower': 'models/new/flower/models/textured.urdf',
  'bowl': 'models/new/bowl/models/textured.urdf',
  'brown_bowl': 'models/new/brown_bowl/models/textured.urdf',
  'chips_can': 'models/new/chips_can/textured.urdf',
  'coffee_mug': 'models/new/coffee_mug/models/textured.urdf',
  'coke_bottle': 'models/new/coke_bottle/models/textured.urdf',
  'coke_can': 'models/new/coke_can/models/textured.urdf',
  'drying_rack': 'models/new/drying_rack/models/textured.urdf',
  'gelatin_box': 'models/new/gelatin_box/model.urdf',
  'knife': 'models/new/knife/models/textured.urdf',
  'mug': 'models/new/mug/models/textured.urdf',
  'mustard': 'models/new/mustard/model.urdf',
  'potted_meat_can': 'models/new/potted_meat_can/model.urdf',
  'powerade': 'models/new/powerade/models/textured.urdf',
  'soap': 'models/new/soap/models/textured.urdf',
  'soda_can': 'models/new/soda_can/models/textured.urdf',
  'spaghettios': 'models/new/spaghettios/models/textured.urdf',
  'steak_knife': 'models/new/steak_knife/models/textured.urdf',
  'tomato_soup_can': 'models/new/tomato_soup_can/model.urdf',
  'wine_bottle': 'models/new/wine_bottle/models/textured.urdf',

  'toaster': 'models/new/toaster/textured.urdf',
  'bread': 'models/new/bread/textured.urdf',
}

# -------------- Meta ----------------

ALL = OrderedSet(URDFS.keys())
ENV_OBJECTS = OrderedSet(['ph_gripper', 'table'])
METATOOLS = OrderedSet(['stove', 'sink', 'drying_rack'])
CREATABLE = ALL - ENV_OBJECTS - METATOOLS

# ------------------------------------

# -------------- Food ----------------

FRUITS = OrderedSet(['apple', 'pear', 'lemon', 'banana', 'peach', 'orange', 'plum', 'strawberry'])
VEGGIES = OrderedSet(['cabbage', 'tomato', 'pumpkin'])
PRODUCE = FRUITS | VEGGIES

PEELABLE_FOOD = PRODUCE - OrderedSet(['strawberry'])
WASHABLE_FOOD = PRODUCE
COOKABLE_FOOD = PRODUCE

FOOD = PRODUCE | OrderedSet(['bread'])

# ------------------------------------

# ---------- KitchenWare --------------

CONTAINERS = OrderedSet(['plate', 'bowl', 'brown_bowl'])
COOKWARE = OrderedSet(['pot', 'pan'])
KNIVES = OrderedSet(['knife'])

KITCHENWARE = CONTAINERS | COOKWARE | KNIVES
WASHABLE_KITCHENWARE = COOKWARE

# -------------------------------------

# ------------- Objects --------------

FILLED_DRINKS = OrderedSet(['coke_can', 'coke_bottle', 'wine_bottle', 'powerade'])
RECEPTICLES = OrderedSet(['mug'])

GRASPABLE = CREATABLE

OBJECTS = CREATABLE - KITCHENWARE - FOOD

# ------------------------------------


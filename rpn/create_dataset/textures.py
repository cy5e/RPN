import pybullet as p

def load_textures():
  return {
    # Activate (cook)
    'STOVE_ACTIVE_COLOR': [0.8, 0, 0, 1],
    'STOVE_INACTIVE_COLOR': [1, 1, 1, 1],
    'COOKED_COLOR': [0.396, 0.263, 0.129, 1],
    'BURNT': p.loadTexture("textures/burnt.jpg"),

    # Activate (wash)
    'SINK_ACTIVE_COLOR': [0, 0, 0.8, 1],
    'SINK_INACTIVE_COLOR': [1, 1, 1, 1],
    'WASHED_COLOR': [0.1, 0.1, 0.8, 0.5],

    # Toast
    'TOASTED': p.loadTexture("textures/toasted.jpg"),

    # Pour
    'EMPTY_COLOR': [1, 1, 1, 0.9],
    'FILLED_COLOR': p.loadTexture("textures/lightblue.jpg"),
    'WINE': p.loadTexture("textures/wine.jpg"),
    'POWERADE': p.loadTexture("textures/powerade.jpg"),
    'COLA': p.loadTexture("textures/cola.jpg"),

    # Peel
    'WHITE': p.loadTexture("textures/white.jpg"),
    'LIGHTYELLOW': p.loadTexture("textures/lightyellow.jpg"),
    'LIGHTORANGE': p.loadTexture("textures/lightorange.jpg"),
    'PEELED_BANANA': p.loadTexture("textures/peeled_banana.jpg"),
  }

from copy import deepcopy
from json import dump, load
from typing import Any, Dict


def load_scene(scene_path: str) -> Dict[str, Any]:
  with open(scene_path, 'r') as scene_file:
    return load(scene_file)


def output_scene(output_path: str, scene: Dict[str, Any]):
  with open(output_path, 'w') as output_file:
    dump(scene, output_file)


X_POSITIONS = [-0.25, -0.12, 0.0, 0.12, 0.25]
Y_POSITIONS = [0.1, -0.1, -0.1, 0.1]
Z_POSITIONS = [0.53, 0.93]


def generate_new_stick(scene: Dict[str, Any], idx: int) -> Dict[str, Any]:
  last_red_stick = [obj for obj in scene['objects'] if obj['name'] == f'red_stick_{idx - 3}'][0]
  new_red_stick = deepcopy(last_red_stick)
  stick_num = idx - 2
  new_red_stick['name'] = f'red_stick_{stick_num}'
  t = new_red_stick['pose']['translation']
  t[0] = X_POSITIONS[(stick_num - 1) // 4]
  t[1] = Y_POSITIONS[stick_num % 4]
  t[2] = Z_POSITIONS[stick_num % 2]
  print(f'Stick {stick_num} is at {t}')

  scene['objects'].append(new_red_stick)
  return scene


if __name__ == '__main__':
  scene = load_scene('pb_3_initial_scene.json')
  for i in range(4, 23):
    scene = generate_new_stick(scene, i)
    output_scene(f'pb_{i}_initial_scene.json', scene)

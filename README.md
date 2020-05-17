# visual-servoing-simulator-pybullet
A visual servoing simulator for research in both robotic learning algorithms and classical visual servoing methods.

## Install

```
pip install -e .
```

## Play

```
python -i vis_playground/play.py
```

## Use

```
import time
import gym
import vs_simulator
import vs_playground
from vs_playground.utils import *

config_file_path = 'vs_point2point.yaml'

if __name__ == '__main__':
    config_file_path = vs_playground.lib_path + '/configs/' + config_file_path
    env_name = configure(config_file_path)['env_name']
    print('loading env {}'.format(env_name))
    env = gym.make(env_name, config_file=config_file_path)

```

## Visual Servoing

TODO

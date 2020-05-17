import os
import vs_simulator
from gym.envs.registration import register

"""
======= system variables ==========
"""
lib_path = os.path.dirname(vs_simulator.__file__)


"""
======= gym environments ==========
"""
register(
    id='panda-point2point-v0',
    entry_point='vs_simulator.panda_point2point_env:PandaPoint2PointEnv'
)



import math
import pybullet


def euler_angles_to_quaternions(eulers, bullet_client):
    return bullet_client.getQuaternionFromEuler([math.radians(eulers[i]) for i in range(len(eulers))])

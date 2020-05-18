import math
import vs_simulator
import pybullet


def euler_angles_to_quaternions(eulers, bullet_client):
    return bullet_client.getQuaternionFromEuler([math.radians(eulers[i]) for i in range(len(eulers))])


def create_sphere(radius, color, position, bullet_client, fixed=False):
    obj_file = vs_simulator.lib_path + '/data/scene/objects/sphere.obj'
    mesh_scale = [radius/0.03, radius/0.03, radius/0.03]
    visual_shape = bullet_client.createVisualShape(shapeType=bullet_client.GEOM_MESH,
                                                   fileName=obj_file,
                                                   meshScale=mesh_scale,
                                                   rgbaColor=color)
    collision_shape = bullet_client.createCollisionShape(shapeType=bullet_client.GEOM_MESH,
                                                         meshScale=mesh_scale,
                                                         fileName=obj_file,)
    mass = 0 if fixed else 0.0001
    sphere_id = bullet_client.createMultiBody(baseMass=mass,
                                              baseCollisionShapeIndex=collision_shape,
                                              baseVisualShapeIndex=visual_shape,
                                              basePosition=position)

    return sphere_id

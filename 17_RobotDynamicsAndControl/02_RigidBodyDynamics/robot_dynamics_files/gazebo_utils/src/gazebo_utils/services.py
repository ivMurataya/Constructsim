import rospy, math, numpy as np

from geometry_msgs.msg import Wrench
from tf.transformations import quaternion_matrix
from std_srvs.srv import Empty
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import (SetModelState, GetModelState, ApplyBodyWrench,
                             GetPhysicsProperties, SetPhysicsProperties)

def reset_world():
    try:
        reset_world_srv = rospy.ServiceProxy("/gazebo/reset_world", Empty)
        reset_world_srv()
    except rospy.ServiceException, e:
        print "/gazebo/reset_world call failed: %s"%e

def set_model_state(name, new_model_state):
    try:
        set_model_state_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        set_model_state_srv(new_model_state)
    except rospy.ServiceException, e:
        print "/gazebo/set_model_state call failed: %s"%e

def set_pose(name, pose):
    new_model_state = ModelState()
    new_model_state.model_name = name
    new_model_state.pose = pose
    set_model_state(name, new_model_state)

def get_model_state(model_name, relative_entity_name='world'):
    try:
        get_model_state_srv = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        ms = get_model_state_srv(model_name, relative_entity_name)
        if ms.success:
            return ms
        else:
            return None
    except rospy.ServiceException, e:
        print "/gazebo/get_model_state call failed: %s"%e
        return None

def get_pose(model_name):
    ms = get_model_state(model_name)
    if ms:
        return ms.pose 
    else:
        return None 

def get_twist(model_name):
    ms = get_model_state(model_name)
    if ms:
        return ms.twist 
    else:
        return None 

def get_physics_properties():
    try:
        get_phys_prop_srv = rospy.ServiceProxy("/gazebo/get_physics_properties", GetPhysicsProperties)
        p = get_phys_prop_srv()
    except rospy.ServiceException, e:
        print "/gazebo/get_physics_properties call failed: %s"%e
        p = []
    return p

def set_physics_properties(time_step, max_update_rate, gravity, ode_config):
    try:
        set_phys_prop_srv = rospy.ServiceProxy("/gazebo/set_physics_properties", SetPhysicsProperties)
        set_phys_prop_srv(time_step, max_update_rate, gravity, ode_config)
    except rospy.ServiceException, e:
        print "/gazebo/set_physics_properties call failed: %s"%e

def disable_gravity():
    p = get_physics_properties()
    if p:
        p.gravity.z = 0
        set_physics_properties(p.time_step, p.max_update_rate, p.gravity, p.ode_config)

def enable_gravity():
    p = get_physics_properties()
    if p:
        p.gravity.z = -9.8
        set_physics_properties(p.time_step, p.max_update_rate, p.gravity, p.ode_config)

def apply_body_wrench_global(name, wrench, duration):
    try:
        apply_body_wrench_srv = rospy.ServiceProxy("/gazebo/apply_body_wrench", ApplyBodyWrench)
        return apply_body_wrench_srv(body_name=name+'::link', reference_frame=name+'::link',
                              wrench=wrench, duration=duration)
    except rospy.ServiceException, e:
        print "/gazebo/apply_body_wrench call failed: %s"%e

def skew_symmetric(ux,uy,uz):
    return [[  0,-uz, uy],
            [ uz,  0,-ux],
            [-uy, ux,  0]]

def apply_body_wrench(name, wrench, duration):
    pose = get_pose(name)
    q = np.array([pose.orientation.x, pose.orientation.y, 
                  pose.orientation.z, pose.orientation.w])
    H = quaternion_matrix(q)
    A = H[0:3,0:3]
    #sk = np.array(skew_symmetric(-pose.position.x, -pose.position.y, -pose.position.z))
    #T = np.block([
    #    [A, -np.dot(A,sk)],
    #    [np.zeros((3,3)), A]
    #])
    T = np.block([
        [A, np.zeros((3,3))],
        [np.zeros((3,3)), A]
    ])
    w = np.zeros((6,1))
    w[0] = wrench.force.x
    w[1] = wrench.force.y
    w[2] = wrench.force.z
    w[3] = wrench.torque.x
    w[4] = wrench.torque.y
    w[5] = wrench.torque.z
    gw = np.dot(T,w)
    global_wrench = Wrench()
    global_wrench.force.x = gw[0]
    global_wrench.force.y = gw[1]
    global_wrench.force.z = gw[2]
    global_wrench.torque.x = gw[3]
    global_wrench.torque.y = gw[4]
    global_wrench.torque.z = gw[5]
    apply_body_wrench_global(name, global_wrench, duration)
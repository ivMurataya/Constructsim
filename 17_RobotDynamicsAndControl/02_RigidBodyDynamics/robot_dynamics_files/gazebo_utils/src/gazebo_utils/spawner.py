import rospy, rospkg
from gazebo_msgs.srv import SpawnModel

class Spawner:
    def __init__(self):
        rospack = rospkg.RosPack()
        self.models_path=rospack.get_path('gazebo_utils') + '/models/'
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        self.proxy = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        
    def spawn(self,model, name, pose):
        with open(self.models_path + model + "/model.sdf", "r") as f:
            model_xml = f.read()
        self.proxy(name, model_xml, "", pose, "world")

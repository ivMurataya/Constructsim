controller_manager_msgs.srv.ListControllers_Response(
controller=
[controller_manager_msgs.msg.ControllerState(
    name='joint_state_broadcaster', state='active', type='joint_state_broadcaster/JointStateBroadcaster', 
    claimed_interfaces=[], required_command_interfaces=[], 
    required_state_interfaces=['joint1/effort', 'joint1/position', 'joint1/velocity', 'joint2/effort', 'joint2/position', 'joint2/velocity'])

, controller_manager_msgs.msg.ControllerState(
    name='forward_position_controller', state='active', type='forward_command_controller/ForwardCommandController', 
    claimed_interfaces=['joint1/position', 'joint2/position'], 
    required_command_interfaces=['joint1/position', 'joint2/position'], required_state_interfaces=[])])




response:
controller_manager_msgs.srv.ListControllers_Response(
controller=[
    controller_manager_msgs.msg.ControllerState(
        name='joint_state_broadcaster', state='active', type='joint_state_broadcaster/JointStateBroadcaster', 
            claimed_interfaces=[], required_command_interfaces=[], 
            required_state_interfaces=['joint1/position', 'joint1/velocity', 'joint2/position', 'joint2/velocity']), 
    
    controller_manager_msgs.msg.ControllerState(
        name='forward_position_controller', state='active', type='forward_command_controller/ForwardCommandController', 
            claimed_interfaces=['joint1/position', 'joint2/position'], required_command_interfaces=['joint1/position', 'joint2/position'], 
            required_state_interfaces=[]), 
    
    controller_manager_msgs.msg.ControllerState(
        name='position_trajectory_controller', state='inactive', type='joint_trajectory_controller/JointTrajectoryController', 
        claimed_interfaces=[], required
from robot import Robot
from dynamixel_utils.dynamixel import Dynamixel

leader_dynamixel = Dynamixel.Config(baudrate=1_000_000, device_name='/dev/ttyACM0').instantiate()
follower_dynamixel = Dynamixel.Config(baudrate=1_000_000, device_name='/dev/ttyACM1').instantiate()
follower = Robot(follower_dynamixel, servo_ids=[1, 2, 3, 4, 5, 6])
leader = Robot(leader_dynamixel, servo_ids=[1, 2, 3, 4, 5, 6])

follower.set_position_offset()
leader.set_position_offset()
leader.set_trigger_torque()

while True:
    pos_rel_leader = leader.read_position_relative()
    # offset = follower.read_position_offset()
    # pos_follower = [pos_rel_leader[i] + offset[i] for i in range(6)]

    # for i in range(6):
    #     if pos_follower[i] > 4096:
    #         pos_follower[i] = pos_follower[i] % 4096

    follower.set_goal_pos_relative(pos_rel_leader)
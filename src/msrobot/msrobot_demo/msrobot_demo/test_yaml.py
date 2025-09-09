import yaml
from copy import deepcopy
config = {
            'follower': {
                'ids': [1,2,3,4,5,6],
                'homing_positions': [0, 0, 0, 0, 0, 0],
                'servo_limit': [[0,0], [0,0], [0,0], [0,0], [0,0], [0,0]],
                'port': '/dev/ttyACM1',
                'baudrate': 1_000_000
            },
            'leader': {
                'ids': [1,2,3,4,5,6],
                'homing_positions': [0, 0, 0, 0, 0, 0],
                'servo_limit': [[0,0], [0,0], [0,0], [0,0], [0,0], [0,0]],
                'port': '/dev/ttyACM0',
                'baudrate': 1_000_000
            }
        }


servo_pos_offset_limit = [[-1024,1014] for x in [1,2,3,4,5,6]]
config["leader"]["servo_limit"] = deepcopy(servo_pos_offset_limit)
config["follower"]["servo_limit"] = deepcopy(servo_pos_offset_limit[:])

with open("test.yml", 'w') as file:
    yaml.dump(config, file, default_flow_style=False)
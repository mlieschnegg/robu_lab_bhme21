import numpy as np
from dynamixel_utils.dynamixel import Dynamixel, OperatingMode, ReadAttribute
import time
from dynamixel_sdk import GroupSyncRead, GroupSyncWrite, DXL_LOBYTE, DXL_HIBYTE, DXL_LOWORD, DXL_HIWORD
from enum import Enum, auto
from typing import Union


class MotorControlType(Enum):
    PWM = auto()
    POSITION_CONTROL = auto()
    EXTENDED_POSITION_CONTROL = auto()
    DISABLED = auto()
    UNKNOWN = auto()


class Robot:
    # def __init__(self, device_name: str, baudrate=1_000_000, servo_ids=[1, 2, 3, 4, 5]):
    servo_pos_offset = []

    def __init__(self, dynamixel, baudrate=1_000_000, servo_ids=[1, 2, 3, 4, 5]):
        self.servo_ids = servo_ids
        self.dynamixel = dynamixel
        # self.dynamixel = Dynamixel.Config(baudrate=baudrate, device_name=device_name).instantiate()
        self.position_reader = GroupSyncRead(
            self.dynamixel.portHandler,
            self.dynamixel.packetHandler,
            ReadAttribute.POSITION.value,
            4)
        for id in self.servo_ids:
            self.position_reader.addParam(id)

        self.velocity_reader = GroupSyncRead(
            self.dynamixel.portHandler,
            self.dynamixel.packetHandler,
            ReadAttribute.VELOCITY.value,
            4)
        for id in self.servo_ids:
            self.velocity_reader.addParam(id)

        self.pos_writer = GroupSyncWrite(
            self.dynamixel.portHandler,
            self.dynamixel.packetHandler,
            self.dynamixel.ADDR_GOAL_POSITION,
            4)
        for id in self.servo_ids:
            self.pos_writer.addParam(id, [2048])

        self.pwm_writer = GroupSyncWrite(
            self.dynamixel.portHandler,
            self.dynamixel.packetHandler,
            self.dynamixel.ADDR_GOAL_PWM,
            2)
        for id in self.servo_ids:
            self.pwm_writer.addParam(id, [2048])
        self._disable_torque()
        self.motor_control_state = MotorControlType.DISABLED


        self.servo_pos_offset = [0 for x in self.servo_ids]
        self.servo_pos_offset_limit = [[0,0] for x in self.servo_ids]

    def set_position_offset(self):
        self.servo_pos_offset = self.read_position()

    def read_position_offset(self):
        return self.servo_pos_offset
    
    def read_position_relative(self):
        pos = self.read_position()
        return [pos[i] - self.servo_pos_offset[i] for i in range(len(pos))]
    
    def read_position_offset_limit(self):
        return self.servo_pos_offset_limit
    
    def set_position_offset_limit(self, limits):
        if len(limits) == len(self.servo_ids):
            self.servo_pos_offset_limit = limits

    def set_position_offset_limit_by_id(self, id, limit=(0,0)):
        self.servo_pos_offset_limit[id] = limit

    def read_position(self, tries=2):
        """
        Reads the joint positions of the robot. 2048 is the center position. 0 and 4096 are 180 degrees in each direction.
        :param tries: maximum number of tries to read the position
        :return: list of joint positions in range [0, 4096]
        """
        result = self.position_reader.txRxPacket()
        if result != 0:
            if tries > 0:
                return self.read_position(tries=tries - 1)
            else:
                print(f'failed to read position!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
                return []
        positions = []
        for id in self.servo_ids:
            position = self.position_reader.getData(id, ReadAttribute.POSITION.value, 4)
            if position > 2 ** 31:
                position -= 2 ** 32
            positions.append(position)
        return positions

    def read_velocity(self):
        """
        Reads the joint velocities of the robot.
        :return: list of joint velocities,
        """
        self.velocity_reader.txRxPacket()
        velocties = []
        for id in self.servo_ids:
            velocity = self.velocity_reader.getData(id, ReadAttribute.VELOCITY.value, 4)
            if velocity > 2 ** 31:
                velocity -= 2 ** 32
            velocties.append(velocity)
        return velocties


    def set_goal_pos_relative_by_id(self, motor_id, position):
        self.dynamixel._disable_torque(motor_id)
        self.dynamixel.set_operating_mode(motor_id, OperatingMode.EXTENDED_POSITION)
        self.dynamixel._enable_torque(motor_id)
        #for i, motor_id in enumerate(self.servo_ids):
        if position > 4096:
            position = position % 4096
        # data_write = [DXL_LOBYTE(DXL_LOWORD(position)),
        #                 DXL_HIBYTE(DXL_LOWORD(position)),
        #                 DXL_LOBYTE(DXL_HIWORD(position)),
        #                 DXL_HIBYTE(DXL_HIWORD(position))]
        self.dynamixel.set_goal_position(motor_id, position)

    def set_goal_pos(self, action):
        """

        :param action: list or numpy array of target joint positions in range [0, 4096]
        """
        if not self.motor_control_state is MotorControlType.POSITION_CONTROL:
            self._set_position_control()
        for i, motor_id in enumerate(self.servo_ids):
            if action[i] > 4096:
                action[i] = action[i] % 4096
            data_write = [DXL_LOBYTE(DXL_LOWORD(action[i])),
                          DXL_HIBYTE(DXL_LOWORD(action[i])),
                          DXL_LOBYTE(DXL_HIWORD(action[i])),
                          DXL_HIBYTE(DXL_HIWORD(action[i]))]
            self.pos_writer.changeParam(motor_id, data_write)

        self.pos_writer.txPacket()

    def set_goal_pos_relative(self, position_relative):
        """

        :param action: list or numpy array of target joint positions in range [0, 4096]
        """
        if len(position_relative) == len(self.servo_ids):
            if not self.motor_control_state is MotorControlType.EXTENDED_POSITION_CONTROL:
                self._set_extended_position_control()
            for i, motor_id in enumerate(self.servo_ids):
                # if action[i] > 4096:
                #     action[i] = action[i] % 4096
                
                #print('pos %04d %04d %04d %04d %04d %04d' % (action[0], action[1], action[2], action[3], action[4], action[5]))
                offset_min, offset_max = self.servo_pos_offset_limit[i]
                if (offset_min < offset_max):
                    if position_relative[i] > offset_max:
                        position_relative[i] = offset_max
                    elif position_relative[i] < offset_min:
                        position_relative[i] = offset_min

                data_write = [DXL_LOBYTE(DXL_LOWORD(position_relative[i] + self.servo_pos_offset[i])),
                            DXL_HIBYTE(DXL_LOWORD(position_relative[i] + self.servo_pos_offset[i])),
                            DXL_LOBYTE(DXL_HIWORD(position_relative[i] + self.servo_pos_offset[i])),
                            DXL_HIBYTE(DXL_HIWORD(position_relative[i] + self.servo_pos_offset[i]))]
                self.pos_writer.changeParam(motor_id, data_write)

            self.pos_writer.txPacket()

    def set_pwm(self, action):
        """
        Sets the pwm values for the servos.
        :param action: list or numpy array of pwm values in range [0, 885]
        """
        if not self.motor_control_state is MotorControlType.PWM:
            self._set_pwm_control()
        for i, motor_id in enumerate(self.servo_ids):
            data_write = [DXL_LOBYTE(DXL_LOWORD(action[i])),
                          DXL_HIBYTE(DXL_LOWORD(action[i])),
                          ]
            self.pwm_writer.changeParam(motor_id, data_write)

        self.pwm_writer.txPacket()

    def set_trigger_torque(self, pwm_value=200):
        """
        Sets a constant torque torque for the last servo in the chain. This is useful for the trigger of the leader arm
        """
        self.dynamixel._enable_torque(self.servo_ids[-1])
        self.dynamixel.set_pwm_value(self.servo_ids[-1], pwm_value)

    def limit_pwm(self, limit: Union[int, list, np.ndarray]):
        """
        Limits the pwm values for the servos in for position control
        @param limit: 0 ~ 885
        @return:
        """
        if isinstance(limit, int):
            limits = [limit, ] * 5
        else:
            limits = limit
        self._disable_torque()
        for motor_id, limit in zip(self.servo_ids, limits):
            self.dynamixel.set_pwm_limit(motor_id, limit)
        self._enable_torque()

    def disable_torque(self):
        self.motor_control_state = MotorControlType.DISABLED
        self._disable_torque()

    def _disable_torque(self):
        print(f'disabling torque for servos {self.servo_ids}')
        for motor_id in self.servo_ids:
            self.dynamixel._disable_torque(motor_id)

    def _enable_torque(self):
        print(f'enabling torque for servos {self.servo_ids}')
        for motor_id in self.servo_ids:
            self.dynamixel._enable_torque(motor_id)

    def _set_pwm_control(self):
        self._disable_torque()
        for motor_id in self.servo_ids:
            self.dynamixel.set_operating_mode(motor_id, OperatingMode.PWM)
        self._enable_torque()
        self.motor_control_state = MotorControlType.PWM

    def _set_position_control(self):
        self._disable_torque()
        for motor_id in self.servo_ids:
            self.dynamixel.set_operating_mode(motor_id, OperatingMode.POSITION)
        self._enable_torque()
        self.motor_control_state = MotorControlType.POSITION_CONTROL

    def _set_extended_position_control(self):
        self._disable_torque()
        for motor_id in self.servo_ids:
            self.dynamixel.set_operating_mode(motor_id, OperatingMode.EXTENDED_POSITION)
        self._enable_torque()
        self.motor_control_state = MotorControlType.EXTENDED_POSITION_CONTROL

if __name__ == "__main__":
    leader_dynamixel = Dynamixel.Config(baudrate=1_000_000, device_name='/dev/ttyACM0').instantiate()
    robot = Robot(leader_dynamixel, baudrate=1_000_000, servo_ids=[1, 2, 3, 4, 5, 6])
    #robot._set_position_control()
    robot._disable_torque()
    robot.set_position_offset()

    for _ in range(5000):
        s = time.time()
        pos = robot.read_position_relative()
        elapsed = time.time() - s
        print('pos %04d %04d %04d %04d %04d %04d' % (pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]))
        # print(f'read took {elapsed} pos {pos}')
    # robot._enable_torque()
    # robot.set_goal_pos(robot.read_position_offset())
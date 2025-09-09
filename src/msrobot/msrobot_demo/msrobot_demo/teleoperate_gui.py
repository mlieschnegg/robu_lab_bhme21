from tkinter import *
from tkinter import ttk, messagebox
import serial.tools.list_ports
import threading
import time
import yaml
import os
from copy import deepcopy
from robot import Robot, MotorControlType
from dynamixel_utils.dynamixel import Dynamixel

import signal
import sys



class MasterSlaveRobot():
    _flag_do_master_slave_operation = False
    _flag_initialized = False
    _flag_connected_leader = False
    _flag_connected_follower = False
    CONFIG_FILE_NAME = os.path.join(os.path.split(os.path.abspath(__file__))[0], 'msrobot.yml')
    config = {}

    def __init__(self):
        self.load_config()

    def disconnect_from_robots(self):
        if self._flag_connected_follower: self._disconnect_from_robot(self.follower.dynamixel.config.device_name)
        if self._flag_connected_leader: self._disconnect_from_robot(self.leader.dynamixel.config.device_name)

    def is_connected(self):
        return self._flag_connected_follower and self._flag_connected_leader
    
    def connect_to_robots(self, device_name_leader="", device_name_follower=""):
        if device_name_leader == "":
            device_name_leader=self.config['leader']['port']
        if device_name_follower == "":
            device_name_follower=self.config['follower']['port']

        self.leader = self._connect_to_robot(device_name_leader)
        self.follower = self._connect_to_robot(device_name_follower)

        if self.leader == None and self.follower != None:
            self._disconnect_from_robot(self.follower.dynamixel.config.device_name)
        elif self.leader != None and self.follower == None:
            self._disconnect_from_robot(self.leader.dynamixel.config.device_name)

        if self.leader != None:
            self.config['leader']['port'] = device_name_leader
            self.leader.set_position_offset_limit(self.config["leader"]["servo_limit"])
            self._flag_connected_leader = True
        if self.follower != None:
            self.config['leader']['port'] = device_name_leader
            self.follower.set_position_offset_limit(self.config["follower"]["servo_limit"])
            self._flag_connected_follower = True
    
    def get_serial_ports(self):
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports if port.device.startswith('/dev/ttyACM')]
    
    def load_config(self):
        try:
            with open(self.CONFIG_FILE_NAME, 'r') as file:
                config = yaml.safe_load(file)
            print("config loaded")
        except:
            print("standard config loaded")
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
        self.config = config
        if self.is_connected():
            self.leader.set_position_offset_limit(self.config["leader"]["servo_limit"])
            self.follower.set_position_offset_limit(self.config["follower"]["servo_limit"])

    def save_config(self):
        if self.is_connected():
            self.config['follower']['homing_positions'] = \
                deepcopy(self.follower.read_position_offset())
            self.config['leader']['homing_positions'] = \
                deepcopy(self.leader.read_position_offset())
            self.config['follower']['servo_limit']=\
                deepcopy(self.follower.read_position_offset_limit())
            self.config['leader']['servo_limit']=\
                deepcopy(self.leader.read_position_offset_limit())
        print("save config: ", self.config)
        with open(self.CONFIG_FILE_NAME, 'w') as file:
            yaml.dump(self.config, file, default_flow_style=False)

        print("config saved")

    def set_home(self):
        if self.is_connected():
            self.follower.set_position_offset()
            self.leader.set_position_offset()

            self.config['follower']['homing_positions'] = self.follower.read_position_offset()
            self.config['leader']['homing_positions'] = self.leader.read_position_offset()

            self.save_config()
            self._flag_initialized = True

    def go_home(self):
        if self._flag_do_master_slave_operation == False:
            if self.is_connected():
                self.follower.set_goal_pos(self.config['follower']['homing_positions'])
                self.leader.set_goal_pos(self.config['leader']['homing_positions'])

    def set_position_offset_limit(self, limits):
        if self.is_connected():
            self.follower.set_position_offset_limit(limits)
            self.leader.set_position_offset_limit(limits)

    def read_position_relative(self):
        if self.is_connected():
            return self.leader.read_position_relative()
        return None

    def initialize_master_slave_position(self):
        if self.is_connected() and self._flag_do_master_slave_operation == False:
            if len(set((self.config['follower']['homing_positions']))) > 1: #remove all duplicates
                self.follower.disable_torque()
                self.leader.disable_torque()
                
                for i, motor_id in enumerate(self.follower.servo_ids):
                    position = self.config['follower']['homing_positions'][i]
                    print("position: ", position )
                    self.follower.set_goal_pos_relative_by_id(motor_id, position)
                self.follower.motor_control_state = MotorControlType.EXTENDED_POSITION_CONTROL
                self._flag_initialized = True

    def is_busy(self):
        if self._flag_do_master_slave_operation:
            return True
        return False

    def start_master_slave_operation(self, blocking=False):
        if self.is_connected() and self._flag_initialized:
            if self._flag_do_master_slave_operation == False:
                if blocking == False:
                    self._start_master_slave_operation()
                else:
                    self._do_master_slave_operation()
            else:
                self._stop_master_slave_operation()

    def stop_master_slave_operation(self):
        if self.is_connected():
            self._stop_master_slave_operation()

    def _connect_to_robot(self, device_name):
        try:
            config = Dynamixel.Config(baudrate=1_000_000, device_name=device_name).instantiate()
            robot = Robot(config, servo_ids=[1, 2, 3, 4, 5, 6])
            print(f"Connected to {device_name}")
            return robot
        except:
            print(f"Connection to {device_name} FAILED!")
            return None
        
    def _disconnect_from_robot(self, device_name):
        if self._flag_connected_follower == True and self.follower.dynamixel.config.device_name==device_name:
            self.follower.dynamixel.disconnect()
            self.follower = None
            self._flag_connected_follower = False
        elif self._flag_connected_leader == True and self.leader.dynamixel.config.device_name==device_name:
            self.leader.dynamixel.disconnect()
            self.leader = None
            self._flag_connected_leader = False
        print(f"Disconnected from {device_name}")

    def _initialize_robots(self):
        if self.is_connected():
            self.follower.set_position_offset()
            self.leader.set_position_offset()
            self._flag_initialized = True

    def _stop_master_slave_operation(self):
        if self._flag_do_master_slave_operation:
            self._flag_do_master_slave_operation = False
            
            tries = 2
            while tries > 0:
                try:
                    self.leader.disable_torque()
                    self.follower.disable_torque()
                    tries = 0
                except:
                    tries -= 1
            time.sleep(0.2)

    def _start_master_slave_operation(self):
        if self._flag_do_master_slave_operation == False:
            thread = threading.Thread(target=self._do_master_slave_operation)
            thread.start()
    
    def _do_master_slave_operation(self):
        self.leader.set_trigger_torque()
        self._flag_do_master_slave_operation = True
        info_not_initialized = False
        while self._flag_do_master_slave_operation:
            if self._flag_initialized:
                info_not_initialized = True
                pos_rel_leader = self.leader.read_position_relative()
                self.follower.set_goal_pos_relative(pos_rel_leader)
            else:
                time.sleep(0.2)
                if info_not_initialized == False:
                    print("Not initialized")
                    info_not_initialized = True

        tries = 2
        while tries > 0:
            try:
                self.leader.set_trigger_torque(0)
                self.leader.disable_torque()
                self.follower.set_goal_pos_relative_by_id(self.follower.servo_ids[-1], 0)
                time.sleep(0.1)
                self.follower.disable_torque()
                tries = 0
            except:
                tries -= 1

    def __del__(self):
        print("Destructor: MasterSlaveRobot")
        self.stop_master_slave_operation()
        self.disconnect_from_robots()


class MasterSlaveRobotGUI(Tk):
    
    #_msrobot: MasterSlaveRobot

    def __init__(self):
        super().__init__()
        self.geometry("800x480")
        #root.attributes('-fullscreen', True)
        self.title("HTBLA-Kaindorf")

        self._msrobot = MasterSlaveRobot()

        self._pack_widgets()
        self._fill_widgets()

        self._btn_on_connect_disconnect()
        self._btn_on_set_home()
        self._btn_on_start_master_slave_operation()
    
    def _pack_widgets(self):
        # Grid festlegen
        Grid.rowconfigure(self,0,weight=1)
        Grid.columnconfigure(self,0,weight=1)
        #Grid.columnconfigure(self,1,weight=1)

        Grid.rowconfigure(self,1,weight=1)
        Grid.rowconfigure(self,2,weight=1)
        Grid.rowconfigure(self,3,weight=1)
        Grid.rowconfigure(self,4,weight=1)
        Grid.rowconfigure(self,5,weight=1)

        frm_serial = LabelFrame(self, text="Serial Configuration")
        Grid.rowconfigure(frm_serial,0,weight=1)
        Grid.rowconfigure(frm_serial,1,weight=1)
        Grid.columnconfigure(frm_serial,0,weight=0)
        Grid.columnconfigure(frm_serial,1,weight=1)

        lbl = ttk.Label(frm_serial, text="Serial port of the leader:")
        lbl.grid(row=0, column=0, sticky="NSEW")

        serial_ports = self.get_serial_ports()
        self._cbx_leader = ttk.Combobox(frm_serial, values=serial_ports)
        self._cbx_leader.grid(row=0, column=1, sticky="NSEW")
        if len(serial_ports):
            self._cbx_leader.set(serial_ports[0])

        lbl = ttk.Label(frm_serial, text="Serial port of the follower:")
        lbl.grid(row=1, column=0, sticky="NSEW")

        serial_ports = self.get_serial_ports()
        self._cbx_follower = ttk.Combobox(frm_serial, values=serial_ports)
        self._cbx_follower.grid(row=1, column=1, sticky="NSEW")
        if len(serial_ports):
            self._cbx_follower.set(serial_ports[0])
        
        frm_serial.grid(row=0, sticky="NSEW")

        self._btn_connect_disconnect = Button(self, text="Connect", command=self._btn_on_connect_disconnect)
        self._btn_connect_disconnect.grid(row=1, column=0, sticky="NSEW")
        
        self._btn_initialize = Button(self, text="Initialize", command=self._btn_on_initialize)
        self._btn_initialize.grid(row=2, column=0, sticky="NSEW")

        frm_home = LabelFrame(self, text="Home Position")
        Grid.rowconfigure(frm_home,0,weight=1)
        Grid.columnconfigure(frm_home,0,weight=1)
        Grid.columnconfigure(frm_home,1,weight=1)
        Grid.columnconfigure(frm_home,2,weight=1)
        self._btn_set_home = Button(frm_home, text="Set", command=self._btn_on_set_home)
        self._btn_set_home.grid(row=0, column=0, sticky="NSEW")
        self._btn_set_base_limits = Button(frm_home, text="Set base limits", command=self._btn_on_set_base_limits)
        self._btn_set_base_limits.grid(row=0, column=1, sticky="NSEW")
        self._btn_set_gripper_limits = Button(frm_home, text="Set gripper limits", command=self._btn_on_set_gripper_limits)
        self._btn_set_gripper_limits.grid(row=0, column=2, sticky="NSEW")
        frm_home.grid(row=3, sticky="NSEW")

        self._btn_master_slave_operation = Button(self, text="Start", command=self._btn_on_start_master_slave_operation)
        self._btn_master_slave_operation.grid(row=4, column=0, sticky="NSEW")

        btn_quit = Button(self, text="Quit", command=self._btn_on_close)
        btn_quit.grid(row=5, column=0, sticky="NSEW")

    def _fill_widgets(self):
        try:
            self._cbx_follower.set(self._msrobot.config['follower']['port'])
        except:
            pass

        try:
            self._cbx_leader.set(self._msrobot.config['leader']['port'])
        except:
            pass

    def get_serial_ports(self):
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports if port.device.startswith('/dev/ttyACM')]

    def _btn_on_connect_disconnect(self):
        if self._msrobot.is_connected():
            self._msrobot.disconnect_from_robots()
        else:
            self._msrobot.connect_to_robots(self._cbx_leader.get(), self._cbx_follower.get())
        
        if self._msrobot.is_connected():
            self._btn_connect_disconnect.config(text="Disconnect")
        else:
            self._btn_connect_disconnect.config(text="Connect")

    def _btn_on_initialize(self):
        self._btn_on_set_home()
    
    def _btn_on_set_home(self):
        self._msrobot.set_home()

    def _set_servo_limit(self, id):
        messagebox.showinfo("Min-Limit", f"Drehen Sie den Motor {id} zum Min-Limit")
        min_limit = self._msrobot.read_position_relative()[id-1]

        messagebox.showinfo("Max-Limit", f"Drehen Sie den Motor {id} zum Max-Limit")
        max_limit = self._msrobot.read_position_relative()[id-1]

        limits = self._msrobot.config['leader']['servo_limit']
        if (max_limit > min_limit):
            limits[id-1] = [min_limit, max_limit]
        else:
            limits[id-1] = [max_limit, min_limit]

        self._msrobot.config['leader']['servo_limit'] = deepcopy(limits)
        self._msrobot.config['follower']['servo_limit'] = deepcopy(limits)
        self._msrobot.save_config()
        print(f"servo leader {id} limits: {self._msrobot.config['leader']['servo_limit'][id-1]}")
        print(f"servo follower {id} limits: {self._msrobot.config['follower']['servo_limit'][id-1]}")
        self._msrobot.set_position_offset_limit(limits)

    def _btn_on_set_base_limits(self):
        self._set_servo_limit(1)

    def _btn_on_set_gripper_limits(self):
        self._set_servo_limit(5)
    
    def _btn_on_start_master_slave_operation(self):
        if not self._msrobot.is_busy():
            self._msrobot.start_master_slave_operation(blocking=False)
        else:
            self._msrobot.stop_master_slave_operation()

        time.sleep(1.0)
        if self._msrobot.is_busy():
            self._btn_master_slave_operation.config(text="Stop")
        else:
            self._btn_master_slave_operation.config(text="Start")

    def _btn_on_close(self):
        self._msrobot._stop_master_slave_operation()
        self._msrobot.save_config()
        self.quit()

if __name__ == '__main__':
    def main_with_gui():
        global msrobot
        def signal_handler(sig, frame):
            global msrobot
            print('Sie haben Ctrl+C gedrückt!')
            msrobot.quit()

        signal.signal(signal.SIGINT, signal_handler)

        print("Master-Slave-Robot - GUI")
        msrobot = MasterSlaveRobotGUI()
        msrobot.mainloop()

    def main():
        global msrobot
        def signal_handler(sig, frame):
            global msrobot
            print('Sie haben Ctrl+C gedrückt!')
            msrobot.stop_master_slave_operation()

        signal.signal(signal.SIGINT, signal_handler)

        print("Master-Slave-Robot")
        msrobot = MasterSlaveRobot()
        #print(msrobot.get_serial_ports())
        msrobot.connect_to_robots()
        #msrobot.initialize_master_slave_position()
        msrobot.set_home()
        msrobot.start_master_slave_operation(blocking=True)

    if len(sys.argv) > 1 and sys.argv[1] == '-gui':
        main_with_gui()
    else:
        main()
    

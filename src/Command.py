import numpy as np
from Config import Configuration

class Command:
    """Stores movement command
    """

    def __init__(self):
        self.horizontal_velocity = np.array([0, 0])
        self.yaw_rate = 0.0
        _config = Configuration()
        self.height = _config.default_z_ref
        self.pitch = 0.0
        self.roll = 0.0
        self.activation = 0
        self.joy_ps4_usb = True

        self.hop_event = False
        self.trot_event = False
        self.activate_event = False
        self.caliblate_mode_event = False

        self.arm_angles = np.zeros(6)
        # L1/L2で足の前後位置をずらす用（前後方向）
        self.foot_shift_x = 0.0

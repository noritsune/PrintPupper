import numpy as np


class Command:
    """Stores movement command
    """

    def __init__(self):
        self.horizontal_velocity = np.array([0, 0])
        self.yaw_rate = 0.0
        self.height = -0.15
        self.pitch = 0.0
        self.roll = 0.0
        self.activation = 0
        self.joy_ps4_usb = True

        self.hop_event = False
        self.trot_event = False
        self.activate_event = False
        self.caliblate_mode_event = False

        self.leg_pos_offsets = np.zeros((3, 4))
        self.is_legs_offset_mode = False

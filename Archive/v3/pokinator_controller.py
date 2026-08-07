import os
os.environ["BLINKA_FT232H"] = "1"

import time
import board
from pokinator_core import PokinatorCore

# Import Phidget Libraries
from Phidget22.Phidget import *
from Phidget22.Devices.VoltageRatioInput import *

class PokinatorController(PokinatorCore):
    """High-level logic layer for movement and force sensor integration"""
    
    def __init__(self, dir_pin_id, lc_channel=0, lc_offset=0.0, lc_gain=1.0):
        super().__init__(
            spi_clock=board.SCK, 
            spi_mosi=board.MOSI, 
            spi_miso=board.MISO, 
            dir_pin_id=dir_pin_id
        )
        
        # Load Cell Parameters
        self.lc_offset = lc_offset
        self.lc_gain = lc_gain
        
        print("Initializing Phidget Load Cell...")
        self.lc = VoltageRatioInput()
        self.lc.setChannel(lc_channel)
        self.lc.openWaitForAttachment(Phidget.DEFAULT_TIMEOUT)
        self.lc.setBridgeEnabled(True)
        self.lc.setBridgeGain(BridgeGain.BRIDGE_GAIN_128)
        
        # Hardware calibration delay required by the Phidget API
        time.sleep(3) 
        print("Load Cell Ready.")

    def get_force_grams(self):
        """Calculates current force based on V/V, offset, and gain"""
        vv = self.lc.getVoltageRatio()
        return (vv - self.lc_offset) * self.lc_gain

    def move_axis_distance(self, axis_name, distance_mm, target_rpm=60, max_force_g=None):
        """Moves a specific axis, aborts instantly if max_force_g is exceeded"""
        if distance_mm == 0 or axis_name not in self.axes: 
            return self.get_force_grams()

        axis = self.axes[axis_name]

        # 1. Determine Direction (But DO NOT set it yet!)
        new_direction = False if distance_mm > 0 else True

        total_steps = int(abs(distance_mm) * axis['steps_per_mm'])
        
        # Create the dynamic stop condition
        if max_force_g is not None:
            stop_cond = lambda: abs(self.get_force_grams()) >= max_force_g
        else:
            stop_cond = None
        
        # 2. Pass the new_direction down into the burst function
        bytes_sent = self._execute_burst(axis_name, total_steps, target_rpm, new_direction, stop_condition=stop_cond)
        
        # Update Position Tracking for this specific axis
        actual_distance = (bytes_sent * 8) / axis['steps_per_mm']
        if distance_mm < 0:
            axis['current_pos'] -= actual_distance
        else:
            axis['current_pos'] += actual_distance
            
        return self.get_force_grams()

# --- Execution Block ---
if __name__ == "__main__":
    # AC1 is our Global Direction Pin -> board.C1
    pokinator = PokinatorController(dir_pin_id=board.C1)
    
    # Register axes and their Selector pins (AC2 -> board.C2)
    pokinator.add_axis("Y1", select_pin_id=board.C2)
    
    # Example of how easily you can add the future Y1/Y2 and X axes:
    # pokinator.add_axis("Y1_AXIS", select_pin_id=board.C3)
    # pokinator.add_axis("Y2_AXIS", select_pin_id=board.C4)
    # pokinator.add_axis("X_AXIS", select_pin_id=board.C5)
    
    try:
        while True:
            val = input("\nEnter distance for Y1, mm (+Away, -Towards) or 'q': ")
            if val.lower() == 'q': break
            
            pokinator.move_axis_distance("Y1", float(val), target_rpm=240)
            print(f"Current Position (Y1): {pokinator.axes['Y1']['current_pos']:.2f} mm")
            
    finally:
        pokinator.shutdown()
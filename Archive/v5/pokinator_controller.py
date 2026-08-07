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
        
        # Hardware calibration delay
        time.sleep(3)
        print("Load Cell Ready.")

    def get_force_grams(self):
        vv = self.lc.getVoltageRatio()
        return (vv - self.lc_offset) * self.lc_gain

    def move_axis_distance(self, axis_name, distance_mm, target_rpm=60, max_force_g=None):
        if distance_mm == 0: return self.get_force_grams()
        
        direction_state = False if distance_mm > 0 else True
        total_steps = int(abs(distance_mm) * self.steps_per_mm)
        
        # Force Condition (Safe to poll because it uses Phidget USB, not FT232H)
        if max_force_g is not None:
            stop_cond = lambda: abs(self.get_force_grams()) >= max_force_g
        else:
            stop_cond = None
            
        bytes_sent = self._execute_burst(axis_name, total_steps, target_rpm, direction_state, stop_condition=stop_cond)
        
        # Update software position 
        actual_distance = (bytes_sent * 8) / self.steps_per_mm
        if distance_mm < 0:
            self.axes[axis_name]['current_pos'] -= actual_distance
        else:
            self.axes[axis_name]['current_pos'] += actual_distance
            
        return self.get_force_grams()

    def shutdown(self):
        self.lc.close()
        super().shutdown()

# --- Execution Block ---

if __name__ == "__main__":
    # Initialize Core (Global DIR is C1)
    pokinator = PokinatorController(dir_pin_id=board.C1)
    
    # Register all 3 axes to the multiplexer and assign their limit switches
    pokinator.add_axis("Y", select_pin_id=board.C2, home_pin_id=board.C5, home_trigger=False)
    pokinator.add_axis("X", select_pin_id=board.C3, home_pin_id=board.C6, home_trigger=False)
    pokinator.add_axis("Z", select_pin_id=board.C4, home_pin_id=board.C7, home_trigger=False)

    try:
        active_axis = None
        
        while True:
            # Step 1: Axis Selection
            if not active_axis:
                axis_input = input("\nEnter axis to control ('X', 'Y', 'Z') or 'q' to quit: ").upper()
                
                if axis_input == 'Q': 
                    break
                    
                if axis_input not in pokinator.axes:
                    print("Invalid axis. Please enter X, Y, or Z.")
                    continue
                    
                active_axis = axis_input

            # Step 2: Axis Command
            val = input(f"\n[{active_axis} AXIS] Enter 'h' to Home, distance in mm, 'b' to change axis, or 'q' to quit: ")
            
            if val.lower() == 'q': 
                break
                
            if val.lower() == 'b':
                active_axis = None # Clear the active axis to return to the selection menu
                continue
                
            if val.lower() == 'h':
                # Utilizing the smooth 45 RPM macro-burst strategy for homing
                pokinator.home_axis(active_axis, rpm=45)
                continue
                
            try:
                distance = float(val)
                pokinator.move_axis_distance(active_axis, distance, target_rpm=240)
                print(f"Current Position ({active_axis}): {pokinator.axes[active_axis]['current_pos']:.2f} mm")
            except ValueError:
                print("Invalid input. Please enter 'h', 'b', 'q', or a valid number.")
                
    finally:
        # Safely closes all 3 valves, releases SPI, and drops the DIR pin
        pokinator.shutdown()
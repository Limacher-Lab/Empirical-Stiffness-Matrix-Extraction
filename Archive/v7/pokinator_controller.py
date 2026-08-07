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
        
        self.lc_offset = lc_offset
        self.lc_gain = lc_gain
        self.live_force_g = 0.0  # Ultra-fast memory variable for the loop
        
        print("Initializing Phidget Load Cell...")
        self.lc = VoltageRatioInput()
        self.lc.setChannel(lc_channel)
        
        # --- FIX 2: ZERO-LATENCY EVENT HANDLER ---
        # This background thread updates the force variable instantly on hardware change
        def on_force_change(ch, voltageRatio):
            self.live_force_g = (voltageRatio - self.lc_offset) * self.lc_gain
            
        # Must be attached BEFORE opening
        self.lc.setOnVoltageRatioChangeHandler(on_force_change)
        self.lc.openWaitForAttachment(Phidget.DEFAULT_TIMEOUT)
        
        self.lc.setBridgeEnabled(True)
        self.lc.setBridgeGain(BridgeGain.BRIDGE_GAIN_128)
        
        # --- FIX 1: MAXIMIZE HARDWARE SPEED ---
        # Forces the Phidget to blast updates every ~8ms instead of ~250ms
        self.lc.setDataInterval(self.lc.getMinDataInterval())
        
        time.sleep(3)
        print("Load Cell Ready.")

    def get_force_grams(self):
        # Now returns the memory variable instantly instead of waiting for a USB response!
        return self.live_force_g

    def move_axis_distance(self, axis_name, distance_mm, target_rpm=60, max_force_g=None):
        if distance_mm == 0: return self.get_force_grams()
        
        direction_state = False if distance_mm > 0 else True
        total_steps = int(abs(distance_mm) * self.steps_per_mm)
        
        trigger_memory = {"peak_force": None}
        
        # Force Condition 
        if max_force_g is not None:
            def stop_cond():
                current_force = self.get_force_grams()
                if abs(current_force) >= max_force_g:
                    trigger_memory["peak_force"] = current_force # Freeze the value!
                    return True
                return False
        else:
            stop_cond = None
            
        bytes_sent = self._execute_burst(axis_name, total_steps, target_rpm, direction_state, stop_condition=stop_cond)
        
        # Update software position 
        actual_distance = (bytes_sent * 8) / self.steps_per_mm
        if distance_mm < 0:
            self.axes[axis_name]['current_pos'] -= actual_distance
        else:
            self.axes[axis_name]['current_pos'] += actual_distance
            
        # --- Return the frozen impact force if we hit the limit ---
        if trigger_memory["peak_force"] is not None:
            return trigger_memory["peak_force"]
            
        # If we didn't hit the limit (e.g. reached max distance), return live force
        return self.get_force_grams()

    def tare_load_cell(self, samples=20, delay=0.01):
        """Zeroes the load cell to account for resting weight and structural torque."""
        print("[*] Taring load cell to 0g... ", end="", flush=True)
        total_ratio = 0.0
        
        for _ in range(samples):
            # Using self.lc to match your Controller's specific Phidget variable
            total_ratio += self.lc.getVoltageRatio()
            time.sleep(delay)
            
        # Overwrite the hardcoded offset with the live resting offset
        self.lc_offset = total_ratio / samples
        print(f"Done. (New Offset: {self.lc_offset:.8e})")
        
        return self.lc_offset

    def probe_z_axis(self, max_distance_mm, target_force_g=10.0, speed_rpm=24):
        """
        Slowly moves the Z-axis to probe the sample until the target force is reached.
        """
        if "Z" not in self.axes:
            print("[!] Z-Axis is not registered. Cannot probe.")
            return None

        # 1. Tare the scale to 0 grams before doing anything
        self.tare_load_cell()
        
        print(f"\nProbing Z-Axis at {speed_rpm} RPM for a maximum of {max_distance_mm}mm...")
        print(f"Will halt instantly if {target_force_g}g of force is detected.")
        
        # 2. Execute the move using the controller's overridden move_axis_distance
        final_force = self.move_axis_distance(
            axis_name="Z", 
            distance_mm=max_distance_mm, 
            target_rpm=speed_rpm, 
            max_force_g=target_force_g
        )
        
        print("\n=== PROBE COMPLETE ===")
        print(f"Final Z Position: {self.axes['Z']['current_pos']:.2f} mm")
        print(f"Final Force Detected: {final_force:.2f} g")
        
        return final_force
    
    def shutdown(self):
        self.lc.close()
        super().shutdown()

# --- Execution Block ---

if __name__ == "__main__":
    import board
    
    # --- INPUT YOUR PHIDGET PARAMETERS HERE ---
    PHIDGET_OFFSET = -6.1872E-004
    PHIDGET_GAIN = 762172.29269146 

    # Initialize Core (Global DIR is C1) with load cell parameters
    pokinator = PokinatorController(
        dir_pin_id=board.C1,
        lc_offset=PHIDGET_OFFSET, 
        lc_gain=PHIDGET_GAIN
    )
    
    # Register all 3 axes to the multiplexer and assign their limit switches as homing switches
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
            val = input(f"\n[{active_axis} AXIS] Enter 'h' to Home, distance in mm, 'p' to Probe (Z only), 'b' to change axis, or 'q' to quit: ")
            
            if val.lower() == 'q': 
                break
                
            if val.lower() == 'b':
                active_axis = None # Clear the active axis to return to the selection menu
                continue
                
            if val.lower() == 'h':
                # Utilizing the smooth 45 RPM macro-burst strategy for homing
                pokinator.home_axis(active_axis, rpm=45)
                continue
                
            if val.lower() == 'p':
                if active_axis == 'Z':
                    dist_val = input("Enter max distance to travel (mm): ")
                    try:
                        max_dist = float(dist_val)
                        # The probe_z_axis method encapsulates the slow RPM and force check
                        pokinator.probe_z_axis(
                            max_distance_mm=max_dist, 
                            target_force_g=400.0, 
                            speed_rpm=12
                        )
                    except ValueError:
                        print("Invalid distance input.")
                else:
                    print("[!] Probing is physically restricted to the Z-axis.")
                continue
                
            try:
                distance = float(val)
                pokinator.move_axis_distance(active_axis, distance, target_rpm=240)
                print(f"Current Position ({active_axis}): {pokinator.axes[active_axis]['current_pos']:.2f} mm")
            except ValueError:
                print("Invalid input. Please enter 'h', 'p', 'b', 'q', or a valid number.")
                
    finally:
        # Safely closes all 3 valves, releases SPI, and drops the DIR pin
        pokinator.shutdown()
import os
os.environ["BLINKA_FT232H"] = "1"
import time
import board
from simple_pid import PID
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

    # def probe_z_axis(self, max_distance_mm, target_force_g=100.0, speed_rpm=24, hold_time_s=5.0, deadband_pct=0.02, debounce_time_s=0.05):
    #     """
    #     Actively probes and maintains force on the Z-axis using a PI control loop.
    #     Features a force deadband to prevent hunting on extremely stiff materials.
    #     Includes a time-based signal debounce to filter out inertial motor jolts.
    #     Records and returns force data at 10Hz during the hold phase.
    #     Triggers the DIC camera system upon reaching the target force.
    #     """
    #     if "Z" not in self.axes:
    #         print("[!] Z-Axis is not registered. Cannot probe.")
    #         return 0.0, []

    #     self.tare_load_cell()
        
    #     # Calculate the acceptable +/- force window
    #     tolerance_g = target_force_g * deadband_pct
    #     trigger_threshold = target_force_g - tolerance_g
        
    #     print(f"\n[*] Probing Z-Axis at max {speed_rpm} RPM for a max travel of {max_distance_mm} mm...")
    #     print(f"[*] Target Force: {target_force_g}g | Active Hold Time: {hold_time_s}s")
    #     print(f"[*] Deadband Active: +/- {tolerance_g:.1f}g ({deadband_pct*100}%)")
    #     print(f"[*] Debounce Filter Active: {debounce_time_s*1000} ms continuous impact required.")

    #     pid = PID(Kp=0.01, Ki=0.0015, Kd=0.0, setpoint=target_force_g)
    #     pid.output_limits = (-0.5, 0.5) 
        
    #     start_z = self.axes['Z']['current_pos']
    #     absolute_z_limit = start_z + abs(max_distance_mm) 

    #     force_achieved = False
    #     debounce_start_time = None  # New variable to track our time-verification
    #     hold_start_time = None
    #     next_sample_time = None
    #     recorded_forces = [] 
        
    #     try:
    #         while True:
    #             current_force = self.get_force_grams()
    #             current_time = time.time()
                
    #             # --- Debounce Trigger Logic ---
    #             if not force_achieved:
    #                 if current_force >= trigger_threshold:
    #                     # The force has spiked above our target. Start the stopwatch if it isn't running.
    #                     if debounce_start_time is None:
    #                         debounce_start_time = current_time
    #                     # Check if the force has stayed high for the required debounce duration
    #                     elif (current_time - debounce_start_time) >= debounce_time_s:
    #                         print(f"\n[+] Target force verified (held for {debounce_time_s}s). Initiating PI hold for {hold_time_s} seconds.")
                            
    #                         # Fire 1ms TTL Trigger to external Camera System
    #                         print("[*] Firing 1ms TTL Trigger to external Camera System...")
    #                         self.trigger_camera()
                            
    #                         force_achieved = True
    #                         hold_start_time = current_time
    #                         next_sample_time = current_time
    #                 else:
    #                     # If the force drops below the threshold, it was a transient jolt. Reset the stopwatch.
    #                     if debounce_start_time is not None:
    #                         debounce_start_time = None
    #             # ----------------------------------
                
    #             # If we are in the hold phase, handle timing and 10Hz sampling
    #             if force_achieved:
    #                 elapsed_hold = current_time - hold_start_time
    #                 if elapsed_hold >= hold_time_s:
    #                     print(f"\n[+] Active hold time complete ({hold_time_s}s).")
    #                     break
                        
    #                 # Exactly 10Hz (100ms) sampling with Time Debt Catch-up
    #                 while current_time >= next_sample_time:
    #                     recorded_forces.append(current_force)
    #                     next_sample_time += 0.1 
                
    #             # Calculate PI adjustment
    #             delta_mm = pid(current_force)
                
    #             # Force Deadband Logic
    #             if force_achieved and abs(target_force_g - current_force) <= tolerance_g:
    #                 delta_mm = 0.0
                
    #             # Enforce hardware travel limits
    #             current_z = self.axes['Z']['current_pos']
    #             if (current_z + delta_mm) > absolute_z_limit:
    #                 if not force_achieved:
    #                     print("\n[!] Max probe distance reached BEFORE target force was met.")
    #                     print("[!] Increase 'max_dist' in your CSV if you want to push harder.")
    #                     break
    #                 else:
    #                     delta_mm = 0.0
                    
    #             # Execute the micro-move (using the 3200 microstep limit: 0.003mm)
    #             if abs(delta_mm) > 0.003: 
    #                 self.move_axis_distance(
    #                     axis_name="Z", 
    #                     distance_mm=delta_mm, 
    #                     target_rpm=speed_rpm
    #                 )
    #             else:
    #                 # High-frequency sleep to keep the PI loop responsive
    #                 time.sleep(0.01)
                    
    #     except KeyboardInterrupt:
    #         print("\n[!] Emergency Stop triggered during active Probing!")

    #     final_force = self.get_force_grams()
    #     print("\n=== PROBE COMPLETE ===")
    #     print(f"Final Z Position: {self.axes['Z']['current_pos']:.2f} mm")
    #     print(f"Final Force Detected: {final_force:.2f} g")
        
    #     return final_force, recorded_forces
    
    def probe_z_axis_limited(self, max_distance_mm, target_force_g=100.0, speed_rpm=24, hold_time_s=60.0, deadband_pct=0.05, debounce_time_s=0.05):
        """
        Actively probes and maintains force on the Z-axis using a PI control loop.
        Features a 5-second sliding window to detect true mechanical equilibrium.
        Fires TTL trigger only when the force stabilizes (<5% change over 5s).
        The hold_time_s parameter acts as an absolute maximum timeout for the sequence.
        """
        if "Z" not in self.axes:
            print("[!] Z-Axis is not registered. Cannot probe.")
            return 0.0, []

        self.tare_load_cell()
        
        # Calculate acceptable +/- force windows
        tolerance_g = target_force_g * deadband_pct
        trigger_threshold = target_force_g - tolerance_g
        stability_tolerance_g = target_force_g * deadband_pct # 10% limit for equilibrium
        
        print(f"\n[*] Probing Z-Axis LTD at max {speed_rpm} RPM for a max travel of {max_distance_mm} mm...")
        print(f"[*] Target Force: {target_force_g}g | Max Allowed Time: {hold_time_s}s")
        print(f"[*] Deadband Active: +/- {tolerance_g:.1f}g ({deadband_pct*100}%)")
        print(f"[*] Stability Requirement: < 5% change over 5 seconds.")

        pid = PID(Kp=0.001, Ki=0.0015, Kd=0.0, setpoint=target_force_g)
        pid.output_limits = (-0.1, 0.1) 
        
        start_z = self.axes['Z']['current_pos']
        absolute_z_limit = start_z + abs(max_distance_mm) 

        force_achieved = False
        is_stable = False
        debounce_start_time = None
        final_hold_start_time = None
        next_sample_time = None
        recorded_forces = [] 
        
        force_window = [] # Sliding window memory array
        absolute_start_time = time.time()
        
        try:
            while True:
                current_time = time.time()
                current_force = self.get_force_grams()
                
                # --- Timeout Logic ---
                if current_time - absolute_start_time > hold_time_s:
                    print(f"\n[!] Timeout: Maximum allowed probing sequence time ({hold_time_s}s) reached.")
                    break
                    
                # --- Sliding Window Memory ---
                force_window.append((current_time, current_force))
                # Prune data older than 5 seconds from the window
                while force_window and (current_time - force_window[0][0]) > 5.0:
                    force_window.pop(0)
                
                # --- Plunge -> Hold Transition ---
                if not force_achieved:
                    if current_force >= trigger_threshold:
                        # Force spiked. Start the stopwatch if it isn't running.
                        if debounce_start_time is None:
                            debounce_start_time = current_time
                        
                        # Check if the force has stayed high for the required debounce duration
                        elif (current_time - debounce_start_time) >= debounce_time_s:
                            print(f"\n[+] Target threshold crossed and debounced. Monitoring for static equilibrium...")
                            force_achieved = True
                            next_sample_time = current_time
                    else:
                        # If the force drops below the threshold, it was a transient jolt. Reset.
                        if debounce_start_time is not None:
                            debounce_start_time = None
                
                # --- The Hold & Stability Phase ---
                if force_achieved:
                    # Exactly 10Hz (100ms) sampling with Time Debt Catch-up
                    while current_time >= next_sample_time:
                        recorded_forces.append(current_force)
                        next_sample_time += 0.1 
                        
                    if not is_stable:
                        # Ensure we actually have ~5 seconds of data built up before analyzing
                        if len(force_window) > 1 and (force_window[-1][0] - force_window[0][0]) >= 4.9:
                            window_forces = [f[1] for f in force_window]
                            max_f = max(window_forces)
                            min_f = min(window_forces)
                            
                            # Calculate the change percentage across the 5s window
                            change_pct = (max_f - min_f) / target_force_g
                            # Check if the absolute current force is within 20% of the target
                            within_target = abs(current_force - target_force_g) <= stability_tolerance_g
                            
                            if change_pct <= 0.05 and within_target:
                                print(f"\n[+] Force stabilized! (<5% fluctuation over 5s, within {deadband_pct*100}% of target).")
                                print("[*] Firing 1ms TTL Trigger to external Camera System...")
                                self.trigger_camera()
                                is_stable = True
                                final_hold_start_time = current_time
                    else:
                        # We are currently in the final 2-second logging hold after triggering
                        if current_time - final_hold_start_time >= 2.0:
                            print(f"\n[+] Final 2s data hold complete. Probing sequence finished.")
                            break

                # --- Execute PI Adjustments ---
                delta_mm = pid(current_force)
                
                # Force Deadband Logic
                if force_achieved and abs(target_force_g - current_force) <= tolerance_g:
                    delta_mm = 0.0
                
                # Enforce hardware travel limits
                current_z = self.axes['Z']['current_pos']
                if (current_z + delta_mm) > absolute_z_limit:
                    if not force_achieved:
                        print("\n[!] Max probe distance reached BEFORE target force was met.")
                        print("[!] Increase 'max_dist' in your CSV if you want to push harder.")
                        break
                    else:
                        delta_mm = 0.0
                    
                # Execute the micro-move (using the 3200 microstep limit: 0.003mm)
                if abs(delta_mm) > 0.05: 
                    self.move_axis_distance(
                        axis_name="Z", 
                        distance_mm=delta_mm, 
                        target_rpm=speed_rpm
                    )
                else:
                    # High-frequency sleep to keep the PI loop responsive
                    time.sleep(0.01)
                    
        except KeyboardInterrupt:
            print("\n[!] Emergency Stop triggered during active Probing!")

        final_force = self.get_force_grams()
        print("\n=== PROBE COMPLETE ===")
        print(f"Final Z Position: {self.axes['Z']['current_pos']:.2f} mm")
        print(f"Final Force Detected: {final_force:.2f} g")
        
        return final_force, recorded_forces

    def shutdown(self):
        self.lc.close()
        super().shutdown()

# --- Execution Block ---
if __name__ == "__main__":
    
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
            val = input(f"\n[{active_axis} AXIS] Enter 'h' to Home, distance in mm, 'p' to Probe (Z only), 't' to Trigger Camera, 'b' to change axis, or 'q' to quit: ")
            
            if val.lower() == 'q': 
                break
                
            if val.lower() == 'b':
                active_axis = None # Clear the active axis to return to the selection menu
                continue

            if val.lower() == 't':
                print("[*] Firing 1ms TTL pulse to camera...")
                pokinator.trigger_camera()
                print("[+] Snap! Camera triggered.")
                continue
                
            if val.lower() == 'h':
                # Utilizing the smooth 45 RPM macro-burst strategy for homing
                pokinator.home_axis(active_axis, rpm=12)
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
                pokinator.move_axis_distance(active_axis, distance, target_rpm=120)
                print(f"Current Position ({active_axis}): {pokinator.axes[active_axis]['current_pos']:.2f} mm")
            except ValueError:
                print("Invalid input. Please enter 'h', 'p', 't', 'b', 'q', or a valid number.")

    except KeyboardInterrupt:
        print("\n[!] Program interrupted by user.")
    finally:
        print("[*] Shutting down Pokinator hardware safely...")
        pokinator.shutdown()
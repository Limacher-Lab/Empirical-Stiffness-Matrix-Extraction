import os
# THIS MUST COME BEFORE IMPORTING BOARD
os.environ["BLINKA_FT232H"] = "1"

import csv
import time
import board

# Import your controller architecture
from pokinator_controller import PokinatorController

class PokinatorProgrammer:
    """Highest-level abstraction layer: parses and executes CSV instruction files."""
    
    def __init__(self, controller_instance):
        self.pokinator = controller_instance
        
        # --- AXIS MACHINE LIMITS (Absolute Bounds) ---
        self.x_lim = 460.00  # mm
        self.y_lim = 505.00  # mm
        self.z_lim = 99.00  # mm
        
        self.limits = {
            'X': self.x_lim,
            'Y': self.y_lim,
            'Z': self.z_lim
        }

    def _parse_node_list(self, node_str):
        """Helper function: Parses '1-39' or '1, 10, 15' into a set of string node IDs."""
        nodes = set()
        parts = [p.strip() for p in node_str.split(',')]
        for part in parts:
            if not part:
                continue
            if '-' in part:
                try:
                    start, end = part.split('-')
                    for i in range(int(start), int(end) + 1):
                        nodes.add(str(i))
                except ValueError:
                    pass # Ignore malformed ranges gracefully
            else:
                nodes.add(part)
        return nodes

    def validate_gcode(self, filepath):
        """
        Pre-flight check: runs a Kinematic Simulation to catch boundary errors,
        syntax errors, and worst-case probing collisions BEFORE the machine moves.
        """
        print(f"\n[*] Running Pre-Flight Diagnostics & Kinematic Simulation on '{filepath}'...")
        
        if not os.path.exists(filepath):
            print(f"[!] CRITICAL ERROR: File '{filepath}' not found.")
            return False

        errors_found = False
        
        # 1. Spawn the "Ghost Machine" using our current real-world positions
        virtual_pos = {
            'X': self.pokinator.axes.get('X', {}).get('current_pos', 0.0),
            'Y': self.pokinator.axes.get('Y', {}).get('current_pos', 0.0),
            'Z': self.pokinator.axes.get('Z', {}).get('current_pos', 0.0)
        }

        # Node filtering states
        only_nodes = set()
        skip_nodes = set()
        filtering_active = False
        
        # Added utf-8-sig to prevent BOM reading errors during pre-flight
        with open(filepath, mode='r', encoding='utf-8-sig') as file:
            reader = csv.reader(file)
            for line_num, row in enumerate(reader, start=1):
                
                if not row or all(cell.strip() == '' for cell in row):
                    continue
                    
                if len(row) < 2:
                    print(f"[!] Line {line_num} Error: Incomplete command. Missing instruction.")
                    errors_found = True
                    continue

                node = row[0].strip()
                instruction = row[1].strip().upper()

                # --- NODE FILTERING COMMANDS ---
                if instruction in ["ONLY_NODES", "SKIP_NODES"]:
                    if len(row) < 3:
                        print(f"[!] Line {line_num} Error: {instruction} missing node list parameters.")
                        errors_found = True
                        continue
                    
                    # Reconstruct the parameter string in case unquoted commas split it across cells
                    node_str = ",".join(cell.strip() for cell in row[2:])
                    parsed_nodes = self._parse_node_list(node_str)
                    
                    if instruction == "ONLY_NODES":
                        only_nodes.update(parsed_nodes)
                        filtering_active = True
                    elif instruction == "SKIP_NODES":
                        skip_nodes.update(parsed_nodes)
                    continue

                # --- APPLY FILTER ---
                if filtering_active and node not in only_nodes:
                    continue
                if node in skip_nodes:
                    continue
                
                # --- HOMING COMMAND VALIDATION ---
                if instruction.endswith("_HOME"):
                    if len(row) < 3:
                        print(f"[!] Line {line_num} Error: HOMING command missing speed. Expected format: Node#, AXIS_HOME, Speed(RPM).")
                        errors_found = True
                        continue
                        
                    axis = instruction.split("_")[0]
                    if axis not in self.limits:
                        print(f"[!] Line {line_num} Error: Invalid homing axis '{axis}'. Expected X, Y, or Z.")
                        errors_found = True
                        continue
                        
                    try:
                        home_speed = int(row[2].strip())
                        if home_speed <= 0:
                            print(f"[!] Line {line_num} Error: Homing speed must be > 0.")
                            errors_found = True
                        else:
                            # KINEMATIC UPDATE: Homing zeroes the ghost machine's axis
                            virtual_pos[axis] = 0.0
                    except ValueError:
                        print(f"[!] Line {line_num} Error: Homing speed '{row[2]}' must be an integer (RPM).")
                        errors_found = True
                    continue 

                # --- HOLD COMMAND VALIDATION ---
                if instruction == "HOLD":
                    if len(row) < 3:
                        print(f"[!] Line {line_num} Error: HOLD command missing time.")
                        errors_found = True
                        continue
                    try:
                        duration = float(row[2].strip())
                        if duration < 0:
                            print(f"[!] Line {line_num} Error: HOLD duration cannot be negative.")
                            errors_found = True
                    except ValueError:
                        print(f"[!] Line {line_num} Error: HOLD duration '{row[2]}' must be a number.")
                        errors_found = True
                    continue
                    
                # --- TRIGGER COMMAND VALIDATION ---
                if instruction == "TRIGGER":
                    if len(row) < 2:
                        print(f"[!] Line {line_num} Error: TRIGGER command incomplete. Expected: Node, TRIGGER.")
                        errors_found = True
                    continue

                # --- PROBE COMMAND VALIDATION ---
                if instruction == "Z_PROBE" or instruction == "Z_PROBE_LTD":
                    if len(row) < 6:
                        print(f"[!] Line {line_num} Error: {instruction} missing parameters. Expected: Node, {instruction}, Max_Dist, Probe_Speed, Force, Time.")
                        errors_found = True
                        continue
                        
                    try:
                        max_dist = float(row[2].strip())
                        
                        # KINEMATIC SIMULATION: Calculate worst-case collision scenario
                        worst_case_z = virtual_pos['Z'] + max_dist
                        
                        if worst_case_z > self.limits['Z'] or worst_case_z < 0:
                            print(f"[!] Line {line_num} Error: COLLISION DETECTED. The {instruction} requests a max travel of {max_dist}mm.")
                            print(f"    -> Theoretical starting position: {virtual_pos['Z']}mm.")
                            print(f"    -> Worst-case end position: {worst_case_z}mm. This exceeds limits (0 - {self.limits['Z']}mm).")
                            errors_found = True
                        else:
                            # Update the ghost machine's position to the worst-case endpoint for subsequent relative tracking
                            virtual_pos['Z'] = worst_case_z
                            
                    except ValueError:
                        print(f"[!] Line {line_num} Error: {instruction} max_dist '{row[2]}' must be a number.")
                        errors_found = True
                        
                    try:
                        probe_speed = int(row[3].strip())
                        if probe_speed <= 0:
                            print(f"[!] Line {line_num} Error: {instruction} probe_speed must be > 0.")
                            errors_found = True
                    except ValueError:
                        print(f"[!] Line {line_num} Error: {instruction} probe_speed '{row[3]}' must be an integer (RPM).")
                        errors_found = True
                        
                    try:
                        target_force = float(row[4].strip())
                        if target_force < 0:
                            print(f"[!] Line {line_num} Error: {instruction} target_force must be > 0.")
                            errors_found = True
                    except ValueError:
                        print(f"[!] Line {line_num} Error: {instruction} target_force '{row[4]}' must be a number.")
                        errors_found = True
                        
                    try:
                        hold_time = float(row[5].strip())
                        if hold_time < 0:
                            print(f"[!] Line {line_num} Error: {instruction} time cannot be negative.")
                            errors_found = True
                    except ValueError:
                        print(f"[!] Line {line_num} Error: {instruction} time '{row[5]}' must be a number.")
                        errors_found = True
                    continue

                # --- MOVE COMMAND VALIDATION ---
                if len(row) < 4:
                    print(f"[!] Line {line_num} Error: Move command missing parameters. Expected: Node, Axis, Location, Speed.")
                    errors_found = True
                    continue
                    
                axis = instruction
                if axis not in self.limits:
                    # Updated the error message to include the new commands
                    print(f"[!] Line {line_num} Error: Invalid axis '{axis}'. Expected X, Y, Z, HOLD, Z_PROBE, Z_PROBE_LTD, TRIGGER, _HOME, ONLY_NODES, or SKIP_NODES.")
                    errors_found = True
                    continue
                    
                try:
                    speed = int(row[3].strip())
                    if speed <= 0:
                        print(f"[!] Line {line_num} Error: Speed must be > 0. Got {speed} RPM.")
                        errors_found = True
                except ValueError:
                    print(f"[!] Line {line_num} Error: Speed '{row[3].strip()}' is not a valid integer.")
                    errors_found = True
                    
                try:
                    location = float(row[2].strip()) 
                    if location < 0 or location > self.limits[axis]:
                        print(f"[!] Line {line_num} Error: Target location {location}mm out of bounds for {axis}-axis (Max: {self.limits[axis]}mm).")
                        errors_found = True
                    else:
                        # KINEMATIC UPDATE: Move the ghost machine to the new safe absolute location
                        virtual_pos[axis] = location
                except ValueError:
                    print(f"[!] Line {line_num} Error: Location '{row[2].strip()}' is not a valid number.")
                    errors_found = True

        if errors_found:
            print("\n[-] Pre-Flight FAILED. The file contains unsafe commands or collision risks.")
            return False
            
        print("[+] Pre-Flight PASSED. Sequence is kinematically validated and safe.")
        return True

    def execute_program(self, filepath):
        """Executes a validated CSV program."""
        if not self.validate_gcode(filepath):
            return
            
        print(f"\n[*] Initiating Program Sequence: '{filepath}'...")
        
        probe_data = {}

        # Node filtering states
        only_nodes = set()
        skip_nodes = set()
        filtering_active = False
        
        # 'utf-8-sig' automatically strips the invisible BOM (ï»¿) left by Excel
        with open(filepath, mode='r', encoding='utf-8-sig') as file:
            reader = csv.reader(file)
            
            # Added enumerate to track the exact CSV line number alongside the Node number
            for line_num, row in enumerate(reader, start=1):
                if not row or all(cell.strip() == '' for cell in row):
                    continue
                    
                node = row[0].strip()
                instruction = row[1].strip().upper()

                # --- NODE FILTERING COMMANDS ---
                if instruction in ["ONLY_NODES", "SKIP_NODES"]:
                    node_str = ",".join(cell.strip() for cell in row[2:])
                    parsed_nodes = self._parse_node_list(node_str)
                    
                    if instruction == "ONLY_NODES":
                        only_nodes.update(parsed_nodes)
                        filtering_active = True
                        print(f"\n[*] [Line {line_num}] ONLY_NODES Engaged: Restricting sequence to {len(only_nodes)} specified nodes.")
                    elif instruction == "SKIP_NODES":
                        skip_nodes.update(parsed_nodes)
                        print(f"\n[*] [Line {line_num}] SKIP_NODES Engaged: Bypassing {len(skip_nodes)} specified nodes.")
                    continue

                # --- APPLY FILTER ---
                if filtering_active and node not in only_nodes:
                    continue
                if node in skip_nodes:
                    continue
                
                # --- HOMING ---
                if instruction.endswith("_HOME"):
                    axis = instruction.split("_")[0]
                    home_speed = int(row[2].strip())
                    print(f"\n[*] [Line {line_num} | Node {node}] Commanded {axis} to HOME at {home_speed} RPM...")
                    success = self.pokinator.home_axis(axis_name=axis, rpm=home_speed)
                    if not success:
                        print(f"[!] CRITICAL: Homing failed on {axis}. Halting program.")
                        break 
                    continue

                # --- HOLD ---
                if instruction == "HOLD":
                    duration = float(row[2].strip())
                    print(f"\n[*] [Line {line_num} | Node {node}] HOLDING position for {duration} seconds...")
                    time.sleep(duration)
                    continue
                    
                # --- STANDARD Z_PROBE ---
                if instruction == "Z_PROBE":
                    max_dist = float(row[2].strip())
                    probe_speed = int(row[3].strip())
                    target_force = float(row[4].strip())
                    hold_time = float(row[5].strip())
                    
                    print(f"\n[*] [Line {line_num} | Node {node}] PROBING Z-Axis (Max {max_dist}mm, Speed: {probe_speed} RPM, Target: {target_force}g)...")
                    
                    final_force, recorded_samples = self.pokinator.probe_z_axis(
                        max_distance_mm=max_dist, 
                        target_force_g=target_force, 
                        speed_rpm=probe_speed,
                        hold_time_s=hold_time  
                    )
                    
                    base_key = f"Force_Node{node}"
                    node_key = base_key
                    counter = 2
                    while node_key in probe_data:
                        node_key = f"{base_key}_{counter}"
                        counter += 1
                        
                    probe_data[node_key] = recorded_samples
                    
                    if len(recorded_samples) > 0:
                        avg_force = sum(recorded_samples) / len(recorded_samples)
                        print(f"    -> Hold Complete. Average Force: {avg_force:.2f}g")
                    else:
                        print(f"    -> Hold Complete. No samples recorded.")
                        
                    continue

                # --- Z_PROBE_LTD (EQUILIBRIUM PROBE) ---
                if instruction == "Z_PROBE_LTD":
                    max_dist = float(row[2].strip())
                    probe_speed = int(row[3].strip())
                    target_force = float(row[4].strip())
                    max_time = float(row[5].strip())
                    
                    print(f"\n[*] [Line {line_num} | Node {node}] PROBING Z-Axis LTD (Max {max_dist}mm, Target: {target_force}g, Max Time: {max_time}s)...")
                    
                    final_force, recorded_samples = self.pokinator.probe_z_axis_limited(
                        max_distance_mm=max_dist, 
                        target_force_g=target_force, 
                        speed_rpm=probe_speed,
                        hold_time_s=max_time  
                    )
                    
                    base_key = f"Force_Node{node}_LTD"
                    node_key = base_key
                    counter = 2
                    while node_key in probe_data:
                        node_key = f"{base_key}_{counter}"
                        counter += 1
                        
                    probe_data[node_key] = recorded_samples
                    
                    if len(recorded_samples) > 0:
                        avg_force = sum(recorded_samples) / len(recorded_samples)
                        print(f"    -> LTD Sequence Complete. Average Force over final 2s: {avg_force:.2f}g")
                    else:
                        print(f"    -> LTD Sequence Complete. No samples recorded.")
                        
                    continue

                # --- CAMERA TRIGGER ---
                if instruction == "TRIGGER":
                    print(f"\n[*] [Line {line_num} | Node {node}] Firing Manual 1ms TTL Trigger...")
                    self.pokinator.trigger_camera()
                    continue

                # --- STANDARD ABSOLUTE MOVE ---
                axis = instruction
                target_absolute_loc = float(row[2].strip())
                speed_rpm = int(row[3].strip())
                
                current_pos = self.pokinator.axes[axis]['current_pos']
                relative_distance = target_absolute_loc - current_pos
                
                print(f"\n[*] [Line {line_num} | Node {node}] Commanded {axis} to {target_absolute_loc}mm...")
                
                if relative_distance == 0:
                    print(f"    -> Already at {target_absolute_loc}mm. Skipping move.")
                    continue
                else:
                    print(f"    -> Calculating relative shift: moving {relative_distance:+.2f}mm at {speed_rpm} RPM")
                    
                self.pokinator.move_axis_distance(
                    axis_name=axis, 
                    distance_mm=relative_distance, 
                    target_rpm=speed_rpm
                )
                
        print("\n[+] Program sequence complete. Pokinator standing by.")
        
        # --- DATA EXPORT ---
        if probe_data:
            print("\n[*] Exporting probe data to 'recorded_forces.csv'...")
            max_len = max(len(lst) for lst in probe_data.values())
            time_col = [round(i * 0.1, 1) for i in range(max_len)]
            headers = ['Time'] + list(probe_data.keys())
            
            with open('recorded_forces.csv', mode='w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(headers)
                
                for i in range(max_len):
                    row_data = [time_col[i]]
                    for key in probe_data.keys():
                        if i < len(probe_data[key]):
                            row_data.append(f"{probe_data[key][i]:.2f}")
                        else:
                            row_data.append("") 
                            
                    writer.writerow(row_data)
                    
            print(f"[+] Data successfully saved. Logged {max_len} sample rows.")


# --- Execution Block ---
if __name__ == "__main__":
    
    PHIDGET_OFFSET = -6.1872E-004
    PHIDGET_GAIN = 762172.29269146 

    pokinator_controller = PokinatorController(
        dir_pin_id=board.C1,
        lc_offset=PHIDGET_OFFSET, 
        lc_gain=PHIDGET_GAIN
    )
    
    pokinator_controller.add_axis("Y", select_pin_id=board.C2, home_pin_id=board.C5, home_trigger=False)
    pokinator_controller.add_axis("X", select_pin_id=board.C3, home_pin_id=board.C6, home_trigger=False)
    pokinator_controller.add_axis("Z", select_pin_id=board.C4, home_pin_id=board.C7, home_trigger=False)

    programmer = PokinatorProgrammer(controller_instance=pokinator_controller)
    
    try:
        csv_file = 'g-code.csv'
        if os.path.exists(csv_file):
            programmer.execute_program(csv_file)
        else:
            print(f"\n[!] Place your '{csv_file}' in the same directory to execute.")
            
    finally:
        pokinator_controller.shutdown()
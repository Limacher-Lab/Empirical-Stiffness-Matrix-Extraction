import os
os.environ["BLINKA_FT232H"] = "1"

import board
from pokinator_controller import PokinatorController

# --- INPUT YOUR PHIDGET PARAMETERS HERE ---
# Replace these with the exact values from the Phidget Control Panel
PHIDGET_OFFSET = -4.1939E-005
PHIDGET_GAIN = 8.1414E+005    

def main():
    # Boot the machine with the specific load cell parameters
    machine = PokinatorController(
        dir_pin_id=board.C1, 
        lc_offset=PHIDGET_OFFSET, 
        lc_gain=PHIDGET_GAIN
    )
    
    try:
        # Prompt the user
        val = input("\nEnter max distance to travel (mm): ")
        dist = float(val)
        
        target_force = 200.0 # grams
        speed_rpm = 24       # Slow probing speed
        
        print(f"\nProbing at {speed_rpm} RPM for {dist}mm...")
        print(f"Will halt instantly if {target_force}g of force is detected.")
        
        # Execute the move
        final_force = machine.move_distance(dist, target_rpm=speed_rpm, max_force_g=target_force)
        
        # Report the final state
        print("\n=== TEST COMPLETE ===")
        print(f"Final Position: {machine.current_pos:.2f} mm")
        print(f"Final Force Detected: {final_force:.2f} g")
        
    except KeyboardInterrupt:
        print("\nEmergency Stop Triggered by User!")
    finally:
        machine.shutdown()

if __name__ == "__main__":
    main()
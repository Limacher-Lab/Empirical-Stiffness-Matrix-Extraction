import os
os.environ["BLINKA_FT232H"] = "1"
import time
import board
import digitalio

# Define the pin
REED_PIN = board.C3  

# Initialize the pin
reed_switch = digitalio.DigitalInOut(REED_PIN)
reed_switch.direction = digitalio.Direction.INPUT

# Note: reed_switch.pull = digitalio.Pull.UP has been REMOVED. 
# We are now relying on your physical hardware pull-up resistor.

print("Pokinator Reed Switch Test Initialized.")
print(f"Monitoring pin {REED_PIN} with physical pull-up...")
print("Waiting for magnetic trigger... (Press Ctrl+C to exit)")

# Keep track of the last state so we only print when a change actually occurs
last_state = reed_switch.value

try:
    while True:
        current_state = reed_switch.value
        
        if current_state != last_state:
            if current_state == False:
                # The switch connected the pin to GND
                print("\n[!] Magnet Detected! Switch is CLOSED (Pulled LOW).")
            else:
                # The switch opened, resistor pulled it back up to 3.3V
                print("[-] Magnet Removed. Switch is OPEN (Pulled HIGH).")
                
            last_state = current_state
            
        # A 10ms sleep acts as a debounce and keeps the script from hogging your CPU
        time.sleep(0.01)

except KeyboardInterrupt:
    print("\nTest terminated by user.")
finally:
    # Safely release the hardware pin
    reed_switch.deinit()
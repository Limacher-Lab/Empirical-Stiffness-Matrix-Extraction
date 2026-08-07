import time
import busio
import digitalio
import board

class PokinatorCore:
    """Low-level Hardware Abstraction Layer for the Pokinator CNC"""
    
    def __init__(self, spi_clock, spi_mosi, spi_miso, dir_pin_id, lead_mm=10, spr=1600, lc_offset=None, lc_gain=None, camera_pin_id=board.D4):
        self.spi = busio.SPI(spi_clock, MOSI=spi_mosi, MISO=spi_miso)
        
        self.dir = digitalio.DigitalInOut(dir_pin_id)
        self.dir.direction = digitalio.Direction.OUTPUT
        
        self.lead = lead_mm
        self.spr = spr
        self.steps_per_mm = self.spr / self.lead
        self.axes = {}
        
        # --- Phidget Load Cell Initialization ---
        self.lc_offset = lc_offset
        self.lc_gain = lc_gain

        # --- Camera TTL Trigger Setup ---
        self.camera_pin = digitalio.DigitalInOut(camera_pin_id)
        self.camera_pin.direction = digitalio.Direction.OUTPUT
        self.camera_pin.value = False # Default state is LOW (0V)
        
        if self.lc_offset is not None and self.lc_gain is not None:
            from Phidget22.Phidget import PhidgetException
            from Phidget22.Devices.VoltageRatioInput import VoltageRatioInput
            from Phidget22.BridgeGain import BridgeGain
            import time
            
            self.load_cell = VoltageRatioInput()
            
            # Optional: If your load cell is plugged into a specific port (e.g., Port 0), uncomment this:
            # self.load_cell.setChannel(0)
            
            try:
                self.load_cell.openWaitForAttachment(1000)
                
                self.load_cell.setBridgeEnabled(True)
                self.load_cell.setBridgeGain(BridgeGain.BRIDGE_GAIN_128)
                self.load_cell.setDataInterval(self.load_cell.getMinDataInterval()) 
                
                # --- The Wake-Up Protocol ---
                print("\n[*] Waiting for Phidget ADC to wake up and stabilize...")
                adc_ready = False
                for _ in range(50):  # Try for up to 5 seconds
                    try:
                        # Attempt to pull a valid reading
                        self.load_cell.getVoltageRatio()
                        adc_ready = True
                        break
                    except PhidgetException:
                        time.sleep(0.1) # Wait 100ms and try again
                        
                if adc_ready:
                    print("[+] Phidget Load Cell is online and transmitting.")
                else:
                    print("[!] Phidget ADC failed to stabilize. Check wiring.")
                
            except PhidgetException as e:
                print(f"[!] Failed to attach or configure Phidget Load Cell: {e}")

    def add_axis(self, axis_name, select_pin_id, home_pin_id=None, home_trigger=True):
        """Registers a new axis, its multiplex selector, and an optional homing switch"""
        select_pin = digitalio.DigitalInOut(select_pin_id)
        select_pin.direction = digitalio.Direction.OUTPUT
        select_pin.value = False
        
        axis_data = {
            'select_pin': select_pin,
            'current_pos': 0.0
        }
        
        if home_pin_id:
            home_pin = digitalio.DigitalInOut(home_pin_id)
            home_pin.direction = digitalio.Direction.INPUT
            axis_data['home_pin'] = home_pin
            axis_data['home_trigger'] = home_trigger
            
        self.axes[axis_name] = axis_data

    def _safe_set_pin(self, pin, value, retries=3, delay=0.03):
        """Robust pin setter bypassing FT232H collisions"""
        for _ in range(retries):
            try:
                pin.value = value
                return
            except Exception:
                time.sleep(delay)
        pin.value = value

    def _safe_read_pin(self, pin, retries=5, delay=0.01):
        """Robust pin reader bypassing FT232H collisions"""
        for _ in range(retries):
            try:
                return pin.value
            except Exception:
                time.sleep(delay)
        return pin.value

    def move_axis_distance(self, axis_name, distance_mm, target_rpm=120, max_force_g=None):
        """
        Translates a distance request into hardware steps, executes the move,
        and updates the internal position tracker. Optionally halts early if 
        max_force_g is exceeded (used for probing).
        """
        if axis_name not in self.axes:
            print(f"[!] Axis '{axis_name}' is not registered.")
            return 0.0

        axis = self.axes[axis_name]
        
        # Calculate step count and coordinate direction
        steps = int(abs(distance_mm) * self.steps_per_mm)
        direction_state = distance_mm < 0  # True for negative (towards motor), False for positive
        
        # Define the load cell stop condition (if requested and load cell exists)
        stop_condition = None
        if max_force_g is not None and hasattr(self, 'get_force_g'):
            stop_condition = lambda: self.get_force_g() >= max_force_g

        # Execute movement through the synchronized pipeline
        bytes_sent = self._execute_burst(axis_name, steps, target_rpm, direction_state, stop_condition)
        
        # Calculate exactly how far the machine actually traveled
        actual_steps = bytes_sent * 8
        actual_distance = actual_steps / self.steps_per_mm
        
        # Maintain coordinate geometry
        if direction_state:
            actual_distance = -actual_distance
            
        axis['current_pos'] += actual_distance
        
        # Return final force if probing, otherwise return 0.0
        if max_force_g is not None and hasattr(self, 'get_force_g'):
            return self.get_force_g()
            
        return 0.0
    
    def _execute_burst(self, axis_name, steps, rpm, direction_state, stop_condition=None):
        """Sends synchronized SPI pulses and pipelines the USB buffer"""
        if steps <= 0: return 0
        
        axis = self.axes[axis_name]
        frequency = max(1, int((rpm * self.spr) / 60))
        total_bytes = max(1, steps // 8)
        
        if self.spi.try_lock():
            try:
                self.spi.configure(baudrate=frequency)
                
                poll_rate = 0.01 if stop_condition is not None else 0.25
                max_bytes_per_chunk = max(1, int((frequency / 8) * poll_rate))
                bytes_sent = 0
                
                # GPIO Setup Sequence
                self._safe_set_pin(axis['select_pin'], True)
                time.sleep(0.02) # Settle high-impedance gate
                
                self._safe_set_pin(self.dir, direction_state)
                time.sleep(0.005) # Driver setup buffer
                
                start_time = time.time()    
                
                while bytes_sent < total_bytes:
                    
                    if stop_condition is not None and stop_condition():
                        break

                    chunk_size = min(max_bytes_per_chunk, total_bytes - bytes_sent)
                    self.spi.write(bytearray(chunk_size))
                    bytes_sent += chunk_size
                    
                    # Buffer Pipelining
                    chunk_duration = (chunk_size * 8) / frequency
                    if bytes_sent < total_bytes:
                        time.sleep(chunk_duration * 0.75)
                        
                # Absolute Hardware Synchronization
                expected_total_duration = (bytes_sent * 8) / frequency
                elapsed_time = time.time() - start_time
                
                drift_margin = expected_total_duration * 0.05
                remaining_time = expected_total_duration - elapsed_time + 0.05 + drift_margin
                
                if remaining_time > 0:
                    time.sleep(remaining_time)
                else:
                    time.sleep(0.05 + drift_margin) 
                    
                return bytes_sent
                
            finally:
                self._safe_set_pin(axis['select_pin'], False)
                self.spi.unlock()
                
        return 0
    
    def home_axis(self, axis_name, rpm=30, max_travel_mm=500):
        """
        Hardware-safe homing sequence. 
        Uses rhythmic Macro-Bursts to prevent acoustic resonance and USB collisions.
        """
        axis = self.axes[axis_name]
        if 'home_pin' not in axis:
            print(f"[!] No home pin registered for {axis_name}")
            return False
            
        print(f"[*] Homing {axis_name} at {rpm} RPM...")
        frequency = max(1, int((rpm * self.spr) / 60))
        
        # Macro-burst: Send ~150ms of movement per chunk. 
        # This prevents the 40Hz "pneumatic drill" vibration caused by 25ms chunks.
        chunk_size = max(1, int((frequency / 8) * 0.15)) 
        max_bytes = int(max_travel_mm * self.steps_per_mm) // 8
        
        if self.spi.try_lock():
            try:
                self.spi.configure(baudrate=frequency)
                
                # 1. Open Valve FIRST
                self._safe_set_pin(axis['select_pin'], True)
                time.sleep(0.02)
                
                # 2. Set Direction to Negative (True = Towards Motor)
                self._safe_set_pin(self.dir, True) 
                time.sleep(0.005)
                
                bytes_sent = 0
                while bytes_sent < max_bytes:
                    
                    # SAFE READ: The USB buffer is definitively empty here!
                    if self._safe_read_pin(axis['home_pin']) == axis['home_trigger']:
                        axis['current_pos'] = 0.0
                        print(f"\n[+] {axis_name} homed successfully. Position Zeroed.")
                        return True
                        
                    # Write macro-burst
                    self.spi.write(bytearray(chunk_size))
                    bytes_sent += chunk_size
                    
                    # ANTI-PIPELINING: Force Python to wait until the burst physically finishes,
                    # plus a 15ms buffer to let the motor's physical momentum settle 
                    # before the next burst. This entirely eliminates the grinding noise.
                    time.sleep(((chunk_size * 8) / frequency) + 0.015)
                    
                print(f"\n[-] Homing failed: Max travel ({max_travel_mm}mm) exceeded.")
                return False
                
            finally:
                self._safe_set_pin(axis['select_pin'], False)
                self.spi.unlock()
                
        return False
    
    def trigger_camera(self):
        """
        Fires a TTL pulse to trigger external image acquisition.
        The pulse is held HIGH for 1ms, easily exceeding the 1us minimum requirement.
        """
        self.camera_pin.value = True   # Pull HIGH (~3.3V TTL)
        time.sleep(0.001)              # Hold for 1 millisecond
        self.camera_pin.value = False  # Drop LOW (0V)

    def shutdown(self):
        """Safely releases all hardware pins"""
        for axis in self.axes.values():
            axis['select_pin'].deinit()
            if 'home_pin' in axis:
                axis['home_pin'].deinit()
        self.dir.deinit()
        self.spi.deinit()
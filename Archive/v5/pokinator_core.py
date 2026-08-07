import time
import busio
import digitalio

class PokinatorCore:
    """Low-level Hardware Abstraction Layer for the Pokinator CNC"""
    
    def __init__(self, spi_clock, spi_mosi, spi_miso, dir_pin_id, lead_mm=10, spr=1600):
        self.spi = busio.SPI(spi_clock, MOSI=spi_mosi, MISO=spi_miso)
        
        self.dir = digitalio.DigitalInOut(dir_pin_id)
        self.dir.direction = digitalio.Direction.OUTPUT
        
        self.lead = lead_mm
        self.spr = spr
        self.steps_per_mm = self.spr / self.lead
        self.axes = {}

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

    def _execute_burst(self, axis_name, steps, rpm, direction_state, stop_condition=None):
        """Sends synchronized SPI pulses and pipelines the USB buffer"""
        if steps <= 0: return 0
        
        axis = self.axes[axis_name]
        frequency = max(1, int((rpm * self.spr) / 60))
        total_bytes = max(1, steps // 8)
        
        if self.spi.try_lock():
            try:
                self.spi.configure(baudrate=frequency)
                
                poll_rate = 0.05 if stop_condition is not None else 0.25
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
    
    def home_axis(self, axis_name, rpm=45, max_travel_mm=500):
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

    def shutdown(self):
        """Safely releases all hardware pins"""
        for axis in self.axes.values():
            axis['select_pin'].deinit()
            if 'home_pin' in axis:
                axis['home_pin'].deinit()
        self.dir.deinit()
        self.spi.deinit()
import busio
import digitalio
import time

class PokinatorCore:
    """Low-level Hardware Abstraction Layer for the multiplexed Pokinator CNC"""
    
    def __init__(self, spi_clock, spi_mosi, spi_miso, dir_pin_id):
        # Initialize SPI (Global Pulse sent via SCK/AD0)
        self.spi = busio.SPI(spi_clock, MOSI=spi_mosi, MISO=spi_miso)
        
        # Initialize Global Direction Pin 
        self.dir = digitalio.DigitalInOut(dir_pin_id)
        self.dir.direction = digitalio.Direction.OUTPUT
        
        # Dictionary to hold our multiplexed axes
        self.axes = {}

    def add_axis(self, axis_name, select_pin_id, lead_mm=10, spr=1600):
        """Registers a new axis with its dedicated AND-gate selector pin"""
        sel_pin = digitalio.DigitalInOut(select_pin_id)
        sel_pin.direction = digitalio.Direction.OUTPUT
        sel_pin.value = False # Default to LOW (Valve closed)
        
        self.axes[axis_name] = {
            'select_pin': sel_pin,
            'lead': lead_mm,
            'spr': spr,
            'steps_per_mm': spr / lead_mm,
            'current_pos': 0.0
        }

    def _execute_burst(self, axis_name, steps, rpm, direction_state, stop_condition=None):
        """Sends SPI pulses to the specified axis by pulling its selector HIGH."""
        if steps <= 0 or axis_name not in self.axes: return 0
        
        axis = self.axes[axis_name]
        frequency = max(1, int((rpm * axis['spr']) / 60))
        total_bytes = max(1, steps // 8)
        
        # 1. OPEN THE VALVE FIRST 
        self._safe_set_pin(axis['select_pin'], True)
        
        # 2. SET THE DIRECTION SECOND
        self._safe_set_pin(self.dir, direction_state)
        
        bytes_sent = 0
        import time
        try:
            if self.spi.try_lock():
                try:
                    self.spi.configure(baudrate=frequency)
                    
                    # Dynamic Polling Rate
                    poll_rate = 0.05 if stop_condition is not None else 0.25
                    max_bytes_per_chunk = max(1, int((frequency / 8) * poll_rate))
                    
                    while bytes_sent < total_bytes:
                        # Evaluate the universal stop condition FIRST
                        if stop_condition is not None and stop_condition():
                            print(f"\n[!] Interrupt Condition Met on {axis_name}! Halting motor.")
                            break
                        
                        # Execute the safe chunk
                        chunk_size = min(max_bytes_per_chunk, total_bytes - bytes_sent)
                        self.spi.write(bytearray(chunk_size))
                        bytes_sent += chunk_size
                        
                        # CRITICAL FIX: Wait for the physical world to catch up to Python!
                        # pyftdi's spi.write() dumps to USB and returns instantly.
                        # We must sleep for the exact duration it takes the FT232H to shift these bits.
                        chunk_time_seconds = (chunk_size * 8) / frequency
                        time.sleep(chunk_time_seconds)
                        
                finally:
                    self.spi.unlock()
        finally:
            # 3. CLOSE THE VALVE 
            # Add a microscopic buffer to ensure the FT232H internal shifter is completely empty
            time.sleep(0.01)
            self._safe_set_pin(axis['select_pin'], False)
            
        return bytes_sent

    def _safe_set_pin(self, pin, state, retries=5, delay=0.03):
        """
        Forces a GPIO state change. Catches pyftdi MPSSE buffer collisions 
        and retries until the Windows USB latency timer clears.
        """
        import time
        for attempt in range(retries):
            try:
                time.sleep(delay) # Wait 30ms to clear the 16ms FTDI buffer
                pin.value = state
                return # Success! Exit the function.
            except Exception as e:
                if attempt == retries - 1:
                    print(f"\n[CRITICAL] Hardware out of sync. Failed to set valve to {state}: {e}")
                else:
                    # The USB buffer was still choked. Loop back and try again.
                    pass

    def shutdown(self):
        """Releases the hardware pins"""
        self.dir.deinit()
        for axis in self.axes.values():
            axis['select_pin'].deinit()
        self.spi.deinit()
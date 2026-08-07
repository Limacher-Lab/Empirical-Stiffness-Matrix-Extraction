import os
os.environ["BLINKA_FT232H"] = "1"

import time
import board
import busio
import digitalio

class PokinatorHighSpeed:
    def __init__(self, dir_pin_id, lead_mm=10, spr=1600):
        # Initialize SPI (AD0 is SCK)
        self.spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
        
        self.dir = digitalio.DigitalInOut(dir_pin_id)
        self.dir.direction = digitalio.Direction.OUTPUT
        
        self.lead = lead_mm
        self.spr = spr
        self.steps_per_mm = self.spr / self.lead
        self.current_pos = 0.0

    def move_distance(self, distance_mm, target_rpm=60):
        """A robust, constant-velocity move. Best for speeds under 60 RPM."""
        # 1. Setup Direction (+ Away, - Towards)
        self.dir.value = False if distance_mm > 0 else True
        
        total_steps = int(abs(distance_mm) * self.steps_per_mm)
        
        print(f"Executing Constant Speed Move: {abs(distance_mm)}mm at {target_rpm} RPM")
        
        # Execute the entire move in one command.
        # The _execute_burst function will handle breaking it into safe USB chunks.
        self._execute_burst(total_steps, target_rpm)
        
        self.current_pos += distance_mm
        print(f"Final Position: {self.current_pos}mm")

    def _execute_burst(self, steps, rpm, stop_sensor=None, trigger_value=True):
        """
        Helper to send a SPI pulse burst. 
        If stop_sensor is provided, it polls the sensor between chunks.
        Returns the actual number of bytes sent before stopping.
        """
        if steps <= 0: return 0
        
        frequency = max(1, int((rpm * self.spr) / 60))
        total_bytes = max(1, steps // 8)
        
        if self.spi.try_lock():
            try:
                self.spi.configure(baudrate=frequency)
                
                # Dynamic Polling Rate:
                # If a sensor is active, chunk every 0.05s (50ms) for fast reaction time.
                # If no sensor, use the highly efficient 0.25s (250ms) chunk.
                poll_rate = 0.05 if stop_sensor is not None else 0.25
                max_bytes_per_chunk = max(1, int((frequency / 8) * poll_rate))
                
                bytes_sent = 0
                while bytes_sent < total_bytes:
                    # 1. Check the hardware interrupt FIRST
                    if stop_sensor is not None and stop_sensor.value == trigger_value:
                        print("Interrupt triggered! Halting motor.")
                        break # Instantly kill the loop
                    
                    # 2. Execute the safe chunk
                    chunk_size = min(max_bytes_per_chunk, total_bytes - bytes_sent)
                    self.spi.write(bytearray(chunk_size))
                    bytes_sent += chunk_size
                    
                return bytes_sent # Return exactly how far we moved
            finally:
                self.spi.unlock()
        return 0

    def shutdown(self):
        self.dir.deinit()
        self.spi.deinit()

if __name__ == "__main__":
    pokinator = PokinatorHighSpeed(board.C1)
    try:
        while True:
            val = input("\nEnter distance, mm (+Away, -Towards) or 'q': ")
            if val.lower() == 'q': break
            
            # Test at 240 RPM (40 mm/s)
            pokinator.move_distance(float(val), target_rpm=240)
    finally:
        pokinator.shutdown()
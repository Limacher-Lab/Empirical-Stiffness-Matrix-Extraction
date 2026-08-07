import busio

import digitalio



class PokinatorCore:

    """Low-level Hardware Abstraction Layer for the Pokinator CNC"""

   

    def __init__(self, spi_clock, spi_mosi, spi_miso, dir_pin_id, lead_mm=10, spr=1600):

        # Initialize SPI

        self.spi = busio.SPI(spi_clock, MOSI=spi_mosi, MISO=spi_miso)

       

        # Initialize Direction Pin

        self.dir = digitalio.DigitalInOut(dir_pin_id)

        self.dir.direction = digitalio.Direction.OUTPUT

       

        # Machine Constants

        self.lead = lead_mm

        self.spr = spr

        self.steps_per_mm = self.spr / self.lead

        self.current_pos = 0.0



    def _execute_burst(self, steps, rpm, stop_condition=None):

        """

        Helper to send a SPI pulse burst.

        If stop_condition (a function) is provided, it evaluates it between chunks.

        """

        if steps <= 0: return 0

       

        frequency = max(1, int((rpm * self.spr) / 60))

        total_bytes = max(1, steps // 8)

       

        if self.spi.try_lock():

            try:

                self.spi.configure(baudrate=frequency)

               

                # Dynamic Polling Rate: 50ms if watching a condition, 250ms if blind

                poll_rate = 0.05 if stop_condition is not None else 0.25

                max_bytes_per_chunk = max(1, int((frequency / 8) * poll_rate))

               

                bytes_sent = 0

                while bytes_sent < total_bytes:

                    # 1. Evaluate the universal stop condition FIRST

                    if stop_condition is not None and stop_condition():

                        print("\n[!] Interrupt Condition Met! Halting motor.")

                        break

                   

                    # 2. Execute the safe chunk

                    chunk_size = min(max_bytes_per_chunk, total_bytes - bytes_sent)

                    self.spi.write(bytearray(chunk_size))

                    bytes_sent += chunk_size

                   

                return bytes_sent

            finally:

                self.spi.unlock()

        return 0



    def shutdown(self):

        """Releases the hardware pins"""

        self.dir.deinit()

        self.spi.deinit() 


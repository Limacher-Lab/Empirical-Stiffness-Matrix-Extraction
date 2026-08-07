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

       

        # Load Cell Parameters

        self.lc_offset = lc_offset

        self.lc_gain = lc_gain

       

        print("Initializing Phidget Load Cell...")

        self.lc = VoltageRatioInput()

        self.lc.setChannel(lc_channel)

        self.lc.openWaitForAttachment(Phidget.DEFAULT_TIMEOUT)

        self.lc.setBridgeEnabled(True)

        self.lc.setBridgeGain(BridgeGain.BRIDGE_GAIN_128)

       

        # Hardware calibration delay required by the Phidget API

        time.sleep(3)

        print("Load Cell Ready.")



    def get_force_grams(self):

        """Calculates current force based on V/V, offset, and gain"""

        vv = self.lc.getVoltageRatio()

        return (vv - self.lc_offset) * self.lc_gain



    def move_distance(self, distance_mm, target_rpm=60, max_force_g=None):

        """Moves distance_mm, but aborts instantly if max_force_g is exceeded"""

        if distance_mm == 0: return self.get_force_grams()



        self.dir.value = False if distance_mm > 0 else True

        total_steps = int(abs(distance_mm) * self.steps_per_mm)

       

        # Create the dynamic stop condition (a lambda function)

        if max_force_g is not None:

            # We use absolute value so it stops whether pushing or pulling

            stop_cond = lambda: abs(self.get_force_grams()) >= max_force_g

        else:

            stop_cond = None

       

        # Execute the move, checking the stop_cond 20 times a second

        bytes_sent = self._execute_burst(total_steps, target_rpm, stop_condition=stop_cond)

       

        # Update Position

        actual_distance = (bytes_sent * 8) / self.steps_per_mm

        if distance_mm < 0:

            self.current_pos -= actual_distance

        else:

            self.current_pos += actual_distance

           

        return self.get_force_grams() # Return the final force reading



    def shutdown(self):

        """Safely close hardware connections"""

        self.lc.close()

        super().shutdown()



# --- Execution Block ---

if __name__ == "__main__":

    pokinator = PokinatorController(board.C1)

   

    try:

        while True:

            val = input("\nEnter distance, mm (+Away, -Towards) or 'q': ")

            if val.lower() == 'q': break

           

            # Since the hardware proved it can handle 240 RPM instantly, we can keep using it

            pokinator.move_distance(float(val), target_rpm=240)

    finally:

        pokinator.shutdown()


import time
from machine import Pin #, Timer
# import rp2
# import sys
from utilisation import Utilisation
from digimatic import MitutoyoGauge
#region IO Comments

# IO Config for original single interface
#
#
# GPIO  6 - Data           input   IN 0
# GPIO  7 - Clock          input   IN 1
# GPIO  8 - Sample Button  input   IN 2
#          _______
# GPIO  9 - Request        output  SET and SIDESET

#_data        = Pin(6, mode=Pin.IN, pull=Pin.PULL_UP)
#_clock       = Pin(7, mode=Pin.IN, pull=Pin.PULL_UP)
#_data_button = Pin(8, mode=Pin.IN, pull=Pin.PULL_UP)
#_nreq        = Pin(9, mode=Pin.OPEN_DRAIN)

# IO Config for quad interface board
#
#
# GPIO  2 - Data           input   IN 0
# GPIO  3 - Clock          input   IN 1
# GPIO  4 - Sample Button  input   IN 2
#          _______
# GPIO  5 - Request        output  SET and SIDESET
#
#
# GPIO 10 - Data           input   IN 0
# GPIO 11 - Clock          input   IN 1
# GPIO 12 - Sample Button  input   IN 2
#          _______
# GPIO 13 - Request        output  SET and SIDESET
#
#
# GPIO 18 - Data           input   IN 0
# GPIO 19 - Clock          input   IN 1
# GPIO 20 - Sample Button  input   IN 2
#          _______
# GPIO 21 - Request        output  SET and SIDESET
#
#
# GPIO 26 - Data           input   IN 0
# GPIO 27 - Clock          input   IN 1
# GPIO 28 - Sample Button  input   IN 2
#          _______
# GPIO 22 - Request        output  SET and SIDESET

#endregion

# An example class holding some information relating to each gauge
class GaugeState:
    def __init__(self):
        self.start_time = time.ticks_ms()

    def store_start_time(self):
        self.start_time = time.ticks_ms()

    def get_period_ms(self):
        return time.ticks_ms() - self.start_time

# Stores the number of gauge reads performed
count = 0

# Called when the gauge class is ready get readings
def on_started(gauge):
    print(f"{gauge} Started:")
    gauge.pin_print( lambda s: print("    " + s) )
    gauge.state = GaugeState()
    gauge.state.store_start_time()
    gauge.get_reading_async()

# Called each time a reading is available
def on_reading(gauge, reading, error):
    global count

    if error:
        print(f"{gauge} reading error: ", reading.error_message)
    else:
        print(f"{gauge} {gauge.state.get_period_ms()}ms {reading} {reading.nibble_string()}")
        count = count +1

    gauge.state.store_start_time()
    gauge.get_reading_async()

# Called when a reading request times out
def on_timeout(gauge):
    print(f"{gauge} Timeout")
    gauge.state.store_start_time()
    gauge.get_reading_async()

# Let's see how much processing power we are using
utilisation = Utilisation(1)
print(f"Utilisation calibration reps = {utilisation.calibration_reps}")

gauge_list = []

# Data and clock pin pairs must be consecutive for the PIO code to work
if False:
    # Single gauge board
    gauge_list.append(MitutoyoGauge(smid=0, data=Pin( 6), clock=Pin (7), button=Pin( 8), nreq=Pin( 9)))
else:
    # Quad gauge board
    gauge_list.append(MitutoyoGauge(smid=0, data=Pin( 2), clock=Pin( 3), button=Pin( 4), nreq=Pin( 5)))
    gauge_list.append(MitutoyoGauge(smid=1, data=Pin(10), clock=Pin(11), button=Pin(12), nreq=Pin(13)))
    gauge_list.append(MitutoyoGauge(smid=2, data=Pin(18), clock=Pin(19), button=Pin(20), nreq=Pin(21)))
    gauge_list.append(MitutoyoGauge(smid=3, data=Pin(26), clock=Pin(27), button=Pin(28), nreq=Pin(22)))

for gauge in gauge_list:
    gauge.started_callback = on_started
    gauge.reading_callback = on_reading
    gauge.timeout_callback = on_timeout
    gauge.activate()

RUNNING_TIME = const(1)

# Give time to receive some readings
utilisation.measure( RUNNING_TIME )

print(f"{count} readings in {RUNNING_TIME}s with {utilisation}")
print()

for gauge in gauge_list:
    print(f"{gauge} Stopping")
    gauge.deactivate()

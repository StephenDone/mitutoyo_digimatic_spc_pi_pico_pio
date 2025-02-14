# Measures approximate interrupt driven processor utilisation

import time
from machine import Pin, Timer

class Utilisation:
    def __init__(self, calibration_time=1.0):
        self.calibrate(calibration_time)

    def calibrate(self, time):
        ( self.calibration_reps, self.calibration_time ) = self._sleep(time)

    def measure(self, time):
        ( self.measure_reps, self.measure_time ) = self._sleep(time)

        self.idle_fraction = (self.measure_reps * self.calibration_time) \
                           / (self.measure_time * self.calibration_reps)
        self.busy_fraction = 1 - self.idle_fraction
    
    def _sleep(self, time_s):
        reps = 0
        end_ticks = time.ticks_add( time.ticks_ms(), int( time_s * 1000 ) )

        while time.ticks_ms() < end_ticks:
            reps=reps+1

        return ( reps, time_s )

    def __str__(self):
        return f"{self.busy_fraction:.1%} processor utilisation"

if __name__ == "__main__":

    # Interrupt callback that deliberately blocks to use some cycles
    def bad_timer_callback(timer):
        led.on()
        time.sleep_ms(50)
        led.off()

    # Better callback that doesn't block for so long
    def good_timer_callback(timer):
        led.toggle()

    print( "Running Utilisation Module Example:" )
    print()

    utilisation = Utilisation(2)
    print(f"Calibration reps = {utilisation.calibration_reps} reps in {utilisation.calibration_time} seconds")
    print()

    led = Pin("LED", Pin.OUT)

    timer = Timer(period=100, mode=Timer.PERIODIC, callback=bad_timer_callback)
    utilisation.measure(3)
    print(f"{utilisation.measure_reps} reps ({utilisation}) in {utilisation.measure_time} seconds")
    print()
    timer.deinit()

    timer = Timer(period=100, mode=Timer.PERIODIC, callback=good_timer_callback)
    utilisation.measure(3)
    print(f"{utilisation.measure_reps} reps ({utilisation}) in {utilisation.measure_time} seconds")
    print()
    timer.deinit()

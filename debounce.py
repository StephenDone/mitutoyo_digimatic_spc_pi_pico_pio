# Interrupt driven contact debounce class
# Assumes button push is active low. i.e. Push to make switch between input pin and ground.
# Both falling and rising edges are debounced

import time
from machine import Pin

class DebouncedSwitch:

    def __init__(self, pin: Pin, button_press_callback, debounce_period_ms: int=200):
        self._callback = button_press_callback
        self._pin = pin
        self._pin.init(Pin.IN, Pin.PULL_UP)
        self._pin.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._button_callback)
        self.debounce_period_ms = debounce_period_ms
        self._debouncing = False
        self._debounce_end_ticks_ms = 0

    def _button_callback(self, pin):
        if self._debouncing:
            self._debouncing = time.ticks_ms() < self._debounce_end_ticks_ms

        if not self._debouncing:
            falling = (pin.irq().flags() & Pin.IRQ_FALLING) == Pin.IRQ_FALLING

            if falling and self._callback is not None:
                    self._callback(self._pin)

            self._debouncing = True                
            self._debounce_end_ticks_ms = time.ticks_ms() + self.debounce_period_ms

if __name__ == "__main__":

    print( "Running Switch Debounce Module Example:" )
    print()

    switch = DebouncedSwitch(
         pin=Pin(20), 
         debounce_period_ms=200,
         button_press_callback=lambda pin:print(f"Button press on {pin}") 
    )

    print(f"Waiting for button presses on {switch._pin}")
    time.sleep(10)
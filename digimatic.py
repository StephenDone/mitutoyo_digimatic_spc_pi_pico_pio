import time
from machine import Pin, Timer
import rp2

#region mitutoyo pio code

@rp2.asm_pio(in_shiftdir=rp2.PIO.SHIFT_RIGHT, 
             sideset_init=rp2.PIO.OUT_HIGH,     # Side set for request line, active low
             )
def get_nibbles():

    pull()                  # Wait on a value from the TX FIFO

    set(x, 31).side(0)      # Loop 32 times, side set 'request data active low'

    label("bitloop")

    wait(0, pin, 1)         # Wait for clock to go low
    in_(pins, 1)            # Read in 1 data bit
    wait(1, pin, 1)         # Wait for clock to go high

    jmp(x_dec, "bitloop")

    mov(y, isr)             # Save the first 32 bits until we've got the second

    set(x, 19) .side(1)     # Loop 20 times, side set 'request data inactive high'

    label("bitloop2")

    wait(0, pin, 1)         # Wait for clock to go low
    in_(pins, 1)            # Read in 1 data bit
    wait(1, pin, 1)         # Wait for clock to go high

    jmp(x_dec, "bitloop2")

    # At this point, we have got 4x13=52 bits or 13 nibbles of data.
    # We have 3 spare nibbles that we can fill up!

    set(x,0)
    jmp(pin,"button_up")    # Final nibble is 0, or 1 when data button down
    set(x,1)
    label("button_up")

    in_(x, 12)              # Add final 12 bits to the second 32-bit value

    mov(x, isr)             # Save second 32-bit ISR value to x
    mov(isr, y)             # Put first 32-bit y value back in ISR
    push()                  # Push the first 32 bits

    mov(isr, x)             # Restore the first 32-bit value from y
    push()                  # Push the final 32-bit value

    irq(rel(0))

#endregion

class MitutoyoReading:

    # todo - time the methods

    # Creates a reading from 2x 32-bit words
    def __init__(self, a, b):
        self.a = a
        self.b = b
        self.nibbles = bytearray(14)

        self._extract_nibbles()
        self._validate_nibbles()
        self._create_reading()

    # Creates a reading by getting 2x 32-bit words from the StateMachine FIFO
    @classmethod
    def from_sm( cls, sm ):
        a = sm.get()
        b = sm.get()
        return cls( a, b )

    # Extracts 14 nibbles from the 2x 32-bit words. The remaining (16-14=)2 nibbles are unused
    # The first 13 nibbles are from the gauge. The final nibble returns the button state.
    def _extract_nibbles(self):
        # Extract first 8 nibbles from first 32-bit value
        a = self.a
        for n in range(8):  
            self.nibbles[n  ] = a & 0xF
            a = a >> 4

        # Extract first 6 nibbles from second 32-bit value
        b = self.b
        for n in range(6):  
            self.nibbles[n+8] = b & 0xF
            b = b >> 4

    # Sanity checks the nibbles, to make sure their values are at least possible.
    def _validate_nibbles(self):
        self.error = False
        self.error_message = ""
        
        # 0..3
        for n in range(4):
            if self.nibbles[n] != 0xF:
                self._reading_error("Data must begin with 0xFFFF")
                return False

        # 4
        if not self.nibbles[4] in [0, 8]:
            self._reading_error(f"Invalid negative nibble value 0x{self.nibbles[4]:x}")
            return False

        # 5..10
        for digit in range(6):
            digit_value = self.nibbles[digit+5]
            if digit_value > 9:
                self._reading_error(f"Digit[{digit}] out of range: 0x{digit_value:x}")
                return False

        # 11
        if self.nibbles[11] < 1 or self.nibbles[11] > 5:
            self._reading_error(f"Decimal position out of range: 0x{self.nibbles[11]:x}")
            return False

        # 12
        if not self.nibbles[12] in [0, 1]:
            self._reading_error(f"Invalid units nibble value 0x{self.nibbles[12]:x}")
            return False

        # 13
        if not self.nibbles[13] in [0, 1]:
            self._reading_error(f"Invalid button value: 0x{self.nibbles[13]:x}")
            return False
        
        return True
      
    def _reading_error(self, msg):
        self.error = True
        self.error_message = f"{msg} ({self.nibble_string()})"

    def _create_reading(self):
        # 0..3 are just 0xFFFF

        # 4
        self.negative = self.nibbles[4]==8

        # 5..10
        self.digits=0
        for digit in range(6):
            self.digits = self.digits*10 + self.nibbles[digit+5]

        # 11
        self.dps = self.nibbles[11]
        self.value = self.digits / (10**self.dps) * (-1 if self.negative else 1)

        # 12
        self.inches = self.nibbles[12] == 1
        self.units = 'in' if self.inches else 'mm'

        # 13
        self.button_down = self.nibbles[13]==1

    def __str__(self): 
        value = f"{self.value:{self.dps+3}.{self.dps}f}"
        return f"value={value:6} {self.units}{'  Button Down' if self.button_down else ''}"

    def nibble_string(self): 
        return ''.join('{:01x}'.format(x) for x in self.nibbles)

class MitutoyoGauge:

    # read_timeout_ms may need increasing for very slow gauges
    # Note: Some gauges are slower to respond when measurements are changing

    # PIO_FREQUENCY may need increasing for very fast gauges
    PIO_FREQUENCY = const(100000)

    def __init__(self, smid, data, clock, button, nreq, read_timeout_ms=250):
        self.smid = smid
        self.data = data
        self.clock = clock
        self.button = button
        self.nreq = nreq
        self.read_timeout_ms = read_timeout_ms

        self.data.init(mode=Pin.IN, pull=Pin.PULL_UP)
        self.clock.init(mode=Pin.IN, pull=Pin.PULL_UP)
        self.button.init(mode=Pin.IN, pull=Pin.PULL_UP)
        self.nreq.init(mode=Pin.OPEN_DRAIN)

        self.sm  = rp2.StateMachine(smid, get_nibbles, freq=self.PIO_FREQUENCY, 
                            in_base=data, 
                            sideset_base=nreq, 
                            jmp_pin=button)
        
        self.sm.irq(self.irq_action)
        self._timeout_timer = None

        # Somewhere to store user defined data relating to the gauge
        self.state = None

        self.started_callback = None
        self.reading_callback = None
        self.timeout_callback = None

    def activate(self):
        self.nreq.value(1)
        self._timeout_timer = Timer(period=self.read_timeout_ms, mode=Timer.ONE_SHOT, callback=self.on_quiesce_callback)

    def deactivate(self):
        self.sm.active(0)
        self.nreq.value(1)

    def get_reading_async(self):
        # We only request another reading if there is not one already pending
        # Because if we did fill the TX FIFO, this code would stall.
        if self.sm.tx_fifo() == 0 and self._timeout_timer is None:
            self.sm.put(0)
            self._timeout_timer = Timer(period=self.read_timeout_ms, mode=Timer.ONE_SHOT, callback=self.on_timeout_callback)

    def on_quiesce_callback(self, timer):
        self._timeout_timer.deinit()
        self._timeout_timer = None
        self.sm.active(1)
        if self.started_callback:
            self.started_callback(self)

    def on_timeout_callback(self, timer):
        # Dispose of the timer that triggered this event
        self._timeout_timer.deinit()
        self._timeout_timer = None

        # Critical line of code that recovers from errors
        self.sm.restart()

        # Call the user's event handler for timeouts
        if self.timeout_callback is not None:
            self.timeout_callback(self)

    def irq_action(self, p):
        # We've received a response from the PIO, so cancel the timeout timer
        if self._timeout_timer is not None:
            self._timeout_timer.deinit()
            self._timeout_timer = None

        # Process the reading and pass it to the user's event handler
        if self.reading_callback:
            reading = MitutoyoReading.from_sm( self.sm )
            self.reading_callback(self, reading, reading.error)

    def __str__(self):
        return f"gauge[{self.smid}]"

    def pin_print(self, print_callback):
        print_callback(f"data={self.data}")
        print_callback(f"clock={self.clock}")
        print_callback(f"button={self.button}")
        print_callback(f"nreq={self.nreq}")

if __name__ == "__main__":
    def on_started(gauge):
        print(f"{gauge} Started")
        gauge.get_reading_async()

    def on_reading(gauge, reading, error):
        if error: print(f"{gauge} reading error: ", reading.error_message)
        else:     print(f"{gauge} {reading}")
        gauge.get_reading_async()

    def on_timeout(gauge):
        print(f"{gauge} Timeout")
        gauge.get_reading_async()

    print( "Running Digimatic Module Example:" )
    print()

    gauge = MitutoyoGauge(smid=0, data=Pin( 2), clock=Pin( 3), button=Pin( 4), nreq=Pin( 5))

    gauge.started_callback = on_started
    gauge.reading_callback = on_reading
    gauge.timeout_callback = on_timeout

    print(f"{gauge} Starting:")
    gauge.pin_print( lambda s: print("    " + s) )
    gauge.activate()

    time.sleep(3)

    print(f"{gauge} Stopping")
    gauge.deactivate()

# import micropython
import time
from machine import Pin, Timer
import rp2
import sys

#region Blink 1Hz Pio Code

@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def blink_1hz():

    wrap_target()

    # Cycles: 1 + 1 + 6 + 32 * (30 + 1) = 1000
    irq(rel(0))
    set(pins, 1)
    set(x, 31)                  [5]
    label("delay_high")
    nop()                       [29]
    jmp(x_dec, "delay_high")

    # Cycles: 1 + 7 + 32 * (30 + 1) = 1000
    #irq(rel(1))
    set(pins, 0)
    set(x, 31)                  [6]
    label("delay_low")
    nop()                       [29]
    jmp(x_dec, "delay_low")

    wrap()

#sm = rp2.StateMachine(0, blink_1hz, freq=2000, set_base=Pin(25))

#endregion

#region Try PIO Input FIFO

#@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW, in_shiftdir=rp2.PIO.SHIFT_LEFT)
@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW, in_shiftdir=rp2.PIO.SHIFT_RIGHT)
def rx_fifo():

    set(y, 0)

    wrap_target()

    set(x, 7)

    label("nibbleloop")
    in_(y, 4)
    
    jmp(y_dec, "dummyjump")
    label("dummyjump")

    jmp(x_dec, "nibbleloop")
    push()

    # irq(block, rel(0))

    wrap()

#sm = rp2.StateMachine(0, rx_fifo, freq=2000, set_base=Pin(25))
#endregion

#region PIO Code Snippets

    # We currently have memory for 6 more instructions
    # nop()
    # nop()
    # nop()
    # nop()
    # nop()
    # nop()

    # Delay loop on startup, with nreq high, to ensure no data is mid-way through being sent
    # Delay will depend on the clock frequency set
    #
    # x=32, y=32, delay=1
    # Delay = 1 + x + x + (1+delay)xy = 65 + (1+1) x 32 x 32 = 2113 cycles, so 211ms at 10kHz clock.
    
    # set(x, 31).side(1)                      # 1 instruction
    # label("delayloop_outer")

    # set(y, 31)                              # 1 instruction x 32x reps

    # label("delayloop_inner")
    # jmp(y_dec, "delayloop_inner").delay(1)  # (1i+1d) x 32x reps x 32y reps

    # jmp(x_dec, "delayloop_outer")           # 1 instruction x 32x reps

    # Maybe IRQ here to signal we are ready

    # wrap_target()
    # ...
    # wrap()

#endregion

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

class DebouncedSwitch:

    def __init__(self, pin: Pin, button_press_callback):
        self._callback = button_press_callback
        self._pin = pin
        self._pin.init(Pin.IN, Pin.PULL_UP)
        self._pin.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._button_callback)
        self._debouncing = False
        self._debounce_time = 0

    def _button_callback(self, pin):
        if self._debouncing:
            self._debouncing = (time.ticks_ms() - self._debounce_time) < 200

        if not self._debouncing:
            falling = pin.irq().flags() & Pin.IRQ_FALLING == Pin.IRQ_FALLING

            if falling and self._callback is not None:
                    self._callback(self._pin)

            self._debouncing=True                
            self._debounce_time=time.ticks_ms()

#region mitutoyo test program

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


class GaugeState:
    def __init__(self):
        self.start_time = time.ticks_ms()

    def store_start_time(self):
        self.start_time = time.ticks_ms()

    def get_period_ms(self):
        return time.ticks_ms() - self.start_time

if False:
    button = DebouncedSwitch(Pin(8), lambda pin : print (f"Button Press on {pin}"))
    time.sleep(10)
    exit()

if True:
    count = 0

    def on_started(gauge):
        print(f"{gauge} Started: {gauge.state.get_period_ms()}ms")
        gauge.state.store_start_time()
        gauge.get_reading_async()

    def on_reading(gauge, reading, error):
        global count

        if error:
            print(f"{gauge} reading error: ", reading.error_message)
        else:
            print(f"{gauge} {gauge.state.get_period_ms()}ms {reading} {reading.nibble_string()}")
            # print(".", end='')
            count = count +1

        gauge.state.store_start_time()
        gauge.get_reading_async()

    def on_timeout(gauge):
        # print(f"{gauge} Timeout")
        print(f"-")
        gauge.state.store_start_time()
        gauge.get_reading_async()

    def timer_callback(timer):
        led.on()
        time.sleep_ms(50)
        led.off()

    utilisation = Utilisation(1)
    print(f"Calibration reps = {utilisation.calibration_reps}")

    # led = Pin(25, Pin.OUT)
    # timer = Timer(period=100, mode=Timer.PERIODIC, callback=timer_callback)
    # utilisation.measure(3)
    # print(utilisation)
    # sys.exit()

    # Single gauge board
    # gauge = mitutoyo_gauge(smid=0, data=Pin(6), clock=Pin(7), button=Pin(8), nreq=Pin(9))

    # Quad gauge board
    # Data and clock pins must be consecutive for this to work
    gauge0 = MitutoyoGauge(smid=0, data=Pin( 2), clock=Pin( 3), button=Pin( 4), nreq=Pin( 5))
    gauge1 = MitutoyoGauge(smid=1, data=Pin(10), clock=Pin(11), button=Pin(12), nreq=Pin(13))
    gauge2 = MitutoyoGauge(smid=2, data=Pin(18), clock=Pin(19), button=Pin(20), nreq=Pin(21))
    gauge3 = MitutoyoGauge(smid=3, data=Pin(26), clock=Pin(27), button=Pin(28), nreq=Pin(22))

    gauge_list = []
    gauge_list.append((gauge0, GaugeState()))
    gauge_list.append((gauge1, GaugeState()))
    # gauge_list.append((gauge2, GaugeState()))
    # gauge_list.append((gauge3, GaugeState()))

    for gauge, state in gauge_list:
        gauge.state = state
        gauge.started_callback = on_started
        gauge.reading_callback = on_reading
        gauge.timeout_callback = on_timeout
        print(f"{gauge} Starting:")
        gauge.pin_print( lambda s: print("    " + s) )
        gauge.activate()


    RUNNING_TIME = const(1)

    # Give time to receive some readings
    utilisation.measure( RUNNING_TIME )

    print(f"{count} readings in {RUNNING_TIME}s with {utilisation}")
    print()

    for gauge, _ in gauge_list:
        print(f"{gauge} Stopping")
        gauge.deactivate()

#endregion


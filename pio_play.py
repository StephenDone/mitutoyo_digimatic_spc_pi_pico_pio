import time
from machine import Pin, Timer
import rp2

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

#region mitutoyo pio code

@rp2.asm_pio(in_shiftdir=rp2.PIO.SHIFT_RIGHT, 
             sideset_init=rp2.PIO.OUT_HIGH,     # Side set for request line, active low
             # fifo_join=rp2.PIO.JOIN_RX,         # Double size RX buffer, no TX buffer
             )
def get_nibbles():

    pull()

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
    in_(x, 4)               # Pad with a zero nibble
    in_(x, 4)               # Pad with a zero nibble

    jmp(pin,"button_up")    # Final nibble is 0, or 1 when data button down
    set(x,1)
    label("button_up")
    in_(x, 4)

    mov(x, isr)             # Save second 32-bit ISR value to x
    mov(isr, y)             # Put first 32-bit y value back in ISR
    push()                  # Push the first 32 bits

    mov(isr, x)             # Restore the first 32-bit value from y
    push()                  # Push the final 32-bit value

    # Not sure if we should block here.
    # irq(block, rel(0))
    irq(rel(0))

#endregion

class mitutoyo_reading:

    @classmethod
    def from_sm( cls, sm ):
        a = sm.get()
        b = sm.get()
        # print( f"{a:08X}" )
        # print( f"{b:08X}" )
        r = cls( a, b )
        # print(r.nibble_string(), "->", r)
        return r

    def reading_error(self, msg):
        raise ValueError(msg)

    def __init__(self, a, b):
        self.a = a
        self.b = b

        self.nibbles = bytearray(16)
        for n in range(8):  # iterate over each nibble
            self.nibbles[n  ] = a & 0xF
            self.nibbles[n+8] = b & 0xF
            a = a >> 4
            b = b >> 4
        
        for n in range(4):
            if self.nibbles[n] != 0xF:
                self.reading_error("Data must begin with 0xFFFF")

        if not self.nibbles[4] in [0, 8]:
            self.reading_error(f"Invalid negative nibble value 0x{self.nibbles[4]:x}")
        self.negative = self.nibbles[4]==8

        self.digits=0
        for digit in range(6):
            digit_value = self.nibbles[digit+5]
            if digit_value > 9:
                self.reading_error(f"Digit[{digit}] out of range: 0x{digit_value:x}")
            self.digits = self.digits*10 + digit_value

        if self.nibbles[11] < 1 or self.nibbles[11] > 5:
            self.reading_error(f"Decimal position out of range: 0x{self.nibbles[11]:x}")

        self.dps = self.nibbles[11]
        self.value = self.digits / (10**self.dps) * (-1 if self.negative else 1)

        if not self.nibbles[12] in [0, 1]:
            self.reading_error(f"Invalid units nibble value 0x{self.nibbles[12]:x}")
        self.inches = self.nibbles[12] == 1
        self.units = 'in' if self.inches else 'mm'

        if self.nibbles[13]:
            self.reading_error(f"Nibble 13 non zero: 0x{self.nibbles[13]:x}")

        if self.nibbles[14]:
            self.reading_error(f"Nibble 14 non zero: 0x{self.nibbles[14]:x}")

        if not self.nibbles[15] in [0, 1]:
            self.reading_error(f"Invalid button value: 0x{self.nibbles[15]:x}")
        self.button_down = self.nibbles[15]==1

        self.reading_error("Test Value Error")

    def __str__(self): 
        return f"value={self.value}{self.units}, button_down={self.button_down}"

    def nibble_string(self): 
        return ''.join('{:01x}'.format(x) for x in self.nibbles)

class mitutoyo_gauge():

    def __init__(self, smid, data, clock, nreq, button):
        self.data = data
        self.clock = clock
        self.nreq = nreq
        self.button = button

        self.data.init(mode=Pin.IN, pull=Pin.PULL_UP)
        self.clock.init(mode=Pin.IN, pull=Pin.PULL_UP)
        self.button.init(mode=Pin.IN, pull=Pin.PULL_UP)
        self.nreq.init(mode=Pin.OPEN_DRAIN)

        self.sm  = rp2.StateMachine(smid, get_nibbles, freq=100000, 
                            in_base=data, 
                            sideset_base=nreq, 
                            jmp_pin=button)
        
        self.sm.irq(self.irq_action)
        self.reading_callback = None
        self.timeout_timer = None
        self.timeout_callback = None
        self.in_sync_call = False

    def set_reading_callback(self, callback):
        self.reading_callback = callback

    def set_timeout_callback(self, callback):
        self.timeout_callback = callback

    def activate(self):
        print("Starting State Machine")
        self.nreq.value(1)
        time.sleep(0.1)
        self.sm.active(1)

    def deactivate(self):
        print("Stopping State Machine")
        self.sm.active(0)
        self.nreq.value(1)

    def get_reading(self):
        self.in_sync_call = True
        try:
            if self.sm.tx_fifo() != 0:
                raise Exception(f"A reading request is already pending")
            if self.sm.rx_fifo() != 0:
                raise Exception("A reading response is already pending")

            self.sm.put(0)

            start_ticks = time.ticks_ms()
            while time.ticks_ms() < start_ticks + 100:
                if self.sm.rx_fifo() >= 2:
                    return mitutoyo_reading.from_sm( self.sm )

            raise Exception("Timed out waiting for reading")
        finally:
            self.in_sync_call = False

    def get_reading_async(self):
        # print("get_reading_async()")
        # print(f"tx_fifo={self.sm.tx_fifo()}")
        if self.sm.tx_fifo() == 0 and self.timeout_timer is None:
            self.sm.put(0)
            self.timeout_timer = Timer(period=100, mode=Timer.ONE_SHOT, callback=self.on_timeout_callback)

    def on_timeout_callback(self, timer):
        self.timeout_timer.deinit()
        self.timeout_timer = None
        self.sm.restart()

        if self.timeout_callback is not None:
            self.timeout_callback(self)

    def irq_action(self, p):

        if self.in_sync_call:
            return

        if self.timeout_timer is not None:
            self.timeout_timer.deinit()
            self.timeout_timer = None

        if self.reading_callback is not None:
            try:
                self.reading_callback(self, mitutoyo_reading.from_sm( self.sm ))
            except ValueError as e:
                print("irq_action:", e)

    def __str__(self):
        return f"gauge on {self.sm}"

    def pin_print(self):
        print(repr(self.data))
        print(repr(self.clock))
        print(repr(self.nreq))
        print(repr(self.button))

class debounced_switch:

    def __init__(self, pin: Pin, button_press_callback):
        self.callback = button_press_callback
        self.pin = pin
        self.pin.init(Pin.IN, Pin.PULL_UP)
        self.pin.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.button_callback)
        self.debouncing = False
        self.debounce_time = 0

    def button_callback(self, pin):
        if self.debouncing:
            self.debouncing = (time.ticks_ms() - self.debounce_time) < 200

        if not self.debouncing:
            falling = pin.irq().flags() & Pin.IRQ_FALLING == Pin.IRQ_FALLING

            if falling and self.callback is not None:
                    self.callback(self.pin)

            self.debouncing=True                
            self.debounce_time=time.ticks_ms()

#region mitutoyo test program

# GPIO 6 - Data           input   IN 0
# GPIO 7 - Clock          input   IN 1
# GPIO 8 - Sample Button  input   IN 2
#          _______
# GPIO 9 - Request        output  SET and SIDESET

#_data        = Pin(6, mode=Pin.IN, pull=Pin.PULL_UP)
#_clock       = Pin(7, mode=Pin.IN, pull=Pin.PULL_UP)
#_data_button = Pin(8, mode=Pin.IN, pull=Pin.PULL_UP)
#_nreq        = Pin(9, mode=Pin.OPEN_DRAIN)

# Implement reading validation - don't raise exception in interrupt handler!

if False:
    button = debounced_switch(Pin(8), lambda pin : print (f"Button Press on {pin}"))
    time.sleep(10)
    exit()

if True:
    count = 0

    def on_reading(gauge, reading):
        global count

        print(gauge, reading)
        count = count +1
        gauge.get_reading_async()

    def on_timeout(gauge):
        print("Timeout")
        gauge.get_reading_async()

    gauge = mitutoyo_gauge(smid=0, data=Pin(6), clock=Pin(7), nreq=Pin(9), button=Pin(8))

    if False:
        gauge.set_reading_callback( on_reading )
        gauge.set_timeout_callback( on_timeout )
        gauge.activate()

        start_time = time.ticks_ms()
        gauge.get_reading_async()

        # Give time to receive some interrupts
        time.sleep(1)

        time_interval = time.ticks_ms() - start_time
        print(f"{count} readings in {time_interval}ms")

        gauge.deactivate()

    if True:
        gauge.activate()

        reading = gauge.get_reading()
        print( reading )

        gauge.deactivate()

#endregion


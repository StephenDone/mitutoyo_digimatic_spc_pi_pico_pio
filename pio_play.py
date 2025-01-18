# Example using PIO to blink an LED and raise an IRQ at 1Hz.

import time
from machine import Pin
import rp2


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

# GPIO 6 - Data           input   IN 0
# GPIO 7 - Clock          input   IN 1
# GPIO 8 - Sample Button  input   IN 2
#          _______
# GPIO 9 - Request        output  SET and SIDESET

@rp2.asm_pio(in_shiftdir=rp2.PIO.SHIFT_RIGHT, push_thresh=32) #set_init=rp2.PIO.OUT_HIGH, sideset_init=rp2.PIO.OUT_HIGH, in_shiftdir=rp2.PIO.SHIFT_LEFT)
def get_nibble():

    set(y, 12)

    label("nibble_loop")

    set(x, 3)

    label("bitloop")

    wait(0, pin, 1)     # Wait for clock to go low
    in_(pins, 1)        # Read in 1 data bit
    wait(1, pin, 1)     # Wait for clock to go high

    jmp(x_dec, "bitloop")

    push(iffull)

    jmp(y_dec, "nibble_loop")

    set(x, 0x12345678) # Only up to 31 - 0x1F !!!
    in_(x, 12)

    push()

i=0

def irq_action(p):
    global i
    global sm

    i=i+1
    print(f"ticks: {time.ticks_ms()}, i={i}, flags={p.irq().flags()}")
    #print( f"i={i}" )
    #print( p )
    print( f"{sm.get():08X}" )


# Create the StateMachine with the blink_1hz program, outputting on Pin(25).
#sm = rp2.StateMachine(0, blink_1hz, freq=2000, set_base=Pin(25))
#sm = rp2.StateMachine(0, rx_fifo, freq=2000, set_base=Pin(25))

# Set the IRQ handler to print the millisecond timestamp.
#sm.irq(irq_action)

data = Pin(6, mode=Pin.IN, pull=Pin.PULL_UP)
clock = Pin(7, mode=Pin.IN, pull=Pin.PULL_UP)
pin_ready = Pin(8, mode=Pin.IN, pull=Pin.PULL_UP)
nreq = Pin(9, mode=Pin.OUT)

#sm = rp2.StateMachine(0, get_nibble, freq=100000, in_base=Pin(6), set_base=Pin(9), sideset_base=Pin(9))
sm = rp2.StateMachine(0, get_nibble, freq=100000, in_base=data)
#sm = rp2.StateMachine(0, rx_fifo, freq=2000, set_base=Pin(25))

# Start the StateMachine.
print("Starting")

sm.active(1)
nreq.value(False)

# time.sleep(1)

for x in range(13):
    print( f"{sm.get(None, 0):08X}" )

sm.active(0)

nreq.value(True)
print("Stopping")
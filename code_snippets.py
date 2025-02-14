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

import time
import uasyncio
import ir_rcv_pio
from machine import Pin, PWM

IR_DETECT_PIN = 4
led = Pin('LED', Pin.OUT)  # create LED

ledn = Pin(16, Pin.OUT)
ledn.high()  # active low
ledp = PWM(Pin(17))
ledp.freq(38022)
ledp.duty_u16(0)

usBIT0 = 562
usBIT1 = usBIT0 * 3
usREPEAT = usBIT0 * 4
usMSG = usREPEAT * 2
usSTART = usMSG * 2


async def ir_detect():
    msglbls = 'addr cmd cnt'.split()
    while True:
        # print("Waiting")
        while not ir_rcv_pio.is_ready():
            await uasyncio.sleep_ms(50)
        m = await ir_rcv_pio.get_msg()
        if m is ir_rcv_pio.EMPTY_MSG:
            print('E', end='')
        elif m == ir_rcv_pio.INVALID_MSG:
            print('X.', end='')
        else:
            # ignore instant repeats
            if 0 < m[2] < 4:
                continue
            hexd = ['0x{:02x}'.format(x) for x in m[:2]] + [str(m[2])]
            print('Message:', ' '.join(['{}:{}'.format(*z) for z in zip(msglbls, hexd)]))


async def blinky(iters):
    for n in range(iters):
        led.value(1)    # Set led turn on
        await uasyncio.sleep_ms(500)
        led.value(0)    # Set led turn off
        await uasyncio.sleep_ms(500)


def send_ir_msg(addr, cmd):
    """Must calculate all the times, then do the GPIO manipulation
       so our timing stays within spec"""

    def byte_values(b8):
        # get bits: force leading zeros; send LSB first
        bs = reversed(bin(b8 + 0x100)[-8:])
        return [[usBIT0, usBIT1 if b == '1' else usBIT0] for b in bs]

    # calculate a list of times for GPIO changes
    times = [[usSTART, usMSG]]
    for v in (addr, cmd):
        times += byte_values(v)
        times += byte_values(~v)
    times += [[usBIT0, 0]]
    # enable signal carrier
    ledp.duty_u16(0xFFFF // 2)
    for on, off in times:
        ledn.low()  # enables 38K signal to LED
        time.sleep_us(on - 5)  # adjusted for overhead
        ledn.high()  # disables 38K signal to LED
        time.sleep_us(off - 5)
    # turn off noisy signal
    ledp.duty_u16(0)


async def send_ir():

    msgs = [[1, 2], [0xaf, 0x99]]

    while True:
        for a, c in msgs:
            await blinky(2)
            print('Sending:', hex(a), hex(c))
            send_ir_msg(a, c)

# start the IR detection
ir_rcv_pio.start(IR_DETECT_PIN, 2)
# kick off 2 threads: IR detect and IR send
print(uasyncio.run(uasyncio.gather(ir_detect(), send_ir())))

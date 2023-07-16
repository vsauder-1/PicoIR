"""Use PIO mode to decode NEC IR codes
https://techdocs.altium.com/display/FPGA/NEC+Infrared+Transmission+Protocol
"""

from machine import Pin as _Pin
import rp2
import time
from micropython import const as _const
import uctypes
import uasyncio
import _thread


# GPIO control register (RP2040)
_iobase = 0x40014000
_io_bank = {'status': uctypes.UINT32 | 0,
            'ctrl': uctypes.UINT32 | 4}


@rp2.asm_pio(fifo_join=rp2.PIO.JOIN_RX, autopush=True)
def _read_ir():
    """Measure hi and low times of pulses on GPIO"""
    # output start sequence (0xFFFFFFFF, 0)
    set(x, 0)
    jmp(x_dec, 'hi_start')
    in_(x, 32)
    set(x, 0)
    in_(x, 32)
    wait(0, pin, 0)

    # wait for and measure high pulse
    label('hi_start')
    wait(1, pin, 0)
    set(x, 0)
    mov(x, invert(x))
    label('hi_count')
    jmp(x_dec, 'next_hi')
    jmp('fail')   # cycled / timeout
    label('next_hi')
    jmp(pin, 'hi_count').delay(1)  # delay to match low code timing
    mov(x, invert(x))
    in_(x, 32)

    # wait(0, pin, 0)  # already low
    set(x, 0)
    mov(x, invert(x))
    label('lo_count')
    jmp(x_dec, 'next_lo')
    jmp('fail')     # cycled / timeout
    label('next_lo')
    jmp(pin, 'push_lo')
    jmp('lo_count')
    label('push_lo')
    mov(x, invert(x))
    in_(x, 32)
    jmp('hi_start')

    label('fail')  # just restart


_intr_q = []
_intr_evt = uasyncio.ThreadSafeFlag()


_FAIL = _const(0xffffffff)
_START_SEQ = (_FAIL, 0)

# protocol pulse times in microseconds
_BIT_LEAD = _const(562.5)
_BIT_VAL_0 = _BIT_LEAD
_BIT_VAL_1 = _BIT_LEAD * 3  # 1688
_REPEAT = _BIT_LEAD * 4  # 2250
_MSG_VAL = _REPEAT * 2  # 4500
_START_VAL = _MSG_VAL * 2  # 9000


def _pulse_iter(p):
    while True:
        yield p
        p += 1
        p %= 2


def _get1(sm, q, evt):
    """ISR to pull from PIO queue to Python queue"""
    piter = _pulse_iter(1)
    while True:
        bt = sm.get()
        if bt == _FAIL:
            piter = _pulse_iter(1)
        else:
            bt *= 10  # convert value to microseconds
        q.append((bt, next(piter)))
        evt.set()


async def _wait_x(n):
    """Wait for more pulse data"""
    while len(_intr_q) < n:
        await _intr_evt.wait()


def _valeq(t, v):
    return t * .7 < v < t * 1.3


async def _get_vals(x, p=0):
    vals = []
    piter = _pulse_iter(p)
    await _wait_x(x)
    for _ in range(x):
        v, h = _intr_q.pop(0)
        assert next(piter) == h, f'Unexpected pulse direction: {h}'
        vals.append(v)
    # print('vals:', vals)
    return vals


async def _get_val(hi):
    return (await _get_vals(1, hi))[0]


async def _get_bit():
    x, b = await _get_vals(2, 1)
    assert _valeq(_BIT_LEAD, x), f'Invalid bit lead: {x}'
    v = _valeq(_BIT_VAL_1, b)
    if v:
        return 1
    v = _valeq(_BIT_VAL_0, b)
    assert v, f'Invalid bit: {b}'
    return 0


async def _get_byte():
    v = 0
    for x in range(8):
        v |= await _get_bit() << x
    return v


def _wait_for_start():
    dropped = []
    found = False
    while _intr_q:
        b, hi = _intr_q[0]
        if hi and _valeq(_START_VAL, b):
            found = True
            break
        d = _intr_q.pop(0)
        if not hi and not dropped and b > _START_VAL * 2:
            # inter-code space
            # print('Interspace')
            continue
        dropped.append(d)
    # if dropped:
    #     print('Dropped values:', dropped)
    return found


def is_ready():
    return _wait_for_start()


INVALID_MSG = 0
EMPTY_MSG = None
last_msg = INVALID_MSG


async def get_msg():
    """Get next decoded message: {addr, cmd, repeats} (or INVALID_MSG/None on error)
        blocking call if not is_ready()"""
    global last_msg
    msg = INVALID_MSG
    try:
        while True:
            if _wait_for_start():
                break
            await _wait_x(2)
        b = await _get_val(1)
        assert _valeq(_START_VAL, b), f'Invalid START mark: {b}'
        # we have a valid start mark so we should get a complete message
        b = await _get_val(0)
        # print('Code:', b)
        if _valeq(_MSG_VAL, b):
            last_msg = INVALID_MSG
            vals = []
            for _ in range(4):
                vals.append(await _get_byte())
            assert vals[0] == (~vals[1] & 0xFF), f'Invalid msg addr: {vals[:2]}'
            assert vals[2] == (~vals[3] & 0xFF), f'Invalid msg cmd: {vals[2:]}'
            # print('Message:', vals[0], vals[2])
            msg = vals[0], vals[2], 0
            last_msg = list(msg)
        elif _valeq(_REPEAT, b):
            if last_msg:
                last_msg[2] += 1
                msg = tuple(last_msg)
            # print('REPEAT')
        else:
            print('Invalid code', b)
        s = await _get_val(1)
        if not _valeq(_BIT_LEAD, s):
            print('Invalid stop code:', s)
    except Exception as exc:
        print(exc)
        msg = INVALID_MSG
    return msg


def clear():
    _intr_q.clear()


# start everything up
def start(pin, state_mach=0):
    _in_pin = _Pin(pin, _Pin.IN)
    # clock frequency is selected so each count is in tens of microseconds
    _smx = rp2.StateMachine(state_mach, _read_ir, freq=300_000, in_base=_in_pin, jmp_pin=_in_pin)
    _smx.active(1)
    _thread.start_new_thread(_get1, (_smx, _intr_q, _intr_evt))

    # invert the Input pin once the state machine is started
    _gpio_ctrl = _iobase + (pin * 8)
    _mem = uctypes.struct(_gpio_ctrl, _io_bank)
    # print('status:', hex(mem.status), ' ctrl:', hex(mem.ctrl))
    _ctrl = _mem.ctrl & ~(0xf << 16)
    _mem.ctrl = _ctrl | (1 << 16)
    # print('status:', hex(mem.status), ' ctrl:', hex(mem.ctrl))

    time.sleep(0.1)  # make sure it starts and has the start sequence in the queue
    seq = tuple([v[0] for v in _intr_q[:2]])
    assert seq == _START_SEQ, f'Invalid start: {seq} <> {_START_SEQ}'
    del _intr_q[:2]


if __name__ == '__main__':

    async def dbg():
        while True:
            # print("Waiting")
            while not is_ready():
                await uasyncio.sleep_ms(50)
            m = await get_msg()
            if m == INVALID_MSG:
                print('X', end='')
            else:
                print('\nMessage:', m)

    print('starting loop')
    start(4)
    uasyncio.get_event_loop().run_until_complete(dbg())

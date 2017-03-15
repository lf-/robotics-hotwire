import sdl2
import sys
import time
import ctypes
import serial
import struct


LOG_FILE = 'hotwireprogram.c'
MD49_MODE = 0x01

def saber_generate_packet(addr, cmd, data) -> bytes:
    """
    Generate a 4-byte packet conforming to the packetized serial protocol.
    Generates the checksum for you.

    Parameters:
    addr -- address byte, > 128
    cmd -- command byte, < 128
    data -- data byte, < 128

    Returns:
    A 4-byte bytes object suitable for sending to a sabertooth
    """
    checksum = ((addr + cmd + data) % 256) & 0b01111111
    return bytes((addr, cmd, data, checksum))


class SerialConnection:
    def __init__(self, port='/dev/ttyACM0'):
        self.port = port
        self.ser = serial.Serial(self.port, 9600)

    def md49_set_speed(self, motor, speed):
        speed = ctypes.c_ubyte(speed).value
        motor_cmds = {1: 0x31, 2: 0x32}
        cmd = bytes((0x00, motor_cmds[motor], speed))
        self.call(MD49_MODE, cmd)

    def md49_get_encoders(self):
        self.ser.reset_input_buffer()
        self.call(MD49_MODE, (0x00, 0x25,))
        e1raw = self.ser.read(4)
        e2raw = self.ser.read(4)
        #print([hex(x) for x in e1raw], [hex(x) for x in e2raw])
        encoder1 = struct.unpack('>i', e1raw)[0]
        encoder2 = struct.unpack('>i', e2raw)[0]
        return encoder1, encoder2

    def call(self, mode, cmd):
        command = []
        for byte in cmd:
            command.extend((mode, byte))
        #print([hex(x) for x in command])
        self.ser.write(bytes(command))
        self.ser.flush()
        #resp = self.ser.read_all()
        #if resp:
            #print(resp)
        #    print([hex(x) for x in resp])


def log_encoders(sc: SerialConnection):
    left, right = sc.md49_get_encoders()
    line = '{{{}, {}}},'.format(left, right)
    with open(LOG_FILE, 'a') as log:
        log.write(line)


def translate(value, leftMin, leftMax, rightMin, rightMax):
    """
    Map one range of values to another.

    Parameters:
    value -- the number, between leftMin and leftMax to map into
             rightMin to rightMax
    leftMin -- bottom of range of value parameter
    leftMax -- top of range of value parameter
    rightMin -- bottom of range of output
    rightMax -- top of range of output
    """
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)


def init():
    sdl2.SDL_Init(sdl2.SDL_INIT_JOYSTICK | sdl2.SDL_INIT_HAPTIC | sdl2.SDL_INIT_EVENTS)


class ControllerState:
    ind = False
    axis_state = [0, 0, 0, 0, 0, 0]
    l_rev = False
    r_rev = False


def handle_axis_motion(conn, state, axis, value):
    scaled_l = translate(state.axis_state[5], -32768, 32768, 0, 1) ** 3
    scaled_r = translate(state.axis_state[2], -32768, 32768, 0, 1) ** 3
    if state.ind:
        throttle_l = translate(scaled_l, 0, 1, 0, 127)
        throttle_r = translate(scaled_r, 0, 1, 0, 127)
    else:
        throttle = translate(scaled_l - scaled_r, -1, 1, -128, 127)
        throttle_l = throttle
        throttle_r = throttle
    throttle_l = -throttle_l if state.l_rev else throttle_l
    throttle_r = -throttle_r if state.r_rev else throttle_r
    conn.md49_set_speed(1, round(throttle_l))
    conn.md49_set_speed(2, round(throttle_r))
    #print(throttle_l, throttle_r)


def main():
    init()
    joy = sdl2.SDL_JoystickOpen(0)
    print('Name: {}'.format(sdl2.SDL_JoystickName(joy)))
    print('Axes: {}'.format(sdl2.SDL_JoystickNumAxes(joy)))

    state = ControllerState()
    evt = sdl2.SDL_Event()
    running = True
    conn = SerialConnection(sys.argv[1] if len(sys.argv) >= 2 else '/dev/ttyACM0')

    # set to mode 1 so it will not attack
    conn.call(MD49_MODE, (0x00, 0x34, 0x01))

    while running:
        while sdl2.SDL_PollEvent(ctypes.byref(evt)) != 0:
            if evt.type == sdl2.SDL_QUIT:
                running = False
                break
            elif evt.type == sdl2.SDL_JOYAXISMOTION:
                jaxis = evt.jaxis
                # print(state, jaxis.axis, jaxis.value)
                state.axis_state[jaxis.axis] = jaxis.value
                handle_axis_motion(conn, state, jaxis.axis, jaxis.value)
            elif evt.type == sdl2.SDL_JOYBUTTONUP:
                button = evt.jbutton
                if button.button == 0:
                    # A: toggle independent
                    state.ind = not state.ind
                    print('Independent: {}'.format(state.ind))
                elif button.button == 8:
                    # B: log encoders
                    print(conn.md49_get_encoders())
                    log_encoders(conn)
                elif button.button == 3:
                    # Y: reset encoders
                    conn.call(MD49_MODE, (0x00, 0x35,))
                elif button.button == 4:
                    # left shoulder: reverse
                    state.r_rev = not state.r_rev
                    print('Right reverse: {}'.format(state.r_rev))
                elif button.button == 5:
                    # right shoulder: reverse
                    state.l_rev = not state.l_rev
                    print('Left reverse: {}'.format(state.l_rev))
                else:
                    print('Button', button.button)
        time.sleep(0.01)
    sdl2.SDL_Quit()


if __name__ == '__main__':
    main()

from time import sleep
from threading import Thread
from gpiozero import Robot
import RPi.GPIO as GPIO


class Encoder:
    def __init__(self, pin, disc_slots=20):
        self._value = 0
        self.disc_slots_x_2 = 2 * disc_slots

        GPIO.setup(pin, GPIO.IN)
        GPIO.add_event_detect(pin, GPIO.BOTH, callback=self._increment)

    def reset(self):
        self._value = 0

    def _increment(self, callback):
        self._value += 1

    @property
    def value(self):
        return self._value

    @property
    def rotations(self):
        return self._value / self.disc_slots_x_2


class RobotTwoWheeled:
    command_dict = {'w': 'forward',
                    's': 'backward',
                    'a': 'left',
                    'd': 'right',
                    'f': 'stop',
                    'z': 'decrease_speed',
                    'x': 'increase_speed',
                    'c': 'decrease_speed_pid',
                    'v': 'increase_speed_pid'}

    states_dict = {0: 'Stop',
                   1: 'Forward',
                   2: 'Backward',
                   3: 'Left',
                   4: 'Right'}

    direction = {1: (1, 1),
                 2: (-1, -1),
                 3: (-1, 1),
                 4: (1, -1)}

    def __init__(self, left, right, encoders=None, speed=0.5):

        self._t = Thread()
        self._kill_t = False

        self.motors = Robot(left, right)
        self._speed = speed

        self._wheel_size = 0.06

        if encoders is not None:
            self._state = 0

            # PID variables
            self._KP = 0.0125
            self._KD = 0.0001
            self._KI = 0.00005
            self.SAMPLE_INTERVAL = 0.1

            self._steps_l = 0
            self._steps_r = 0

            self._speed_l = 0
            self._speed_r = 0
            self._prev_error_l = 0
            self._prev_error_r = 0
            self._error_sum_l = 0
            self._error_sum_r = 0

            self._max_encoder_speed = 200
            self._speed_pid = round(self._max_encoder_speed * self.SAMPLE_INTERVAL * self._speed)
            self._interrupts_enabled = True
            self._interrupts_available = True
            self._encoder_left = Encoder(encoders[0])
            self._encoder_right = Encoder(encoders[1])
        else:
            self._interrupts_available = False

    @property
    def interrupts_enabled(self):
        return self._interrupts_enabled

    @interrupts_enabled.setter
    def interrupts_enabled(self, value):
        if self._interrupts_available:
            self._interrupts_enabled = value
        else:
            raise AttributeError

    @property
    def interrupts_available(self):
        return self._interrupts_available

    @property
    def pid(self):
        if self._interrupts_available:
            return self._KP, self._KI, self._KD
        else:
            return None

    @pid.setter
    def pid(self, kpid):
        if self._interrupts_available:
            self._KP = kpid[0] if kpid[0] is not None else self._KP
            self._KI = kpid[1] if kpid[1] is not None else self._KI
            self._KD = kpid[2] if kpid[2] is not None else self._KD
        else:
            raise AttributeError

    @property
    def max_encoder_speed(self):
        if self.interrupts_available:
            return self._max_encoder_speed
        else:
            raise AttributeError

    @max_encoder_speed.setter
    def max_encoder_speed(self, value):
        if value > 0 and self._interrupts_available:
            self._speed_pid = round(self._speed_pid * value / self._max_encoder_speed)
            self._speed = self._speed_pid / (value * self.SAMPLE_INTERVAL)
            self._max_encoder_speed = value
        else:
            print('Value should be greater than 0')
            raise ValueError

    @property
    def speed(self):
        return self._speed

    @speed.setter
    def speed(self, value):
        if 0.0 <= value <= 1.0:
            self._speed = value
            if self._interrupts_available:
                self._speed_pid = round(self._max_encoder_speed * self.SAMPLE_INTERVAL * value)
        else:
            print('Value range is 0-1')
            raise ValueError

    @property
    def speed_pid(self):
        try:
            return self._speed_pid
        except Exception as e:
            raise e

    @speed_pid.setter
    def speed_pid(self, value):
        if 0 <= value <= self._max_encoder_speed * self.SAMPLE_INTERVAL and self._interrupts_available:
            self._speed_pid = value
            self._speed = value / (self._max_encoder_speed * self.SAMPLE_INTERVAL)
        else:
            try:
                print('Value range is 0-{}'.format(round(self._max_encoder_speed * self.SAMPLE_INTERVAL)))
                raise ValueError
            except Exception as e:
                raise e

    @property
    def state(self):
        return self._state

    @property
    def wheel_size(self):
        return self._wheel_size

    @wheel_size.setter
    def wheel_size(self, m):
        self._wheel_size = m

    @property
    def steps(self):
        return self._steps_l, self._steps_r

    def reset_steps(self):
        if self.interrupts_available:
            self._steps_l = 0
            self._steps_r = 0
        else:
            raise AttributeError

    def detect_max_encoder_speed(self, sample_time=5):
        if self._interrupts_available:
            self.motors.forward()
            speeds = []

            for _ in range(sample_time):
                self._encoder_left.reset()
                self._encoder_right.reset()
                sleep(1)
                speeds.append(self._encoder_left.value)
                speeds.append(self._encoder_right.value)

            self.motors.stop()
            self._encoder_left.reset()
            self._encoder_right.reset()

            return min(speeds)

        else:
            raise AttributeError

    def _thread(self, func, arg):
        while True:
            if self._kill_t:
                break
            func(*arg)

    def _start_thread(self, func, args=()):
        self._kill_thread()
        self._t = Thread(target=self._thread, args=[func, args, ])
        self._t.start()

    def _kill_thread(self):
        try:
            if self._t.is_alive():
                self._kill_t = True
                self._t.join()
                self._kill_t = False
        except:
            pass

        self.motors.stop()
        self._speed_l = 0
        self._speed_r = 0
        self._prev_error_l = 0
        self._prev_error_r = 0
        self._error_sum_l = 0
        self._error_sum_r = 0
        self._encoder_left.reset()
        self._encoder_right.reset()

    def _equal_rotation_pid(self, direction):
        ld, rd = self.direction[direction]
        error_l = self._speed_pid - self._encoder_left.value
        error_r = self._speed_pid - self._encoder_right.value
        self._speed_l += (error_l * self._KP) + (self._prev_error_l * self._KD) + (self._error_sum_l * self._KI)
        self._speed_r += (error_r * self._KP) + (self._prev_error_r * self._KD) + (self._error_sum_r * self._KI)

        self._speed_l = min(max(self._speed_l, 0), 1)
        self._speed_r = min(max(self._speed_r, 0), 1)

        self.motors.value = (ld * self._speed_l, rd * self._speed_r)

        self._encoder_left.reset()
        self._encoder_right.reset()

        self._prev_error_l = error_l
        self._prev_error_r = error_r
        self._error_sum_l += error_l
        self._error_sum_r += error_r

        sleep(self.SAMPLE_INTERVAL)

    def move_steps(self, rot_l, rot_r, direction):
        self._start_thread(self._move_steps_pid, (rot_l, rot_r, direction))
        self._kill_thread()

    def _move_steps_pid(self, rot_l, rot_r, direction):
        ld, rd = self.direction[direction]

        if rot_l <= rot_r:
            spl = round(rot_l / rot_r * self._speed_pid)
            spr = self._speed_pid
        else:
            spl = self._speed_pid
            spr = round(rot_r / rot_l * self._speed_pid)

        pel = 0
        per = 0

        while self._encoder_left.rotations < rot_l or self._encoder_right.rotations < rot_r:
            cel = self._encoder_left.value
            cer = self._encoder_right.value

            error_l = spl - cel + pel
            error_r = spr - cer + per

            self._speed_l += (error_l * self._KP) + (self._prev_error_l * self._KD) + (self._error_sum_l * self._KI)
            self._speed_r += (error_r * self._KP) + (self._prev_error_r * self._KD) + (self._error_sum_r * self._KI)

            self._speed_l = min(max(self._speed_l, 0), 1)
            self._speed_r = min(max(self._speed_r, 0), 1)

            self.motors.value = (ld * self._speed_l, rd * self._speed_r)

            pel = cel
            per = cer

            self._prev_error_l = error_l
            self._prev_error_r = error_r
            self._error_sum_l += error_l
            self._error_sum_r += error_r

            sleep(self.SAMPLE_INTERVAL)

    def forward(self, speed=None):
        self._state = 1
        if self._interrupts_enabled:
            self._start_thread(self._equal_rotation_pid, (self._state,))

        else:
            speed = self._speed if speed is None else speed
            self.motors.value = (speed, speed)

    def backward(self, speed=None):
        self._state = 2
        if self._interrupts_enabled:
            self._start_thread(self._equal_rotation_pid, (self._state,))

        else:
            speed = self._speed if speed is None else speed
            self.motors.value = (-speed, -speed)

    def left(self, speed=None):
        self._state = 3
        if self._interrupts_enabled:
            self._start_thread(self._equal_rotation_pid, (self._state,))

        else:
            speed = self._speed if speed is None else speed
            self.motors.value = (-speed, speed)

    def right(self, speed=None):
        self._state = 4
        if self._interrupts_enabled:
            self._start_thread(self._equal_rotation_pid, (self._state,))

        else:
            speed = self._speed if speed is None else speed
            self.motors.value = (speed, -speed)

    def stop(self):
        self._state = 0
        if self._interrupts_enabled:
            self._kill_thread()

        else:
            self.motors.stop()

    def increase_speed(self):
        self.speed = min(round(self.speed + 0.01, 2), 1)
        return self.speed

    def decrease_speed(self):
        self.speed = max(round(self.speed - 0.01, 2), 0)
        return self.speed

    def increase_speed_pid(self):
        if self._interrupts_available:
            self.speed_pid = min(self.speed_pid + 1, round(self.max_encoder_speed * self.SAMPLE_INTERVAL))
            return self.speed_pid
        else:
            raise AttributeError

    def decrease_speed_pid(self):
        if self._interrupts_available:
            self.speed_pid = max(self.speed_pid - 1, 0)
            return self.speed_pid
        else:
            raise AttributeError

    def command(self, c):
        try:
            args = c.split() + ['']
            return eval('self.{}({})'.format(self.command_dict[args[0]], args[1]))
        except:
            pass


class LineArraySensor:

    def __init__(self, *pins, black_on_white=True):
        self.pins = {}
        self.len = len(pins)
        self.black_on_white = black_on_white

        for i, pin in enumerate(pins):
            GPIO.setup(pin, GPIO.IN)
            self.pins[i] = pin

    def bit(self, i):
        return GPIO.input(self.pins[i]) ^ self.black_on_white

    def byte(self):
        byte = 0

        for i in range(self.len):
            byte <<= 1
            byte += GPIO.input(self.pins[i]) ^ self.black_on_white

        return byte

    def byte_str(self):
        return str(bin(self.byte()))[2:].rjust(self.len, '0')

    def turn_bias(self):
        left_bias = 0
        right_bias = 0
        nl = 0
        nr = 0

        hl = self.len // 2

        for i in range(hl):
            x = GPIO.input(self.pins[i]) ^ self.black_on_white
            nl += x
            left_bias += x * (hl - i)

        for i in range(self.len - 1, hl - 1 if not self.len % 2 else hl, -1):
            x = GPIO.input(self.pins[i]) ^ self.black_on_white
            nr += x
            right_bias += x * (i - hl + 1)

        if not nl:
            nl = 1
        if not nr:
            nr = 1
        return left_bias / nl, right_bias / nr


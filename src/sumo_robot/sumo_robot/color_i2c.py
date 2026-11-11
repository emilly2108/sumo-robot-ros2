"""Reusable Linux I2C requests for the three TCS34725 sensors.

The mux selection is a separate transaction so its STOP takes effect before
the sensor read. Buffers and ctypes request objects are allocated once, not
rebuilt by Adafruit_PureIO for every sample.
"""
import ctypes
import fcntl


I2C_RDWR = 0x0707
I2C_M_RD = 0x0001


class _Message(ctypes.Structure):
    _fields_ = [('addr', ctypes.c_uint16), ('flags', ctypes.c_uint16),
                ('length', ctypes.c_uint16), ('buffer', ctypes.POINTER(ctypes.c_uint8))]


class _Transfer(ctypes.Structure):
    _fields_ = [('messages', ctypes.POINTER(_Message)), ('count', ctypes.c_uint32)]


class LinuxRGBCReader:
    """Read C/R/G/B atomically, with bounded locking and exception-safe cleanup."""

    def __init__(self, bus, fd, device_address, mux_address, channel, ioctl=None):
        if not 0 <= channel <= 7:
            raise ValueError('TCA channel must be 0..7')
        self.bus = bus
        self.fd = fd
        self.ioctl = ioctl or fcntl.ioctl
        self._select_buffer = (ctypes.c_uint8 * 1)(1 << channel)
        self._disable_buffer = (ctypes.c_uint8 * 1)(0)
        # CMD=1, TYPE=01 auto-increment, CDATAL=0x14.
        self._command_buffer = (ctypes.c_uint8 * 1)(0xB4)
        self._data_buffer = (ctypes.c_uint8 * 8)()
        self._select_messages = (_Message * 1)(
            _Message(mux_address, 0, 1, self._select_buffer))
        self._disable_messages = (_Message * 1)(
            _Message(mux_address, 0, 1, self._disable_buffer))
        self._read_messages = (_Message * 2)(
            _Message(device_address, 0, 1, self._command_buffer),
            _Message(device_address, I2C_M_RD, 8, self._data_buffer))
        self._select = _Transfer(self._select_messages, 1)
        self._read = _Transfer(self._read_messages, 2)
        self._disable = _Transfer(self._disable_messages, 1)

    @classmethod
    def from_sensor(cls, sensor):
        device = sensor._device
        channel = device.i2c
        bus = channel.tca.i2c
        try:
            # The installed Blinka generic_linux backend owns this open fd.
            fd = bus._bus._i2c._i2c_bus._device.fileno()
        except AttributeError as exc:
            raise RuntimeError('Reusable color I2C reads require Blinka generic_linux') from exc
        return cls(bus, fd, device.device_address, channel.tca.address,
                   channel.channel_switch[0].bit_length() - 1)

    def read_rgbc(self):
        # BoundedI2C.try_lock raises on its 10ms deadline.
        if not self.bus.try_lock():
            raise TimeoutError('Color I2C bus lock unavailable')
        primary_error = None
        try:
            self._execute(self._select)
            self._execute(self._read)
            data = self._data_buffer
            c = data[0] | data[1] << 8
            r = data[2] | data[3] << 8
            g = data[4] | data[5] << 8
            b = data[6] | data[7] << 8
            return r, g, b, c
        except BaseException as exc:
            primary_error = exc
            raise
        finally:
            try:
                self._execute(self._disable)
            except Exception as cleanup_error:
                if primary_error is None:
                    raise
                primary_error.add_note('Mux disable also failed: ' + str(cleanup_error))
            finally:
                self.bus.unlock()

    def _execute(self, request):
        transferred = self.ioctl(self.fd, I2C_RDWR, request)
        if transferred != request.count:
            raise OSError(f'Incomplete color I2C transfer: {transferred}/{request.count} messages')

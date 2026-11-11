"""Bounded I2C locking and exception-safe TCA channel selection.

The lock deadline does not cancel a blocking kernel I2C ioctl. The OS driver
still owns the transfer timeout; initialization checks its budget between calls.
"""
import threading
import time


I2C_LOCK_TIMEOUT_SEC = 0.01
I2C_INITIALIZATION_TIMEOUT_SEC = 8.0


class BoundedI2C:
    def __init__(self, bus, lock_timeout=I2C_LOCK_TIMEOUT_SEC):
        if lock_timeout <= 0:
            raise ValueError("I2C lock timeout must be positive")
        self._bus = bus
        self._timeout = lock_timeout
        self._owner = None

    def try_lock(self):
        owner = threading.get_ident()
        if self._owner == owner:
            raise RuntimeError("Recursive I2C lock acquisition")
        deadline = time.monotonic() + self._timeout
        while True:
            if self._bus.try_lock():
                self._owner = owner
                return True
            if time.monotonic() >= deadline:
                raise TimeoutError(f"I2C bus lock unavailable for {self._timeout*1000:.0f} ms")
            time.sleep(0.0001)

    def unlock(self):
        if self._owner != threading.get_ident():
            raise RuntimeError("Cannot release an I2C lock owned by another caller")
        try:
            self._bus.unlock()
        finally:
            self._owner = None

    def __getattr__(self, name):
        return getattr(self._bus, name)


class BoundedTCAChannel:
    """I2CDevice-compatible channel; never uses TCA's unbounded lock loop."""
    def __init__(self, tca, channel):
        if not 0 <= channel <= 7:
            raise ValueError("TCA channel must be 0..7")
        self.tca = tca
        self.channel_switch = bytes((1 << channel,))

    def try_lock(self):
        self.tca.i2c.try_lock()
        try:
            self.tca.i2c.writeto(self.tca.address, self.channel_switch)
        except BaseException:
            self.tca.i2c.unlock()
            raise
        return True

    def unlock(self):
        try:
            self.tca.i2c.writeto(self.tca.address, b"\x00")
        finally:
            self.tca.i2c.unlock()

    def writeto(self, *args, **kwargs):
        return self.tca.i2c.writeto(*args, **kwargs)

    def readfrom_into(self, *args, **kwargs):
        return self.tca.i2c.readfrom_into(*args, **kwargs)

    def writeto_then_readfrom(self, *args, **kwargs):
        return self.tca.i2c.writeto_then_readfrom(*args, **kwargs)


def check_initialization_deadline(deadline):
    if time.monotonic() >= deadline:
        raise TimeoutError("Color initialization exceeded the 8 s retry budget")

import time
import serial

# Encoder counts per revolution (ELL14 spec)
COUNTS_PER_REVOLUTION = 143360

RESPONSES = [
    'ok',
    'communication timeout',
    'mechanical timeout',
    'command error',
    'value out of range',
    'module isolated',
    'module out of isolation',
    'initialization error',
    'thermal error',
    'busy',
    'sensor error',
    'motor error',
    'out of range',
    'overcurrent',
]


def from_twos_complement(n, bits=32):
    if n < (1 << (bits - 1)):
        return n
    return n - (1 << bits)


def to_twos_complement(n, bits=32):
    return (1 << bits) + n if n < 0 else n


class ElliptecRotationStage:
    def __init__(
        self,
        port,
        address: int = 0,
        offset: int = 0,
        timeout: float = 1.0,
    ):
        self.address = address
        self._offset = offset
        self._timeout = timeout

        self._conn = serial.Serial(
            port=port,
            baudrate=9600,
            stopbits=1,
            parity='N',
            timeout=0.05,
        )

    # ---------- low-level I/O ----------

    def send(self, command, data=b''):
        packet = (
            str(self.address).encode()
            + command.encode()
            + data.hex().upper().encode()
            + b'\n'
        )
        self._conn.write(packet)

    def query(self, command, data=b''):
        self.send(command, data)

        response = b''
        start = time.time()
        while True:
            response += self._conn.read(8192)
            if response.endswith(b'\r\n'):
                break
            if time.time() - start > self._timeout:
                raise TimeoutError("ELL response timeout")

        header = response[:3]
        payload = response[3:-2]

        if chr(header[0]) != str(self.address):
            raise RuntimeError("ELL address mismatch")

        return header[1:].decode(), int(payload.decode(), 16)

    # ---------- status / position ----------

    @property
    def status(self):
        header, response = self.query('gs')
        if header != 'GS':
            raise RuntimeError(f"Unexpected response: {header}")
        return RESPONSES[response]

    @property
    def _position(self):
        header, response = self.query('gp')
        if header != 'PO':
            raise RuntimeError(f"Unexpected response: {header}")
        return from_twos_complement(response)

    # ---------- public interface ----------

    def _wait_until_ready(self):
        while True:
            s = self.status
            if s == 'ok':
                return
            if s != 'busy':
                raise RuntimeError(f"ELL error: {s}")
            time.sleep(0.05)

    def home(self):
        self._wait_until_ready()
        self.query('ho')
        self._wait_until_ready()

    def tare(self):
        self._offset = -self._position

    @property
    def angle_unwrapped(self):
        return -360 * (self._position + self._offset) / COUNTS_PER_REVOLUTION

    @angle_unwrapped.setter
    def angle_unwrapped(self, degrees):
        self.move_by(degrees - self.angle_unwrapped)

    @property
    def angle(self):
        return self.angle_unwrapped % 360

    @angle.setter
    def angle(self, degrees):
        delta = degrees - self.angle
        if delta > 180:
            delta -= 360
        if delta < -180:
            delta += 360
        self.move_by(delta)

    def move_by(self, degrees):
        self._wait_until_ready()

        delta_counts = -round(degrees * COUNTS_PER_REVOLUTION / 360)
        data = to_twos_complement(delta_counts).to_bytes(4, 'big')

        header, response = self.query('mr', data)
        if header == 'GS':
            raise RuntimeError(RESPONSES[response])

        self._wait_until_ready()

    def close(self):
        self._conn.close()

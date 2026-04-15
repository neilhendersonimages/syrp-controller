import lgpio
import time
import threading

BTN_UP     = 17
BTN_DOWN   = 27
BTN_LEFT   = 22
BTN_RIGHT  = 23
BTN_CENTRE = 24

DEBOUNCE_MS    = 50
HOLD_START_MS  = 500
HOLD_REPEAT_MS = 100

class ButtonHandler:
    def __init__(self, callback):
        self.callback = callback
        self._names = {
            BTN_UP:     'up',
            BTN_DOWN:   'down',
            BTN_LEFT:   'left',
            BTN_RIGHT:  'right',
            BTN_CENTRE: 'centre',
        }
        self._prev     = {pin: 1 for pin in self._names}
        self._held     = {pin: False for pin in self._names}
        self._hold_ts  = {pin: None for pin in self._names}
        self._press_ts = {pin: None for pin in self._names}

        self._h = lgpio.gpiochip_open(0)
        for pin in self._names:
            lgpio.gpio_claim_input(self._h, pin, lgpio.SET_PULL_UP)

        self._running = True
        self._thread = threading.Thread(target=self._poll, daemon=True)
        self._thread.start()

    def _step_size(self, pin):
        if self._press_ts[pin] is None:
            return 1
        held_secs = time.time() - self._press_ts[pin]
        if held_secs < 0.5:
            return 1
        elif held_secs < 1.5:
            return 10
        elif held_secs < 3.0:
            return 50
        else:
            return 100

    def _poll(self):
        while self._running:
            now = time.time()
            for pin, name in self._names.items():
                val = lgpio.gpio_read(self._h, pin)

                # Fresh press
                if val == 0 and self._prev[pin] == 1:
                    self._press_ts[pin] = now
                    self.callback(name, 1)
                    self._held[pin] = True
                    self._hold_ts[pin] = now + HOLD_START_MS / 1000

                # Release
                elif val == 1 and self._prev[pin] == 0:
                    self._held[pin] = False
                    self._hold_ts[pin] = None
                    self._press_ts[pin] = None

                # Hold repeat (up/down only)
                elif (val == 0 and self._held[pin]
                      and name in ('up', 'down')
                      and self._hold_ts[pin]
                      and now >= self._hold_ts[pin]):
                    step = self._step_size(pin)
                    self.callback(name, step)
                    self._hold_ts[pin] = now + HOLD_REPEAT_MS / 1000

                self._prev[pin] = val
            time.sleep(0.02)

    def cleanup(self):
        self._running = False
        lgpio.gpiochip_close(self._h)

from RPLCD.i2c import CharLCD
import threading

class Display:
    def __init__(self):
        self.lcd = CharLCD('PCF8574', 0x27, cols=20, rows=4)
        self.lcd.clear()
        self._lock = threading.Lock()
        self.lcd.create_char(0, [0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F])

    def clear(self):
        with self._lock:
            self.lcd.clear()

    def write_lines(self, lines):
        with self._lock:
            self.lcd.home()
            for i, line in enumerate(lines[:4]):
                self.lcd.cursor_pos = (i, 0)
                self.lcd.write_string(f"{str(line):<20}"[:20])

    def show_progress(self, frame, total, interval, seconds_left):
        pct = frame / total if total > 0 else 0
        filled = int(pct * 20)
        bar = chr(0) * filled + '-' * (20 - filled)
        lines = [
            f"Frame {frame:>4} / {total:<4}     ",
            f"Next in: {seconds_left:>4}s       ",
            bar,
            f"                    ",
        ]
        self.write_lines(lines)

    def show_menu(self, title, items, selected):
        lines = [f"{title:<20}"]
        for i, item in enumerate(items[:3]):
            prefix = '>' if i == selected else ' '
            lines.append(f"{prefix} {item:<18}")
        while len(lines) < 4:
            lines.append(' ' * 20)
        self.write_lines(lines)

    def show_edit(self, title, param, value, unit=''):
        lines = [
            f"{title:<20}",
            f"{param:<20}",
            f"  {value} {unit:<16}",
            f"  up/down  ctr=save ",
        ]
        self.write_lines(lines)

    def show_message(self, line1, line2='', line3='', line4=''):
        self.write_lines([line1, line2, line3, line4])

import time
import threading
import subprocess
import sys
import tempfile
import os
from rich.live import Live
from rich.table import Table
from rich.console import Console


class InstanceMonitor:
    def __init__(self, instance, title="Instance Monitor", refresh_rate=0.5):
        self.instance = instance
        self.title = title
        self.refresh_rate = refresh_rate
        self._stop_event = threading.Event()
        self._thread = None

    def _make_table(self):
        table = Table(title=f"{self.title} — {self.instance.__class__.__name__}", border_style="blue")
        table.add_column("Attribute", style="cyan")
        table.add_column("Value", style="yellow")

        for attr in sorted(dir(self.instance)):
            if attr.startswith("__") or attr == "__dict__":
                continue
            try:
                value = getattr(self.instance, attr)
                if callable(value):
                    continue
                if hasattr(value, '__dict__') and not isinstance(value, (str, int, float, bool, list, dict, tuple, set)):
                    continue
                table.add_row(attr, str(value))
            except:
                continue
        return table

    def _run(self):
        console = Console()
        with Live(self._make_table(), refresh_per_second=4, console=console) as live:
            while not self._stop_event.is_set():
                live.update(self._make_table())
                time.sleep(self.refresh_rate)

    def start(self, new_window=False):
        if new_window:
            # Create a temporary script to run in a new terminal
            code = f"""
import sys
sys.path.insert(0, r"{sys.path[0]}")
from instance_monitor import InstanceMonitor
instance = {self.instance.__class__.__module__}.{self.instance.__class__.__name__}.__new__({self.instance.__class__.__module__}.{self.instance.__class__.__name__})
instance.__dict__.update({repr(self.instance.__dict__)})
monitor = InstanceMonitor(instance, title="{self.title}")
monitor.start()
input("Press Enter to close monitor...")
"""

            with tempfile.NamedTemporaryFile(mode='w', suffix='.py', delete=False) as f:
                f.write(code)
                temp_file = f.name

            subprocess.Popen(f'start cmd /c python "{temp_file}"', shell=True)
            print(f"✅ Monitor started in new window for {self.instance.__class__.__name__}")

        else:
            if self._thread and self._thread.is_alive():
                return
            self._stop_event.clear()
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()
            print(f"✅ Monitor started in current terminal for {self.instance.__class__.__name__}")

    def stop(self):
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=1)
        print("⛔ Monitor stopped.")
# A test program for a Bleak connection with no atexit cleanup.

import asyncio
import platform
import sys
import time
from bleak import BleakClient, BleakScanner


# Adapt to your actual device.
device_address = "0C:8B:95:F2:B4:36"

print(f"OS: {platform.platform()}", flush=True)
print(f"Platform:: {platform.uname()}", flush=True)
print(f"Python {sys.version}", flush=True)

async def test():
    global client
    print(f"Trying to connect to {device_address}", flush=True)
    device = await BleakScanner.find_device_by_address(device_address, timeout=10.0)
    assert device
    client = BleakClient(device)
    assert client
    await client.connect(timeout=10.0)
    assert client.is_connected
    print(f"Connected", flush=True)
    print(f"Waiting 5 secs...", flush=True)
    time.sleep(5.0)
    print(f"Test done", flush=True)

event_loop = asyncio.new_event_loop()
event_loop.run_until_complete(test())
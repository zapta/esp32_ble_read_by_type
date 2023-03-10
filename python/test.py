import asyncio
import sys
import platform
import signal
from bleak import BleakClient, BleakScanner
import signal

# Adapt to your BLE device.
device_address = "EC:62:60:B2:EA:56"

signal.signal(signal.SIGINT, lambda number, frame: sys.exit())

print(f"OS: {platform.platform()}", flush=True)
print(f"Platform:: {platform.uname()}", flush=True)
print(f"Python {sys.version}", flush=True)


async def test():
    global main_event_loop
    print(f"Trying to connect to {device_address}", flush=True)
    device = await BleakScanner.find_device_by_address(device_address, timeout=10.0)
    assert device
    async with BleakClient(device) as client:
        print(f"Device found.", flush=True)
        assert client.is_connected
        print(f"Connected found.", flush=True)
        service = client.services.get_service(
            "6b6a78d7-8ee0-4a26-ba7b-62e357dd9720")
        assert (service)
        print(f"Service found.", flush=True)
        chrc = service.get_characteristic("ff06")
        assert (chrc)
        print(f"Characteristic found.", flush=True)
        for i in range(5):
            print(f"\n{i + 1} Writing...", flush=True)
            await client.write_gatt_char(chrc, bytearray([0x06, 0x03]))
            print("Writing done.")
            await asyncio.sleep(0.2)
        print(f"\nAll done.", flush=True)
        main_event_loop.stop()


main_event_loop = asyncio.new_event_loop()
try:
    main_event_loop.run_until_complete(test())
except:
    pass

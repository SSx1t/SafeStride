import asyncio
from bleak import BleakScanner

async def main():
    print("Scanning for BLE devices...")
    devices = await BleakScanner.discover(timeout=15)

    if not devices:
        print("No BLE devices found.")
        return

    for d in devices:
        print(f"Name: {d.name} | Address: {d.address}")

asyncio.run(main())
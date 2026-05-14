import asyncio
import sys
from pathlib import Path
from bleak import BleakClient

CMD_CHAR_UUID = "4f5b1a01-7b3a-4f9b-9e80-1a1e5b7c2d40"
LOG_CHAR_UUID = "4f5b1a04-7b3a-4f9b-9e80-1a1e5b7c2d40"

DEVICE_ADDRESS = "4C:C3:82:36:80:42"

CMD_START = bytes([0x20])
CMD_STOP = bytes([0x21])

if len(sys.argv) < 2:
    print("Usage: python record_walk_ble.py calibration/<filename>.csv")
    sys.exit(1)

outfile = Path(sys.argv[1])
outfile.parent.mkdir(parents=True, exist_ok=True)


def has_char(client, uuid):
    uuid = uuid.lower()
    for service in client.services:
        for char in service.characteristics:
            if char.uuid.lower() == uuid:
                return True
    return False


async def main():
    print(f"Connecting to ESP32 at {DEVICE_ADDRESS}...")

    async with BleakClient(DEVICE_ADDRESS) as client:
        print("Connected.")

        if not has_char(client, CMD_CHAR_UUID):
            print("ERROR: CMD characteristic not found.")
            print("Re-upload the Arduino code above.")
            return

        if not has_char(client, LOG_CHAR_UUID):
            print("ERROR: LOG characteristic not found.")
            return

        samples = 0
        recording_active = False

        with open(outfile, "w", newline="") as f:
            f.write("# t_ms,ax,ay,az,gx,gy,gz,alt\n")

            def handle_notify(_handle, data):
                nonlocal samples, recording_active

                if not recording_active:
                    return

                line = data.decode("utf-8", errors="ignore").strip()
                if not line:
                    return

                try:
                    f.write(line + "\n")
                    f.flush()
                except ValueError:
                    return

                samples += 1

                if samples % 50 == 0:
                    print(f"{samples} samples...", end="\r")

            await client.start_notify(LOG_CHAR_UUID, handle_notify)

            input("Press Enter to START recording...")
            recording_active = True
            await client.write_gatt_char(CMD_CHAR_UUID, CMD_START, response=True)

            print("Recording... Press Enter again to STOP.")
            await asyncio.to_thread(input)

            print("Stopping...")
            recording_active = False
            await client.write_gatt_char(CMD_CHAR_UUID, CMD_STOP, response=True)

            await asyncio.sleep(0.3)
            await client.stop_notify(LOG_CHAR_UUID)

            f.write(f"# RECORDING STOPPED. Samples: {samples}\n")

        print(f"\nSaved {samples} samples to {outfile}")


if __name__ == "__main__":
    asyncio.run(main())
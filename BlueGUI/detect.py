import asyncio
import typing
from bleak import BleakScanner, BleakClient

TARGET_NAME = "Unicycle_BLE"
Client : BleakClient

def get_client() -> typing.Optional[BleakClient]:
    global Client
    return Client if 'Client' in globals() else None

async def connect_ble():
    global Client
    print("🔍 Scanning for BLE devices...")
    devices = await BleakScanner.discover()
    
    # list all found devices
    for d in devices:
        print(f"Found device: {d.name}, {d.address}")

    target = next((d for d in devices if d.name == TARGET_NAME), None)
    if not target:
        print(f"❌ Device '{TARGET_NAME}' not found.")
        return False

    Client = BleakClient(target)
    await Client.connect()

    if not Client.is_connected:
        print("❌ Connection failed.")
        return False

    print(f"✅ Connected to {TARGET_NAME}")
    for service in Client.services:
        print(f"[Service] {service.uuid}: {service.description}")
        for char in service.characteristics:
            props = ", ".join(char.properties)
            print(f"  └── [Char] {char.uuid} ({props})")
            #handle = char.handle
            print(f"    └── Handle: {char.handle}")

    return True


async def disconnect_ble():
    global Client
    if Client and Client.is_connected:
        await Client.disconnect()
        print("🔌 Disconnected.")


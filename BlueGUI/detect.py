import asyncio
import typing
from bleak import BleakScanner, BleakClient
from data import MotorData

TARGET_NAME = "Unicycle_BLE"
Client: typing.Optional[BleakClient] = None
NOTIFY_CHAR_UUID = "00000001-8e22-4541-9d4c-21edae82ed19"  # notify uuid, receiving data from it, Main_Motor in MCU


def get_client() -> typing.Optional[BleakClient]:
    global Client
    return Client if 'Client' in globals() else None

async def handle_notify(sender, data: bytearray):
    print(f"Notification from {sender}: {data.hex()}")
    motor_data = MotorData()
    motor_data.update_from_bytes(data)
    # process the received data

async def connect_ble():
    global Client
    print("Scanning for BLE devices...")
    devices = await BleakScanner.discover()
    
    # list all found devices
    # for d in devices:
    #     print(f"Found device: {d.name}, {d.address}")

    target = next((d for d in devices if d.name == TARGET_NAME), None)
    if not target:
        print(f"Device '{TARGET_NAME}' not found.")
        return False

    Client = BleakClient(target)
    await Client.connect()

    if not Client.is_connected:
        print("Connection failed.")
        return False

    print(f"Connected to {TARGET_NAME}")
    for service in Client.services:
        print(f"[Service] {service.uuid}: {service.description}")
        for char in service.characteristics:
            props = ", ".join(char.properties)
            print(f"  └── [Char] {char.uuid} ({props})")
            #handle = char.handle
            print(f"    └── Handle: {char.handle}")
            
      # 启用通知
    try:
        await Client.start_notify(NOTIFY_CHAR_UUID, handle_notify)
        print("🔔 Notifications enabled!")
    except Exception as e:
        print("Failed to enable notify:", e)

    print("⚠️ Connection lost.")
    return True


    return True

async def write_ble() -> bool:
    print("Writing data to BLE device...")
    motor_data = MotorData()
    motor_data.update()
    data = motor_data.response_bytes
    
    global Client
    if not Client or not Client.is_connected:
        print("⚠️ BLE client not connected!")
        return False
    
    WRITE_CHAR_UUID = "00000003-8e22-4541-9d4c-21edae82ed19"  # data receive uuid in MCU, UcCommand
    
    try:
        await Client.write_gatt_char(WRITE_CHAR_UUID, data, response=False)
        print(f"Wrote {len(data)} bytes: {data.hex()}")
        return True
    except Exception as e:
        print("Write failed:", e)
        return False


async def disconnect_ble():
    global Client
    if Client and Client.is_connected:
        await Client.disconnect()
        print("🔌 Disconnected.")


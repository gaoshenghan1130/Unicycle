from detect import get_client, connect_ble, disconnect_ble
import bleak
import asyncio

async def main():
    if await connect_ble():
        # Example: read/write operations here  
        client = get_client()
        if client is not None:
            #data = await client.read_gatt_char("00000001-8e22-4541-9d4c-21edae82ed19")
            #print("Data:", data)
            write_data = bytearray([0x01, 0x02, 0x03, 0x04])
            await client.write_gatt_char("00000003-8e22-4541-9d4c-21edae82ed19", write_data, response=False
            )
            print("Wrote:", write_data)
        else:
            print("No BLE client connected")


asyncio.run(main())
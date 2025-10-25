from detect import get_client, connect_ble, disconnect_ble, write_ble, check_connection
import bleak
import asyncio
import threading
import data

async def main():
    await connect_ble()
    while True:
        await asyncio.sleep(1) 
        print("Preparing to write data...")
        await write_ble()
        await check_connection()

asyncio.run(main())
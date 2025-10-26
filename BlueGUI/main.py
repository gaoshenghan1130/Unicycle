from detect import connect_ble,  write_ble
import asyncio
import control_panel.pmain as pmain
import threading
import time


async def main():
    time.sleep(2)  # wait for other threads to initialize
    await connect_ble()
    while True:
        await asyncio.sleep(1) 
        await write_ble()
        
def displayValue_onterminal():
    time.sleep(2)  # wait for other threads to initialize
    while True:
        left, forward =  pmain.get_torques()
        print(f"Left/Right velocity: {left:+.2f}, Forward/Back velocity: {forward:+.2f}")
        time.sleep(0.5)
        
if __name__ == "__main__":
    loop = asyncio.new_event_loop()
    threading.Thread(target=lambda: loop.run_until_complete(main()), daemon=True).start() # use an additional thread for asyncio event loop
    threading.Thread(target=displayValue_onterminal, daemon=True).start()
    
    
    pmain.initialize_torque_ui() # must be in the main thread for tkinter
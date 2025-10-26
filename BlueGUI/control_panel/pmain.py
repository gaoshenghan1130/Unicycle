from control_panel.velDisplay import VelocityUI
import tkinter as tk
import threading    

ui_instance = None
lock = threading.Lock()

def get_torques():
    with lock:
        global ui_instance
        if ui_instance is None:
            raise RuntimeError("Torque UI has not been initialized. Please run 'initialize_torque_ui()' first.")
        return ui_instance.get_torques()

def initialize_torque_ui():
    global ui_instance
    root = tk.Tk()
    ui_instance = VelocityUI(root)
    root.mainloop()
    
    
def run_torque_ui():
    pass # Placeholder for potential future functionality

if __name__ == "__main__":
    initialize_torque_ui()
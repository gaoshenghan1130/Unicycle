import tkinter as tk
from tkinter import ttk
import threading

class VelocityUI:
    def __init__(self, root):
        self.root = root
        root.title("Velocity Control UI")
        root.geometry("480x400")
        
        # ---- Threadsafe lock ----
        self.lock = threading.Lock()

        # ---- State ----
        self.left_velocity = 0.0
        self.forward_velocity = 0.0
        self.step = tk.DoubleVar(value=0.2)
        self.max_velocity = 10.0

        # ---- Title ----
        ttk.Label(root, text="Velocity Control (Keyboard)", font=("Arial", 16, "bold")).pack(pady=5)

        # ---- Display labels ----
        info_frame = ttk.Frame(root)
        info_frame.pack(pady=5)
        self.left_label = ttk.Label(info_frame, text="Left/Right velocity: +0.00", font=("Arial", 13))
        self.left_label.grid(row=0, column=0, padx=20, pady=5)
        self.forward_label = ttk.Label(info_frame, text="Forward/Back velocity: +0.00", font=("Arial", 13))
        self.forward_label.grid(row=1, column=0, padx=20, pady=5)

        # ---- Step adjustment bar ----
        bar_frame = ttk.Frame(root)
        bar_frame.pack(pady=5)
        ttk.Label(bar_frame, text="Velocity increment per key press:").pack()
        step_slider = ttk.Scale(
            bar_frame,
            from_=0.05,
            to=1.0,
            variable=self.step,
            orient="horizontal",
            length=200,
            command=lambda e: self.update_labels()
        )
        step_slider.pack()
        self.step_label = ttk.Label(bar_frame, text=f"Current step: {self.step.get():.2f}")
        self.step_label.pack()

        # ---- Coordinate panel ----
        self.canvas_size = 200
        self.canvas = tk.Canvas(root, width=self.canvas_size, height=self.canvas_size, bg="white", highlightthickness=1, highlightbackground="black")
        self.canvas.pack(pady=10)
        self.center = self.canvas_size // 2
        self.scale = self.canvas_size / (2 * self.max_velocity)
        self.point = self.canvas.create_oval(self.center-5, self.center-5, self.center+5, self.center+5, fill="red")

        # Draw axes
        self.canvas.create_line(self.center, 0, self.center, self.canvas_size, fill="gray", dash=(2,2))
        self.canvas.create_line(0, self.center, self.canvas_size, self.center, fill="gray", dash=(2,2))

        # ---- Instructions ----
        ttk.Label(root, text="Controls: W/S = Forward/Back | A/D = Left/Right | Space = Stop all",
                  foreground="gray").pack(pady=5)

        # ---- Bind keyboard ----
        root.bind("<KeyPress-w>", lambda e: self.adjust_velocity("forward", +self.step.get()))
        root.bind("<KeyPress-s>", lambda e: self.adjust_velocity("forward", -self.step.get()))
        root.bind("<KeyPress-a>", lambda e: self.adjust_velocity("left", +self.step.get()))
        root.bind("<KeyPress-d>", lambda e: self.adjust_velocity("left", -self.step.get()))
        root.bind("<space>", lambda e: self.stop_all())

    # ---- Logic ----
    def adjust_velocity(self, direction, delta):
        with self.lock:
            if direction == "left":
                self.left_velocity = self._clamp(self.left_velocity + delta)
            elif direction == "forward":
                self.forward_velocity = self._clamp(self.forward_velocity + delta)
        self.update_labels()

    def stop_all(self):
        with self.lock:
            self.left_velocity = 0.0
            self.forward_velocity = 0.0
        self.update_labels()

    def _clamp(self, v):
        return max(-self.max_velocity, min(self.max_velocity, v))

    def update_labels(self):
        self.left_label.config(text=f"Left/Right velocity: {self.left_velocity:+.2f}")
        self.forward_label.config(text=f"Forward/Back velocity: {self.forward_velocity:+.2f}")
        self.step_label.config(text=f"Current step: {self.step.get():.2f}")
        self.update_canvas_point()

    def update_canvas_point(self):
        x = self.center - self.left_velocity * self.scale
        y = self.center - self.forward_velocity * self.scale
        r = 5
        self.canvas.coords(self.point, x-r, y-r, x+r, y+r)
        
    def get_torques(self):
        with self.lock:
            return self.left_velocity, self.forward_velocity

if __name__ == "__main__":
    root = tk.Tk()
    app = VelocityUI(root)
    root.mainloop()

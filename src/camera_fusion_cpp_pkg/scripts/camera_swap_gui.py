#!/usr/bin/env python3
"""
camera_swap_gui.py
GUI mínima para intercambiar el orden de las cámaras en el nodo de fusión panorámica.
Llama al servicio ROS 2: /camera_fusion/swap_cameras
"""

import threading
import tkinter as tk
from tkinter import font as tkfont

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


# ──────────────────────────────────────────────────────────────────────────────
# Nodo ROS 2 ligero (solo cliente de servicio)
# ──────────────────────────────────────────────────────────────────────────────
class SwapClient(Node):
    def __init__(self):
        super().__init__("camera_swap_gui_node")
        self.cli = self.create_client(Trigger, "/camera_fusion/swap_cameras")

    def call_swap(self, on_done):
        """Llama al servicio de forma asíncrona y ejecuta on_done(success, message)."""
        if not self.cli.service_is_ready():
            on_done(False, "Servicio no disponible.\n¿Está corriendo el nodo de fusión?")
            return

        req = Trigger.Request()
        future = self.cli.call_async(req)
        future.add_done_callback(
            lambda f: on_done(f.result().success, f.result().message)
        )


# ──────────────────────────────────────────────────────────────────────────────
# GUI
# ──────────────────────────────────────────────────────────────────────────────
class App:
    BG         = "#1a1a2e"
    PANEL_BG   = "#16213e"
    ACCENT     = "#e94560"
    ACCENT_HOV = "#c73652"
    SUCCESS    = "#4ecca3"
    ERROR      = "#e94560"
    TEXT       = "#eaeaea"
    MUTED      = "#8892a4"

    def __init__(self, root: tk.Tk, ros_node: SwapClient):
        self.root     = root
        self.ros_node = ros_node

        root.title("Camera Swap")
        root.configure(bg=self.BG)
        root.resizable(False, False)
        root.geometry("320x160")

        self._build_ui()
        self._center_window()

    # ── construcción de la interfaz ──────────────────────────────────────────

    def _build_ui(self):
        status_font = tkfont.Font(family="Segoe UI", size=10, slant="italic")

        # Botón de swap
        self.btn = tk.Button(
            self.root,
            text="⇄  Intercambiar Cámaras",
            bg=self.ACCENT, fg="white",
            activebackground=self.ACCENT_HOV, activeforeground="white",
            font=tkfont.Font(family="Segoe UI", size=13, weight="bold"),
            relief="flat", bd=0, cursor="hand2",
            padx=20, pady=14,
            command=self._on_swap
        )
        self.btn.pack(expand=True)
        self.btn.bind("<Enter>", lambda e: self.btn.config(bg=self.ACCENT_HOV))
        self.btn.bind("<Leave>", lambda e: self.btn.config(bg=self.ACCENT))

        # Etiqueta de estado
        self.status_var = tk.StringVar(value="")
        self.status_lbl = tk.Label(
            self.root, textvariable=self.status_var,
            bg=self.BG, fg=self.MUTED, font=status_font
        )
        self.status_lbl.pack(pady=(0, 12))

    def _center_window(self):
        self.root.update_idletasks()
        w, h = self.root.winfo_width(), self.root.winfo_height()
        x = (self.root.winfo_screenwidth()  - w) // 2
        y = (self.root.winfo_screenheight() - h) // 2
        self.root.geometry(f"{w}x{h}+{x}+{y}")

    # ── lógica del botón ─────────────────────────────────────────────────────

    def _on_swap(self):
        self.btn.config(state="disabled", text="⏳  Llamando servicio…")
        self.status_var.set("Esperando respuesta del nodo…")
        self.status_lbl.config(fg=self.MUTED)
        self.ros_node.call_swap(self._on_swap_done)

    def _on_swap_done(self, success: bool, message: str):
        # Volvemos al hilo de tkinter
        self.root.after(0, self._update_ui, success, message)

    def _update_ui(self, success: bool, message: str):
        if success:
            self.status_var.set("✔  Intercambiadas")
            self.status_lbl.config(fg=self.SUCCESS)
        else:
            self.status_var.set(f"✖  {message}")
            self.status_lbl.config(fg=self.ERROR)

        self.btn.config(state="normal", text="⇄  Intercambiar Cámaras")


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    ros_node = SwapClient()

    # Spin de ROS en hilo separado para no bloquear la GUI
    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    root = tk.Tk()
    App(root, ros_node)

    try:
        root.mainloop()
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

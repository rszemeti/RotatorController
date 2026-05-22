# Aerial Rotator Control GUI - Improved Design
import tkinter as tk
from tkinter import ttk, messagebox, simpledialog
import serial
import serial.tools.list_ports
import threading
import time
import math
import json
import os
import re

def maidenhead_to_latlon(locator):
    """Convert Maidenhead locator to lat/lon (center of grid)"""
    locator = locator.upper().strip()
    
    # Validate format
    if not re.match(r'^[A-R]{2}[0-9]{2}([A-X]{2})?([0-9]{2})?$', locator):
        return None, None
    
    lon = (ord(locator[0]) - ord('A')) * 20 - 180
    lat = (ord(locator[1]) - ord('A')) * 10 - 90
    
    lon += (int(locator[2])) * 2
    lat += (int(locator[3])) * 1
    
    if len(locator) >= 6:
        lon += (ord(locator[4]) - ord('A')) * (2/24) + (1/24)
        lat += (ord(locator[5]) - ord('A')) * (1/24) + (1/48)
    else:
        lon += 1  # Center of field
        lat += 0.5
    
    if len(locator) == 8:
        lon += int(locator[6]) * (2/240) + (1/240)
        lat += int(locator[7]) * (1/240) + (1/480)
    
    return lat, lon

def calculate_bearing(lat1, lon1, lat2, lon2):
    """Calculate bearing from point 1 to point 2"""
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    dlon_rad = math.radians(lon2 - lon1)
    
    x = math.sin(dlon_rad) * math.cos(lat2_rad)
    y = math.cos(lat1_rad) * math.sin(lat2_rad) - \
        math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon_rad)
    
    bearing_rad = math.atan2(x, y)
    bearing_deg = math.degrees(bearing_rad)
    bearing_deg = (bearing_deg + 360) % 360
    
    return bearing_deg

def calculate_distance(lat1, lon1, lat2, lon2):
    """Calculate distance in km between two points"""
    R = 6371  # Earth's radius in km
    
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    
    a = math.sin(dlat/2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    
    return R * c

class SettingsDialog:
    def __init__(self, parent, current_speed, current_accel):
        self.result = None
        self.dialog = tk.Toplevel(parent)
        self.dialog.title("Motion Settings")
        self.dialog.geometry("400x250")
        self.dialog.resizable(False, False)
        self.dialog.configure(bg="#2b2b2b")
        
        # Center the dialog
        self.dialog.transient(parent)
        self.dialog.grab_set()
        
        # Title
        title = tk.Label(self.dialog, text="Motion Settings (Saved to EEPROM)", 
                        bg="#2b2b2b", fg="#ffffff", font=('Arial', 12, 'bold'))
        title.pack(pady=15)
        
        # Settings frame
        settings_frame = tk.Frame(self.dialog, bg="#2b2b2b")
        settings_frame.pack(pady=10)
        
        # Speed setting
        tk.Label(settings_frame, text="Max Speed:", bg="#2b2b2b", fg="#ffffff",
                font=('Arial', 10)).grid(row=0, column=0, padx=10, pady=10, sticky='e')
        
        self.speed_entry = tk.Entry(settings_frame, width=10, font=('Arial', 11))
        self.speed_entry.grid(row=0, column=1, padx=5, pady=10)
        self.speed_entry.insert(0, str(current_speed))
        
        tk.Label(settings_frame, text="steps/s (100-50000)", bg="#2b2b2b", fg="#888888",
                font=('Arial', 9)).grid(row=0, column=2, padx=5, pady=10, sticky='w')
        
        # Acceleration setting
        tk.Label(settings_frame, text="Acceleration:", bg="#2b2b2b", fg="#ffffff",
                font=('Arial', 10)).grid(row=1, column=0, padx=10, pady=10, sticky='e')
        
        self.accel_entry = tk.Entry(settings_frame, width=10, font=('Arial', 11))
        self.accel_entry.grid(row=1, column=1, padx=5, pady=10)
        self.accel_entry.insert(0, str(current_accel))
        
        tk.Label(settings_frame, text="steps/s² (100-100000)", bg="#2b2b2b", fg="#888888",
                font=('Arial', 9)).grid(row=1, column=2, padx=5, pady=10, sticky='w')
        
        # Buttons
        button_frame = tk.Frame(self.dialog, bg="#2b2b2b")
        button_frame.pack(pady=20)
        
        tk.Button(button_frame, text="Apply", command=self.apply,
                 bg="#4a9eff", fg="#ffffff", width=10, font=('Arial', 10, 'bold')).pack(side='left', padx=5)
        
        tk.Button(button_frame, text="Cancel", command=self.cancel,
                 bg="#3a3a3a", fg="#ffffff", width=10, font=('Arial', 10)).pack(side='left', padx=5)
        
        # Bind Enter key
        self.dialog.bind('<Return>', lambda e: self.apply())
        self.dialog.bind('<Escape>', lambda e: self.cancel())
        
    def apply(self):
        try:
            speed = float(self.speed_entry.get())
            accel = float(self.accel_entry.get())
            
            if not (100 <= speed <= 50000):
                messagebox.showerror("Error", "Speed must be 100-50000 steps/s", parent=self.dialog)
                return
            
            if not (100 <= accel <= 100000):
                messagebox.showerror("Error", "Acceleration must be 100-100000 steps/s²", parent=self.dialog)
                return
            
            self.result = (speed, accel)
            self.dialog.destroy()
        except ValueError:
            messagebox.showerror("Error", "Invalid values", parent=self.dialog)
    
    def cancel(self):
        self.dialog.destroy()

class SetPositionDialog:
    def __init__(self, parent, current_angle):
        self.result = None
        self.dialog = tk.Toplevel(parent)
        self.dialog.title("Set Current Position")
        self.dialog.geometry("350x180")
        self.dialog.resizable(False, False)
        self.dialog.configure(bg="#2b2b2b")
        
        # Center the dialog
        self.dialog.transient(parent)
        self.dialog.grab_set()
        
        # Title
        title = tk.Label(self.dialog, text="Set Current Position", 
                        bg="#2b2b2b", fg="#ffffff", font=('Arial', 12, 'bold'))
        title.pack(pady=15)
        
        # Info
        info = tk.Label(self.dialog, text="Define what angle the rotator is currently pointing at", 
                       bg="#2b2b2b", fg="#888888", font=('Arial', 9))
        info.pack(pady=5)
        
        # Angle entry
        entry_frame = tk.Frame(self.dialog, bg="#2b2b2b")
        entry_frame.pack(pady=15)
        
        tk.Label(entry_frame, text="Angle:", bg="#2b2b2b", fg="#ffffff",
                font=('Arial', 10)).pack(side='left', padx=5)
        
        self.angle_entry = tk.Entry(entry_frame, width=10, font=('Arial', 12))
        self.angle_entry.pack(side='left', padx=5)
        self.angle_entry.insert(0, f"{current_angle:.1f}")
        self.angle_entry.select_range(0, tk.END)
        self.angle_entry.focus()
        
        tk.Label(entry_frame, text="° (0-360)", bg="#2b2b2b", fg="#888888",
                font=('Arial', 9)).pack(side='left', padx=5)
        
        # Buttons
        button_frame = tk.Frame(self.dialog, bg="#2b2b2b")
        button_frame.pack(pady=15)
        
        tk.Button(button_frame, text="Set", command=self.apply,
                 bg="#4a9eff", fg="#ffffff", width=10, font=('Arial', 10, 'bold')).pack(side='left', padx=5)
        
        tk.Button(button_frame, text="Cancel", command=self.cancel,
                 bg="#3a3a3a", fg="#ffffff", width=10, font=('Arial', 10)).pack(side='left', padx=5)
        
        # Bind Enter key
        self.dialog.bind('<Return>', lambda e: self.apply())
        self.dialog.bind('<Escape>', lambda e: self.cancel())
    
    def apply(self):
        try:
            angle = float(self.angle_entry.get())
            if not (0 <= angle < 360):
                messagebox.showerror("Error", "Angle must be between 0 and 360", parent=self.dialog)
                return
            
            self.result = angle
            self.dialog.destroy()
        except ValueError:
            messagebox.showerror("Error", "Invalid angle value", parent=self.dialog)
    
    def cancel(self):
        self.dialog.destroy()

class LocationDialog:
    def __init__(self, parent, current_locator):
        self.result = None
        self.dialog = tk.Toplevel(parent)
        self.dialog.title("Set My Location")
        self.dialog.geometry("400x200")
        self.dialog.resizable(False, False)
        self.dialog.configure(bg="#2b2b2b")
        
        self.dialog.transient(parent)
        self.dialog.grab_set()
        
        # Title
        title = tk.Label(self.dialog, text="Set My Location", 
                        bg="#2b2b2b", fg="#ffffff", font=('Arial', 12, 'bold'))
        title.pack(pady=15)
        
        # Info
        info = tk.Label(self.dialog, text="Enter your Maidenhead locator (6 or 8 digits)", 
                       bg="#2b2b2b", fg="#888888", font=('Arial', 9))
        info.pack(pady=5)
        
        # Locator entry
        entry_frame = tk.Frame(self.dialog, bg="#2b2b2b")
        entry_frame.pack(pady=15)
        
        tk.Label(entry_frame, text="Locator:", bg="#2b2b2b", fg="#ffffff",
                font=('Arial', 10)).pack(side='left', padx=5)
        
        self.locator_entry = tk.Entry(entry_frame, width=12, font=('Arial', 12))
        self.locator_entry.pack(side='left', padx=5)
        if current_locator:
            self.locator_entry.insert(0, current_locator)
        self.locator_entry.select_range(0, tk.END)
        self.locator_entry.focus()
        
        tk.Label(entry_frame, text="(e.g., IO91vl or IO91vl34)", bg="#2b2b2b", fg="#888888",
                font=('Arial', 8)).pack(side='left', padx=5)
        
        # Buttons
        button_frame = tk.Frame(self.dialog, bg="#2b2b2b")
        button_frame.pack(pady=15)
        
        tk.Button(button_frame, text="Set", command=self.apply,
                 bg="#4a9eff", fg="#ffffff", width=10, font=('Arial', 10, 'bold')).pack(side='left', padx=5)
        
        tk.Button(button_frame, text="Cancel", command=self.cancel,
                 bg="#3a3a3a", fg="#ffffff", width=10, font=('Arial', 10)).pack(side='left', padx=5)
        
        self.dialog.bind('<Return>', lambda e: self.apply())
        self.dialog.bind('<Escape>', lambda e: self.cancel())
    
    def apply(self):
        locator = self.locator_entry.get().upper().strip()
        
        # Validate
        if not re.match(r'^[A-R]{2}[0-9]{2}([A-X]{2})?([0-9]{2})?$', locator):
            messagebox.showerror("Error", "Invalid Maidenhead locator format\nExpected: IO91vl or IO91vl34", 
                               parent=self.dialog)
            return
        
        lat, lon = maidenhead_to_latlon(locator)
        if lat is None:
            messagebox.showerror("Error", "Invalid Maidenhead locator", parent=self.dialog)
            return
        
        self.result = locator
        self.dialog.destroy()
    
    def cancel(self):
        self.dialog.destroy()

class StationDialog:
    def __init__(self, parent, callsign="", locator=""):
        self.result = None
        self.dialog = tk.Toplevel(parent)
        self.dialog.title("Add/Edit Station")
        self.dialog.geometry("400x230")
        self.dialog.resizable(False, False)
        self.dialog.configure(bg="#2b2b2b")
        
        self.dialog.transient(parent)
        self.dialog.grab_set()
        
        # Title
        title = tk.Label(self.dialog, text="Add/Edit Station", 
                        bg="#2b2b2b", fg="#ffffff", font=('Arial', 12, 'bold'))
        title.pack(pady=15)
        
        # Entry frame
        entry_frame = tk.Frame(self.dialog, bg="#2b2b2b")
        entry_frame.pack(pady=10)
        
        # Callsign
        tk.Label(entry_frame, text="Callsign:", bg="#2b2b2b", fg="#ffffff",
                font=('Arial', 10)).grid(row=0, column=0, padx=10, pady=10, sticky='e')
        
        self.callsign_entry = tk.Entry(entry_frame, width=15, font=('Arial', 11))
        self.callsign_entry.grid(row=0, column=1, padx=5, pady=10)
        self.callsign_entry.insert(0, callsign)
        self.callsign_entry.focus()
        
        # Locator
        tk.Label(entry_frame, text="Locator:", bg="#2b2b2b", fg="#ffffff",
                font=('Arial', 10)).grid(row=1, column=0, padx=10, pady=10, sticky='e')
        
        self.locator_entry = tk.Entry(entry_frame, width=15, font=('Arial', 11))
        self.locator_entry.grid(row=1, column=1, padx=5, pady=10)
        self.locator_entry.insert(0, locator)
        
        tk.Label(entry_frame, text="(e.g., IO91vl)", bg="#2b2b2b", fg="#888888",
                font=('Arial', 8)).grid(row=1, column=2, padx=5, pady=10, sticky='w')
        
        # Buttons
        button_frame = tk.Frame(self.dialog, bg="#2b2b2b")
        button_frame.pack(pady=15)
        
        tk.Button(button_frame, text="Save", command=self.apply,
                 bg="#4a9eff", fg="#ffffff", width=10, font=('Arial', 10, 'bold')).pack(side='left', padx=5)
        
        tk.Button(button_frame, text="Cancel", command=self.cancel,
                 bg="#3a3a3a", fg="#ffffff", width=10, font=('Arial', 10)).pack(side='left', padx=5)
        
        self.dialog.bind('<Return>', lambda e: self.apply())
        self.dialog.bind('<Escape>', lambda e: self.cancel())
    
    def apply(self):
        callsign = self.callsign_entry.get().upper().strip()
        locator = self.locator_entry.get().upper().strip()
        
        if not callsign:
            messagebox.showerror("Error", "Callsign is required", parent=self.dialog)
            return
        
        if not locator:
            messagebox.showerror("Error", "Locator is required", parent=self.dialog)
            return
        
        # Validate locator
        if not re.match(r'^[A-R]{2}[0-9]{2}([A-X]{2})?([0-9]{2})?$', locator):
            messagebox.showerror("Error", "Invalid Maidenhead locator format", parent=self.dialog)
            return
        
        lat, lon = maidenhead_to_latlon(locator)
        if lat is None:
            messagebox.showerror("Error", "Invalid Maidenhead locator", parent=self.dialog)
            return
        
        self.result = (callsign, locator)
        self.dialog.destroy()
    
    def cancel(self):
        self.dialog.destroy()

class RotatorGUI:
    SETTINGS_FILE = "rotator_settings.json"
    
    def __init__(self, root):
        self.root = root
        self.root.title("Aerial Rotator Control")
        self.root.geometry("750x750")  # Wider sidebar, reduced height
        self.root.resizable(False, False)
        
        self.serial_port = None
        self.current_angle = 0.0
        self.target_angle = 0.0
        self.has_target = False
        self.is_moving = False
        self.is_panning = False
        self.pan_direction = None
        self.reading_thread = None
        self.pan_refresh_thread = None
        self.running = False
        self.connection_validated = False
        self.validation_timeout = 3.0  # seconds to wait for response
        
        # Location and stations
        self.my_locator = ""
        self.my_lat = None
        self.my_lon = None
        self.stations = []  # List of (callsign, locator) tuples
        
        # Web server
        self.web_server = None
        self.web_server_thread = None
        self.web_server_port = 5000  # Default, will try others if blocked
        
        # Current settings
        self.current_speed = 4000
        self.current_accel = 2000
        
        # Pan refresh settings
        self.pan_refresh_interval = 0.5
        
        # Colors
        self.bg_color = "#2b2b2b"
        self.fg_color = "#ffffff"
        self.accent_color = "#4a9eff"
        self.button_color = "#3a3a3a"
        self.pan_active_color = "#ff6644"
        self.target_color = "#44ff44"
        self.compass_ring_color = "#303030"  # Dark ring on white
        self.compass_bg_color = "#f5f5f5"  # Light gray/white background
        self.compass_tick_color = "#000000"  # Black for graduations
        self.compass_cardinal_color = "#000000"  # Black for cardinal directions
        self.compass_cardinal_tick_color = "#dd0000"  # Red for cardinal graduations
        
        self.root.configure(bg=self.bg_color)
        
        self.create_widgets()
        self.load_settings()
        self.update_display()
        
        # Keyboard bindings for pan control
        self.root.bind('<less>', lambda e: self.start_pan_left())  # < key
        self.root.bind('<greater>', lambda e: self.start_pan_right())  # > key
        self.root.bind('<KeyRelease-less>', lambda e: self.stop_pan())
        self.root.bind('<KeyRelease-greater>', lambda e: self.stop_pan())
        
        # Also bind comma and period (unshifted < >) for easier access
        self.root.bind('<comma>', lambda e: self.start_pan_left())
        self.root.bind('<period>', lambda e: self.start_pan_right())
        self.root.bind('<KeyRelease-comma>', lambda e: self.stop_pan())
        self.root.bind('<KeyRelease-period>', lambda e: self.stop_pan())
        
        # Start web server and auto-connect after GUI is ready
        self.root.after(100, self.start_web_server)
        self.root.after(500, self.auto_connect)
        
    def create_widgets(self):
        # Main container with left panel and sidebar
        main_container = tk.Frame(self.root, bg=self.bg_color)
        main_container.pack(fill='both', expand=True)
        
        # Left panel (existing controls)
        left_panel = tk.Frame(main_container, bg=self.bg_color)
        left_panel.pack(side='left', fill='both', expand=True)
        
        # Right sidebar for stations
        sidebar = tk.Frame(main_container, bg="#1a1a1a", width=300)
        sidebar.pack(side='right', fill='y')
        sidebar.pack_propagate(False)
        
        # Menu bar
        menubar = tk.Menu(self.root)
        self.root.config(menu=menubar)
        
        settings_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Settings", menu=settings_menu)
        settings_menu.add_command(label="Motion Settings...", command=self.open_settings_dialog)
        settings_menu.add_command(label="Set Current Position...", command=self.open_set_position_dialog)
        settings_menu.add_separator()
        settings_menu.add_command(label="Set My Location...", command=self.open_location_dialog)
        
        # Serial Port Selection (in left panel)
        port_frame = tk.Frame(left_panel, bg=self.bg_color)
        port_frame.pack(pady=10, padx=20, fill='x')
        
        tk.Label(port_frame, text="Serial Port:", bg=self.bg_color, fg=self.fg_color, 
                font=('Arial', 10)).pack(side='left', padx=5)
        
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(port_frame, textvariable=self.port_var, 
                                       width=20, state='readonly')
        self.port_combo.pack(side='left', padx=5)
        self.refresh_ports()
        
        tk.Button(port_frame, text="Refresh", command=self.refresh_ports,
                 bg=self.button_color, fg=self.fg_color).pack(side='left', padx=5)
        
        self.connect_btn = tk.Button(port_frame, text="Connect", command=self.connect,
                 bg=self.accent_color, fg=self.fg_color, width=10)
        self.connect_btn.pack(side='left', padx=5)
        
        self.disconnect_btn = tk.Button(port_frame, text="Disconnect", command=self.disconnect,
                 bg=self.button_color, fg=self.fg_color, width=10, state='disabled')
        self.disconnect_btn.pack(side='left', padx=5)
        
        # Status
        self.status_label = tk.Label(left_panel, text="Disconnected", 
                                     bg=self.bg_color, fg="#ff4444", 
                                     font=('Arial', 10, 'bold'))
        self.status_label.pack(pady=5)
        
        # Compass Display - LARGER
        compass_frame = tk.Frame(left_panel, bg=self.bg_color)
        compass_frame.pack(pady=10)
        
        self.canvas = tk.Canvas(compass_frame, width=400, height=400, 
                               bg=self.compass_bg_color, highlightthickness=0)
        self.canvas.pack()
        
        # Mouse scroll wheel to adjust target heading
        self.canvas.bind('<MouseWheel>', self.on_compass_scroll)  # Windows
        self.canvas.bind('<Button-4>', lambda e: self.on_compass_scroll_linux(e, 1))  # Linux scroll up
        self.canvas.bind('<Button-5>', lambda e: self.on_compass_scroll_linux(e, -1))  # Linux scroll down
        
        # Angle Display - Below compass
        angle_frame = tk.Frame(left_panel, bg=self.bg_color)
        angle_frame.pack(pady=8)
        
        # Current angle
        current_frame = tk.Frame(angle_frame, bg=self.bg_color)
        current_frame.pack(side='left', padx=20)
        
        tk.Label(current_frame, text="CURRENT", bg=self.bg_color, 
                fg="#888888", font=('Arial', 9)).pack()
        
        self.angle_label = tk.Label(current_frame, text="0.00°", 
                                    bg=self.bg_color, fg=self.accent_color, 
                                    font=('Arial', 28, 'bold'))
        self.angle_label.pack()
        
        # Target angle
        target_frame = tk.Frame(angle_frame, bg=self.bg_color)
        target_frame.pack(side='left', padx=20)
        
        tk.Label(target_frame, text="TARGET", bg=self.bg_color, 
                fg="#888888", font=('Arial', 9)).pack()
        
        self.target_label = tk.Label(target_frame, text="---", 
                                     bg=self.bg_color, fg=self.target_color, 
                                     font=('Arial', 28, 'bold'))
        self.target_label.pack()
        
        # Error display
        self.error_label = tk.Label(left_panel, text="", 
                                   bg=self.bg_color, fg="#ffaa44", 
                                   font=('Arial', 10, 'bold'))
        self.error_label.pack(pady=5)
        
        # Go To Position
        goto_frame = tk.LabelFrame(left_panel, text="Go To Position", 
                                  bg=self.bg_color, fg=self.fg_color, 
                                  font=('Arial', 10, 'bold'))
        goto_frame.pack(pady=5, padx=20, fill='x')
        
        inner_frame2 = tk.Frame(goto_frame, bg=self.bg_color)
        inner_frame2.pack(pady=5)
        
        tk.Label(inner_frame2, text="Target:", bg=self.bg_color, fg=self.fg_color,
                font=('Arial', 10)).pack(side='left', padx=5)
        
        self.goto_entry = tk.Entry(inner_frame2, width=10, font=('Arial', 12))
        self.goto_entry.pack(side='left', padx=5)
        self.goto_entry.insert(0, "0")
        self.goto_entry.bind('<Return>', lambda e: self.goto_angle())
        
        tk.Label(inner_frame2, text="°", bg=self.bg_color, fg=self.fg_color,
                font=('Arial', 10)).pack(side='left')
        
        tk.Button(inner_frame2, text="Go", command=self.goto_angle,
                 bg=self.accent_color, fg=self.fg_color, width=8).pack(side='left', padx=10)
        
        # Continuous Pan Controls
        pan_frame = tk.LabelFrame(left_panel, text="Continuous Pan Control", 
                                 bg=self.bg_color, fg=self.fg_color, 
                                 font=('Arial', 10, 'bold'))
        pan_frame.pack(pady=5, padx=20, fill='x')
        
        button_frame = tk.Frame(pan_frame, bg=self.bg_color)
        button_frame.pack(pady=8)
        
        self.pan_left_btn = tk.Button(button_frame, text="◄◄◄ PAN LEFT", 
                 bg=self.button_color, fg=self.fg_color, width=20, height=3,
                 font=('Arial', 12, 'bold'))
        self.pan_left_btn.pack(side='left', padx=10)
        self.pan_left_btn.bind('<ButtonPress-1>', lambda e: self.start_pan_left())
        self.pan_left_btn.bind('<ButtonRelease-1>', lambda e: self.stop_pan())
        
        self.pan_right_btn = tk.Button(button_frame, text="PAN RIGHT ►►►", 
                 bg=self.button_color, fg=self.fg_color, width=20, height=3,
                 font=('Arial', 12, 'bold'))
        self.pan_right_btn.pack(side='left', padx=10)
        self.pan_right_btn.bind('<ButtonPress-1>', lambda e: self.start_pan_right())
        self.pan_right_btn.bind('<ButtonRelease-1>', lambda e: self.stop_pan())
        
        tk.Label(pan_frame, text="Hold button to pan • Watchdog protected", 
                bg=self.bg_color, fg="#888888", font=('Arial', 8, 'italic')).pack(pady=5)
        
        # Quick positions
        quick_frame = tk.LabelFrame(left_panel, text="Quick Positions", 
                                   bg=self.bg_color, fg=self.fg_color, 
                                   font=('Arial', 10, 'bold'))
        quick_frame.pack(pady=5, padx=20, fill='x')
        
        quick_button_frame = tk.Frame(quick_frame, bg=self.bg_color)
        quick_button_frame.pack(pady=5)
        
        positions = [("N", 0), ("NE", 45), ("E", 90), ("SE", 135), 
                    ("S", 180), ("SW", 225), ("W", 270), ("NW", 315)]
        
        for i, (label, angle) in enumerate(positions):
            tk.Button(quick_button_frame, text=label, 
                     command=lambda a=angle: self.goto_angle_direct(a),
                     bg=self.button_color, fg=self.fg_color, width=4, height=1).grid(
                         row=i//4, column=i%4, padx=3, pady=3)
        
        # Home and Stop buttons
        control_frame = tk.Frame(left_panel, bg=self.bg_color)
        control_frame.pack(pady=5)
        
        tk.Button(control_frame, text="HOME (0°)", command=self.home,
                 bg="#44aa44", fg=self.fg_color, width=12, height=2,
                 font=('Arial', 11, 'bold')).pack(side='left', padx=5)
        
        tk.Button(control_frame, text="STOP", command=self.emergency_stop,
                 bg="#ff4444", fg=self.fg_color, width=12, height=2,
                 font=('Arial', 11, 'bold')).pack(side='left', padx=5)
        
        # ===== SIDEBAR =====
        # Sidebar header
        sidebar_header = tk.Frame(sidebar, bg="#1a1a1a")
        sidebar_header.pack(fill='x', padx=5, pady=5)
        
        tk.Label(sidebar_header, text="Stations", bg="#1a1a1a", fg="#ffffff",
                font=('Arial', 12, 'bold')).pack(side='left', padx=5)
        
        # Web server status indicator in its own frame
        web_status_frame = tk.Frame(sidebar_header, bg="#1a1a1a")
        web_status_frame.pack(side='right', padx=5)
        
        self.web_status_label = tk.Label(web_status_frame, text="API", 
                                         bg="#1a1a1a", fg="#ffaa44", 
                                         font=('Arial', 9, 'bold'))
        self.web_status_label.pack(side='left')
        
        self.web_status_dot = tk.Label(web_status_frame, text="⬤", 
                                       bg="#1a1a1a", fg="#888888", 
                                       font=('Arial', 14))
        self.web_status_dot.pack(side='left', padx=(3, 0))
        
        # My location display
        self.my_loc_label = tk.Label(sidebar, text="My QTH: Not Set", 
                                     bg="#1a1a1a", fg="#888888", 
                                     font=('Arial', 9, 'italic'))
        self.my_loc_label.pack(padx=5, pady=5)
        
        # Buttons frame
        button_frame = tk.Frame(sidebar, bg="#1a1a1a")
        button_frame.pack(fill='x', padx=5, pady=5)
        
        tk.Button(button_frame, text="Add", command=self.add_station,
                 bg="#4a9eff", fg="#ffffff", width=8, font=('Arial', 9)).pack(side='left', padx=2)
        
        tk.Button(button_frame, text="Edit", command=self.edit_station,
                 bg="#3a3a3a", fg="#ffffff", width=8, font=('Arial', 9)).pack(side='left', padx=2)
        
        tk.Button(button_frame, text="Delete", command=self.delete_station,
                 bg="#3a3a3a", fg="#ffffff", width=8, font=('Arial', 9)).pack(side='left', padx=2)
        
        # Stations list
        list_frame = tk.Frame(sidebar, bg="#1a1a1a")
        list_frame.pack(fill='both', expand=True, padx=5, pady=5)
        
        # Scrollbar
        scrollbar = tk.Scrollbar(list_frame)
        scrollbar.pack(side='right', fill='y')
        
        # Listbox
        self.stations_listbox = tk.Listbox(list_frame, yscrollcommand=scrollbar.set,
                                          bg="#2a2a2a", fg="#ffffff",
                                          selectbackground="#4a9eff",
                                          font=('Courier', 9), height=25)
        self.stations_listbox.pack(side='left', fill='both', expand=True)
        self.stations_listbox.bind('<Double-Button-1>', lambda e: self.goto_station())
        
        scrollbar.config(command=self.stations_listbox.yview)
        
        # Go to station button
        tk.Button(sidebar, text="Go To Selected Station", command=self.goto_station,
                 bg="#44aa44", fg="#ffffff", font=('Arial', 10, 'bold')).pack(fill='x', padx=5, pady=5)
    
    def on_compass_scroll(self, event):
        """Handle mouse scroll wheel on compass (Windows)"""
        # event.delta is positive for scroll up, negative for scroll down
        # Scroll up = increase angle (clockwise), scroll down = decrease (counter-clockwise)
        step = 1 if event.delta > 0 else -1
        self.adjust_target_by_scroll(step)
    
    def on_compass_scroll_linux(self, event, direction):
        """Handle mouse scroll wheel on compass (Linux)"""
        self.adjust_target_by_scroll(direction)
    
    def adjust_target_by_scroll(self, step):
        """Adjust target heading by scroll step"""
        if not self.serial_port or not self.serial_port.is_open:
            return
        
        # If no target set, start from current angle
        if not self.has_target:
            new_target = self.current_angle + step
        else:
            new_target = self.target_angle + step
        
        # Normalize to 0-360
        new_target = new_target % 360
        if new_target < 0:
            new_target += 360
        
        self.target_angle = new_target
        self.has_target = True
        self.send_command(f"A{new_target}")
        
    def draw_compass(self):
        self.canvas.delete("all")
        
        cx, cy = 200, 200  # Center point
        radius = 160  # Radius
        
        # White/light background circle
        self.canvas.create_oval(cx-radius-10, cy-radius-10, 
                               cx+radius+10, cy+radius+10,
                               fill=self.compass_bg_color, outline="")
        
        # Outer ring - dark on white
        self.canvas.create_oval(cx-radius, cy-radius, cx+radius, cy+radius,
                               outline=self.compass_ring_color, width=4)
        
        # Inner ring
        self.canvas.create_oval(cx-radius+15, cy-radius+15, cx+radius-15, cy+radius-15,
                               outline="#505050", width=2)
        
        # Degree markings
        for angle in range(0, 360, 5):
            rad = math.radians(angle - 90)
            
            # Check if this is a cardinal direction
            is_cardinal = (angle % 90 == 0)
            tick_color = self.compass_cardinal_tick_color if is_cardinal else self.compass_tick_color
            
            if angle % 30 == 0:
                # Major tick marks
                x1 = cx + (radius - 12) * math.cos(rad)
                y1 = cy + (radius - 12) * math.sin(rad)
                x2 = cx + (radius - 28) * math.cos(rad)
                y2 = cy + (radius - 28) * math.sin(rad)
                self.canvas.create_line(x1, y1, x2, y2, fill=tick_color, width=3)
                
                # Degree numbers - black (skip cardinals)
                if angle % 90 != 0:
                    x_text = cx + (radius - 45) * math.cos(rad)
                    y_text = cy + (radius - 45) * math.sin(rad)
                    self.canvas.create_text(x_text, y_text, text=str(angle), 
                                           fill="#202020", font=('Arial', 10, 'bold'))
            elif angle % 10 == 0:
                # Medium tick marks - always black
                x1 = cx + (radius - 12) * math.cos(rad)
                y1 = cy + (radius - 12) * math.sin(rad)
                x2 = cx + (radius - 22) * math.cos(rad)
                y2 = cy + (radius - 22) * math.sin(rad)
                self.canvas.create_line(x1, y1, x2, y2, fill=self.compass_tick_color, width=2)
            else:
                # Minor tick marks - always black
                x1 = cx + (radius - 12) * math.cos(rad)
                y1 = cy + (radius - 12) * math.sin(rad)
                x2 = cx + (radius - 18) * math.cos(rad)
                y2 = cy + (radius - 18) * math.sin(rad)
                self.canvas.create_line(x1, y1, x2, y2, fill=self.compass_tick_color, width=1)
        
        # Cardinal directions - BLACK text on white
        directions = [("N", 0), ("E", 90), ("S", 180), ("W", 270)]
        for label, angle in directions:
            rad = math.radians(angle - 90)
            x = cx + (radius + 30) * math.cos(rad)
            y = cy + (radius + 30) * math.sin(rad)
            
            # White outline for contrast
            for dx, dy in [(-1,-1), (-1,1), (1,-1), (1,1), (0,-2), (0,2), (-2,0), (2,0)]:
                self.canvas.create_text(x+dx, y+dy, text=label, fill="#ffffff", 
                                       font=('Arial', 20, 'bold'))
            # Main text in BLACK
            self.canvas.create_text(x, y, text=label, fill=self.compass_cardinal_color, 
                                   font=('Arial', 20, 'bold'))
        
        # Target indicator
        if self.has_target:
            target_rad = math.radians(self.target_angle - 90)
            
            # Target line from center
            target_x_end = cx + (radius - 10) * math.cos(target_rad)
            target_y_end = cy + (radius - 10) * math.sin(target_rad)
            
            # Dashed line to target
            self.canvas.create_line(cx, cy, target_x_end, target_y_end, 
                                   fill=self.target_color, width=2, dash=(8, 4))
            
            # Target marker - glowing circle
            target_x = cx + (radius - 10) * math.cos(target_rad)
            target_y = cy + (radius - 10) * math.sin(target_rad)
            
            # Glow effect
            self.canvas.create_oval(target_x-16, target_y-16, target_x+16, target_y+16,
                                   fill="", outline=self.target_color, width=1)
            self.canvas.create_oval(target_x-12, target_y-12, target_x+12, target_y+12,
                                   fill="", outline=self.target_color, width=2)
            # Center dot
            self.canvas.create_oval(target_x-8, target_y-8, target_x+8, target_y+8,
                                   fill=self.target_color, outline=self.target_color)
        
        # Current position pointer - enhanced arrow
        angle_rad = math.radians(self.current_angle - 90)
        pointer_length = radius - 40
        px = cx + pointer_length * math.cos(angle_rad)
        py = cy + pointer_length * math.sin(angle_rad)
        
        pointer_color = self.pan_active_color if self.is_panning else self.accent_color
        
        # Pointer shadow
        shadow_px = px + 2
        shadow_py = py + 2
        self.canvas.create_line(cx+2, cy+2, shadow_px, shadow_py, 
                               fill="#888888", width=6, arrow=tk.LAST, 
                               arrowshape=(20, 25, 8))
        
        # Main pointer
        self.canvas.create_line(cx, cy, px, py, fill=pointer_color, width=5,
                               arrow=tk.LAST, arrowshape=(20, 25, 8))
        
        # Center hub - layered circles for depth
        self.canvas.create_oval(cx-15, cy-15, cx+15, cy+15, 
                               fill="#e0e0e0", outline="#404040", width=2)
        self.canvas.create_oval(cx-10, cy-10, cx+10, cy+10, 
                               fill="#f0f0f0", outline=pointer_color, width=2)
        self.canvas.create_oval(cx-5, cy-5, cx+5, cy+5, 
                               fill=pointer_color, outline=pointer_color)
        
    def _blend_colors(self, color1, color2, factor):
        """Simple color blending helper"""
        try:
            r1, g1, b1 = int(color1[1:3], 16), int(color1[3:5], 16), int(color1[5:7], 16)
            r2, g2, b2 = int(color2[1:3], 16), int(color2[3:5], 16), int(color2[5:7], 16)
            r = int(r1 + (r2 - r1) * factor)
            g = int(g1 + (g2 - g1) * factor)
            b = int(b1 + (b2 - b1) * factor)
            return f"#{r:02x}{g:02x}{b:02x}"
        except:
            return color1
    
    def load_settings(self):
        """Load saved settings from JSON file"""
        try:
            if os.path.exists(self.SETTINGS_FILE):
                with open(self.SETTINGS_FILE, 'r') as f:
                    settings = json.load(f)
                    last_port = settings.get('last_port', '')
                    if last_port:
                        self.port_var.set(last_port)
                        print(f"Loaded last port: {last_port}")
                    
                    # Load location
                    self.my_locator = settings.get('my_locator', '')
                    if self.my_locator:
                        self.my_lat, self.my_lon = maidenhead_to_latlon(self.my_locator)
                        self.my_loc_label.config(text=f"My QTH: {self.my_locator}")
                    
                    # Load stations
                    self.stations = settings.get('stations', [])
                    self.update_stations_list()
        except Exception as e:
            print(f"Error loading settings: {e}")
    
    def save_settings(self):
        """Save current settings to JSON file"""
        try:
            settings = {
                'last_port': self.port_var.get(),
                'my_locator': self.my_locator,
                'stations': self.stations
            }
            with open(self.SETTINGS_FILE, 'w') as f:
                json.dump(settings, f, indent=2)
        except Exception as e:
            print(f"Error saving settings: {e}")
    
    def auto_connect(self):
        """Attempt to connect to the last used port automatically"""
        last_port = self.port_var.get()
        if last_port:
            # Check if the port still exists
            available_ports = [port.device for port in serial.tools.list_ports.comports()]
            if last_port in available_ports:
                print(f"Auto-connecting to {last_port}...")
                # Temporarily store the original messagebox function
                original_showerror = messagebox.showerror
                original_showwarning = messagebox.showwarning
                
                # Suppress error popups during auto-connect
                messagebox.showerror = lambda *args, **kwargs: None
                messagebox.showwarning = lambda *args, **kwargs: None
                
                try:
                    self.connect()
                finally:
                    # Restore original messagebox functions
                    messagebox.showerror = original_showerror
                    messagebox.showwarning = original_showwarning
            else:
                print(f"Last port {last_port} not available")
                self.status_label.config(text=f"Last port ({last_port}) not found", fg="#ffaa44")
    
    def calculate_angle_error(self):
        if not self.has_target:
            return 0
        
        error = self.target_angle - self.current_angle
        
        while error > 180:
            error -= 360
        while error < -180:
            error += 360
        
        return error
    
    def open_settings_dialog(self):
        dialog = SettingsDialog(self.root, self.current_speed, self.current_accel)
        self.root.wait_window(dialog.dialog)
        
        if dialog.result:
            speed, accel = dialog.result
            self.send_command(f"S{speed}")
            self.send_command(f"AC{accel}")
    
    def open_set_position_dialog(self):
        dialog = SetPositionDialog(self.root, self.current_angle)
        self.root.wait_window(dialog.dialog)
        
        if dialog.result is not None:
            angle = dialog.result
            self.send_command(f"SETPOS{angle}")
            self.current_angle = angle
            self.has_target = False
    
    def open_location_dialog(self):
        dialog = LocationDialog(self.root, self.my_locator)
        self.root.wait_window(dialog.dialog)
        
        if dialog.result:
            self.my_locator = dialog.result
            self.my_lat, self.my_lon = maidenhead_to_latlon(self.my_locator)
            self.my_loc_label.config(text=f"My QTH: {self.my_locator}")
            self.save_settings()
            self.update_stations_list()  # Refresh to recalculate bearings
    
    def update_stations_list(self):
        """Update the stations listbox with bearings and distances"""
        self.stations_listbox.delete(0, tk.END)
        
        for callsign, locator in self.stations:
            if self.my_lat is not None and self.my_lon is not None:
                # Calculate bearing and distance
                lat, lon = maidenhead_to_latlon(locator)
                if lat is not None:
                    bearing = calculate_bearing(self.my_lat, self.my_lon, lat, lon)
                    distance = calculate_distance(self.my_lat, self.my_lon, lat, lon)
                    
                    # Format: CALLSIGN   LOCATOR  BEARING  DIST
                    line = f"{callsign:12s} {locator:8s} {bearing:6.1f}° {distance:5.0f}km"
                    self.stations_listbox.insert(tk.END, line)
                else:
                    line = f"{callsign:12s} {locator:8s} ---"
                    self.stations_listbox.insert(tk.END, line)
            else:
                # No home location set
                line = f"{callsign:12s} {locator:8s} (Set My Location)"
                self.stations_listbox.insert(tk.END, line)
    
    def add_station(self):
        dialog = StationDialog(self.root)
        self.root.wait_window(dialog.dialog)
        
        if dialog.result:
            callsign, locator = dialog.result
            self.stations.append((callsign, locator))
            self.save_settings()
            self.update_stations_list()
    
    def edit_station(self):
        selection = self.stations_listbox.curselection()
        if not selection:
            messagebox.showwarning("No Selection", "Please select a station to edit")
            return
        
        idx = selection[0]
        callsign, locator = self.stations[idx]
        
        dialog = StationDialog(self.root, callsign, locator)
        self.root.wait_window(dialog.dialog)
        
        if dialog.result:
            callsign, locator = dialog.result
            self.stations[idx] = (callsign, locator)
            self.save_settings()
            self.update_stations_list()
    
    def delete_station(self):
        selection = self.stations_listbox.curselection()
        if not selection:
            messagebox.showwarning("No Selection", "Please select a station to delete")
            return
        
        idx = selection[0]
        callsign, locator = self.stations[idx]
        
        if messagebox.askyesno("Confirm Delete", f"Delete {callsign} ({locator})?"):
            del self.stations[idx]
            self.save_settings()
            self.update_stations_list()
    
    def goto_station(self):
        selection = self.stations_listbox.curselection()
        if not selection:
            messagebox.showwarning("No Selection", "Please select a station")
            return
        
        if self.my_lat is None or self.my_lon is None:
            messagebox.showwarning("Location Not Set", "Please set your location first in Settings > Set My Location")
            return
        
        idx = selection[0]
        callsign, locator = self.stations[idx]
        
        lat, lon = maidenhead_to_latlon(locator)
        if lat is None:
            messagebox.showerror("Error", f"Invalid locator for {callsign}")
            return
        
        bearing = calculate_bearing(self.my_lat, self.my_lon, lat, lon)
        self.goto_angle_direct(bearing)
    
    def start_web_server(self):
        """Start the Flask web server in a separate thread"""
        try:
            # Check if Flask is available
            try:
                from flask import Flask, request, jsonify
                from werkzeug.serving import make_server
            except ImportError:
                print("Flask not installed. Web server disabled.")
                print("Install with: pip install flask")
                return
            
            app = Flask(__name__)
            app.logger.disabled = True  # Disable Flask logging
            
            # CORS support - add headers to all responses
            @app.after_request
            def add_cors_headers(response):
                response.headers['Access-Control-Allow-Origin'] = '*'
                response.headers['Access-Control-Allow-Methods'] = 'GET, POST, PUT, DELETE, OPTIONS'
                response.headers['Access-Control-Allow-Headers'] = 'Content-Type, Authorization, X-Requested-With'
                return response
            
            # Handle preflight OPTIONS requests
            @app.route('/station', methods=['OPTIONS'])
            @app.route('/status', methods=['OPTIONS'])
            def handle_options():
                return '', 204
            
            @app.route('/station', methods=['POST'])
            def add_station_endpoint():
                try:
                    data = request.get_json()
                    
                    if not data:
                        return jsonify({'error': 'No JSON data provided'}), 400
                    
                    callsign = data.get('callsign', '').upper().strip()
                    locator = data.get('locator', '').upper().strip()
                    
                    if not callsign or not locator:
                        return jsonify({'error': 'Both callsign and locator are required'}), 400
                    
                    # Validate locator
                    if not re.match(r'^[A-R]{2}[0-9]{2}([A-X]{2})?([0-9]{2})?$', locator):
                        return jsonify({'error': 'Invalid Maidenhead locator format'}), 400
                    
                    lat, lon = maidenhead_to_latlon(locator)
                    if lat is None:
                        return jsonify({'error': 'Invalid Maidenhead locator'}), 400
                    
                    # Check if station already exists
                    existing_idx = None
                    for i, (existing_call, existing_loc) in enumerate(self.stations):
                        if existing_call == callsign:
                            existing_idx = i
                            break
                    
                    if existing_idx is not None:
                        # Station exists - ask user if they want to update
                        self.root.after(0, lambda: self.handle_existing_station(callsign, locator, existing_idx))
                        return jsonify({
                            'status': 'pending',
                            'message': f'Station {callsign} already exists. Update request sent to user.'
                        }), 200
                    else:
                        # New station - add it
                        self.root.after(0, lambda: self.add_station_from_web(callsign, locator))
                        return jsonify({
                            'status': 'added',
                            'callsign': callsign,
                            'locator': locator
                        }), 201
                
                except Exception as e:
                    return jsonify({'error': str(e)}), 500
            
            @app.route('/status', methods=['GET'])
            def status():
                return jsonify({
                    'status': 'running',
                    'my_locator': self.my_locator if self.my_locator else None,
                    'stations_count': len(self.stations),
                    'rotator_connected': self.serial_port is not None and self.serial_port.is_open,
                    'current_angle': self.current_angle
                }), 200
            
            # Try multiple ports in case one is blocked
            ports_to_try = [5000, 5001, 8000, 8001, 3000]
            server_started = False
            
            for port in ports_to_try:
                try:
                    self.web_server = make_server('127.0.0.1', port, app, threaded=True)
                    self.web_server_port = port
                    server_started = True
                    break
                except OSError as e:
                    print(f"Port {port} unavailable: {e}")
                    continue
            
            if not server_started:
                raise Exception("No available ports found")
            
            # Start server in thread
            self.web_server_thread = threading.Thread(target=self.web_server.serve_forever, daemon=True)
            self.web_server_thread.start()
            
            print(f"Web server started on http://127.0.0.1:{self.web_server_port}")
            
            # Update label and dot on main thread
            def update_status():
                label_text = f":{self.web_server_port}"
                print(f"Updating web status to: API{label_text}")
                self.web_status_label.config(text=f"API{label_text}", fg="#44ff44")
                self.web_status_dot.config(fg="#44ff44")
            
            self.root.after(0, update_status)
            
        except Exception as e:
            print(f"Failed to start web server: {e}")
            def update_error():
                self.web_status_label.config(text="API", fg="#ff4444")
                self.web_status_dot.config(fg="#ff4444")
            self.root.after(0, update_error)
    
    def add_station_from_web(self, callsign, locator):
        """Add a station from web request (called in main thread)"""
        self.stations.append((callsign, locator))
        self.save_settings()
        self.update_stations_list()
        print(f"Added station from web: {callsign} ({locator})")
    
    def handle_existing_station(self, callsign, locator, existing_idx):
        """Handle update request for existing station (called in main thread)"""
        existing_call, existing_loc = self.stations[existing_idx]
        
        response = messagebox.askyesnocancel(
            "Station Exists",
            f"Station {callsign} already exists with locator {existing_loc}.\n\n"
            f"Update to new locator {locator}?\n\n"
            f"Yes = Update | No = Keep existing | Cancel = Ignore",
            icon='question'
        )
        
        if response is True:  # Yes - update
            self.stations[existing_idx] = (callsign, locator)
            self.save_settings()
            self.update_stations_list()
            print(f"Updated station from web: {callsign} ({locator})")
        elif response is False:  # No - keep existing
            print(f"Kept existing station: {callsign} ({existing_loc})")
        else:  # Cancel - ignore
            print(f"Ignored web request for {callsign}")
    
    def stop_web_server(self):
        """Stop the web server"""
        if hasattr(self, 'web_server') and self.web_server:
            try:
                self.web_server.shutdown()
                print("Web server stopped")
            except Exception as e:
                print(f"Error stopping web server: {e}")
        
    def refresh_ports(self):
        ports = serial.tools.list_ports.comports()
        port_list = [port.device for port in ports]
        self.port_combo['values'] = port_list
        if port_list:
            self.port_combo.current(0)
    
    def connect(self):
        try:
            port = self.port_var.get()
            if not port:
                messagebox.showerror("Error", "Please select a port")
                return
            
            # Disable connect button, enable disconnect
            self.connect_btn.config(state='disabled')
            self.disconnect_btn.config(state='normal')
            
            self.serial_port = serial.Serial(port, 19200, timeout=1)
            time.sleep(2)
            
            self.status_label.config(text="Connecting - Validating...", fg="#ffaa44")
            
            # Save the port setting on connection attempt
            self.save_settings()
            
            self.running = True
            self.connection_validated = False
            self.reading_thread = threading.Thread(target=self.read_serial, daemon=True)
            self.reading_thread.start()
            
            # Request position to validate connection
            time.sleep(0.5)
            self.send_command("P")
            
            # Start validation check
            self.root.after(int(self.validation_timeout * 1000), self.check_connection_validation)
            
        except Exception as e:
            self.status_label.config(text="Connection Failed", fg="#ff4444")
            self.connect_btn.config(state='normal')
            self.disconnect_btn.config(state='disabled')
            messagebox.showerror("Connection Error", str(e))
    
    def check_connection_validation(self):
        """Check if we received a valid response from the controller"""
        if not self.connection_validated:
            # No response received - disconnect
            self.status_label.config(text="No Response - Disconnecting", fg="#ff4444")
            print("Connection validation failed - no response from controller")
            self.disconnect()
            messagebox.showwarning("Connection Failed", 
                                  "Connected to port but received no response from controller.\n"
                                  "Please check that the rotator controller is powered on and running.")
        else:
            print("Connection validated successfully")
    
    def disconnect(self):
        self.running = False
        self.connection_validated = False
        self.stop_pan()
        if self.serial_port and self.serial_port.is_open:
            self.send_command("PANSTOP")
            self.send_command("SAVE")  # Save before disconnect
            time.sleep(0.2)
            self.serial_port.close()
        self.status_label.config(text="Disconnected", fg="#ff4444")
        
        # Re-enable connect button, disable disconnect
        self.connect_btn.config(state='normal')
        self.disconnect_btn.config(state='disabled')
    
    def read_serial(self):
        while self.running:
            try:
                if self.serial_port and self.serial_port.is_open:
                    if self.serial_port.in_waiting:
                        line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                        self.parse_response(line)
                time.sleep(0.02)
            except Exception as e:
                print(f"Read error: {e}")
                break
    
    def parse_response(self, line):
        if not line:
            return
        
        # Mark connection as validated on any valid response
        if not self.connection_validated and (line.startswith("Position:") or 
                                              line.startswith("GO:") or 
                                              line.startswith("SET:") or 
                                              line.startswith("SPD:") or 
                                              line.startswith("ACC:") or 
                                              "Loaded Speed:" in line or 
                                              "Loaded Accel:" in line):
            self.connection_validated = True
            self.status_label.config(text="Connected - EEPROM Enabled", fg="#44ff44")
            print("Connection validated - received response from controller")
            
        if line.startswith("Position:"):
            try:
                angle_str = line.split(":")[1].strip().rstrip("°")
                angle = float(angle_str)
                self.current_angle = angle
            except Exception as e:
                print(f"Parse error: {e}")
        
        elif line.startswith("GO:"):
            try:
                angle = float(line.split(":")[1])
                self.target_angle = angle
                self.has_target = True
            except:
                pass
        
        elif line.startswith("PAN:"):
            direction = line.split(":")[1]
            self.is_panning = True
            self.has_target = False
            self.pan_direction = 'left' if direction == 'L' else 'right'
            self.update_button_colors()
        
        elif line.startswith("SET:"):
            try:
                angle = float(line.split(":")[1])
                self.current_angle = angle
                self.has_target = False
            except:
                pass
        
        elif line.startswith("SPD:"):
            try:
                speed = int(float(line.split(":")[1]))
                self.current_speed = speed
            except:
                pass
        
        elif line.startswith("ACC:"):
            try:
                accel = int(float(line.split(":")[1]))
                self.current_accel = accel
            except:
                pass
        
        elif "Loaded Speed:" in line:
            try:
                speed = int(float(line.split(":")[1].strip()))
                self.current_speed = speed
            except:
                pass
        
        elif "Loaded Accel:" in line:
            try:
                accel = int(float(line.split(":")[1].strip()))
                self.current_accel = accel
            except:
                pass
        
        if line and not line.startswith("Position:"):
            print(f"Arduino: {line}")
    
    def send_command(self, cmd):
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(f"{cmd}\n".encode())
            except Exception as e:
                print(f"Send error: {e}")
        else:
            if cmd not in ["PANSTOP", "SAVE"]:
                messagebox.showwarning("Not Connected", "Please connect first")
    
    def pan_refresh_worker(self):
        while self.is_panning and self.pan_direction:
            if self.pan_direction == 'left':
                self.send_command("PANLEFT")
            elif self.pan_direction == 'right':
                self.send_command("PANRIGHT")
            
            time.sleep(self.pan_refresh_interval)
    
    def start_pan_left(self):
        if not self.is_panning:
            self.is_panning = True
            self.pan_direction = 'left'
            self.has_target = False
            self.send_command("PANLEFT")
            self.update_button_colors()
            
            self.pan_refresh_thread = threading.Thread(target=self.pan_refresh_worker, daemon=True)
            self.pan_refresh_thread.start()
    
    def start_pan_right(self):
        if not self.is_panning:
            self.is_panning = True
            self.pan_direction = 'right'
            self.has_target = False
            self.send_command("PANRIGHT")
            self.update_button_colors()
            
            self.pan_refresh_thread = threading.Thread(target=self.pan_refresh_worker, daemon=True)
            self.pan_refresh_thread.start()
    
    def stop_pan(self):
        if self.is_panning:
            self.is_panning = False
            self.pan_direction = None
            self.send_command("PANSTOP")
            self.update_button_colors()
    
    def update_button_colors(self):
        if self.is_panning:
            if self.pan_direction == 'left':
                self.pan_left_btn.config(bg=self.pan_active_color)
                self.pan_right_btn.config(bg=self.button_color)
            elif self.pan_direction == 'right':
                self.pan_right_btn.config(bg=self.pan_active_color)
                self.pan_left_btn.config(bg=self.button_color)
        else:
            self.pan_left_btn.config(bg=self.button_color)
            self.pan_right_btn.config(bg=self.button_color)
    
    def goto_angle(self):
        try:
            angle = float(self.goto_entry.get())
            self.goto_angle_direct(angle)
        except ValueError:
            messagebox.showerror("Error", "Invalid angle value")
    
    def goto_angle_direct(self, angle):
        if 0 <= angle < 360:
            self.target_angle = angle
            self.has_target = True
            self.send_command(f"A{angle}")
        else:
            messagebox.showerror("Error", "Angle must be between 0 and 360")
    
    def home(self):
        self.target_angle = 0
        self.has_target = True
        self.send_command("H")
    
    def emergency_stop(self):
        self.stop_pan()
        self.send_command("STOP")
        self.has_target = False
    
    def update_display(self):
        self.angle_label.config(text=f"{self.current_angle:.2f}°")
        
        if self.has_target:
            self.target_label.config(text=f"{self.target_angle:.2f}°")
            
            error = self.calculate_angle_error()
            if abs(error) > 0.5:
                self.error_label.config(text=f"Error: {error:+.1f}°")
            else:
                self.error_label.config(text="✓ On Target", fg="#44ff44")
        else:
            self.target_label.config(text="---")
            self.error_label.config(text="")
        
        self.draw_compass()
        
        self.root.after(100, self.update_display)
    
    def on_closing(self):
        self.stop_web_server()
        self.disconnect()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = RotatorGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()

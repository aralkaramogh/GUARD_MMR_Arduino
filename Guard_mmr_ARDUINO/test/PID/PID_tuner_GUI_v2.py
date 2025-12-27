"""
Arduino MEGA Single Motor PID Tuner - Per-Motor Tuning
Tune one motor at a time with live visualization
"""

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import collections
import threading
import time
import json
from datetime import datetime

class SingleMotorPIDTuner:
    def __init__(self, root):
        self.root = root
        self.root.title("Arduino MEGA - Single Motor PID Tuner")
        self.root.geometry("1300x850")
        
        # Serial connection
        self.ser = None
        self.is_connected = False
        self.read_thread = None
        self.running = False
        
        # Motor selection
        self.selected_motor = 0  # 0-3 for motors 1-4
        self.motor_names = ["Motor 1 (RF)", "Motor 2 (LF)", "Motor 3 (RB)", "Motor 4 (LB)"]
        
        # Data storage per motor
        self.motor_data = {
            i: {
                'targets': collections.deque([0]*300, maxlen=300),
                'actuals': collections.deque([0]*300, maxlen=300),
                'pwms': collections.deque([0]*300, maxlen=300),
                'errors': collections.deque([0]*300, maxlen=300),
                'kp': 0.8, 'ki': 0.5, 'kd': 0.05, 'ff': 0.85,
                'stats': {'ss_error': 0, 'overshoot': 0, 'peak': 0}
            }
            for i in range(4)
        }
        
        self.setup_ui()
        
    def setup_ui(self):
        # ===== TOP FRAME: Connection =====
        top_frame = ttk.Frame(self.root)
        top_frame.pack(fill="x", padx=10, pady=5)
        
        conn_frame = ttk.LabelFrame(top_frame, text="Connection")
        conn_frame.pack(side="left", fill="x", expand=True, padx=5)
        
        ttk.Label(conn_frame, text="Port:").pack(side="left", padx=5)
        self.port_combo = ttk.Combobox(conn_frame, values=self.get_ports(), width=15)
        self.port_combo.pack(side="left", padx=5)
        if self.port_combo['values']:
            self.port_combo.current(0)
        
        ttk.Button(conn_frame, text="Refresh", command=self.refresh_ports).pack(side="left", padx=2)
        self.btn_connect = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.btn_connect.pack(side="left", padx=5)
        
        self.status_label = ttk.Label(conn_frame, text="● Disconnected", foreground="red", font=("Arial", 10, "bold"))
        self.status_label.pack(side="left", padx=10)
        
        # ===== MOTOR SELECTION =====
        motor_frame = ttk.LabelFrame(top_frame, text="Select Motor to Tune")
        motor_frame.pack(side="right", padx=5)
        
        self.motor_var = tk.IntVar(value=0)
        for i, name in enumerate(self.motor_names):
            rb = ttk.Radiobutton(motor_frame, text=name, variable=self.motor_var, value=i, 
                                command=self.on_motor_select)
            rb.pack(side="left", padx=5)
        
        # ===== MAIN CONTENT =====
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill="both", expand=True, padx=10, pady=5)
        
        # ===== LEFT PANEL: Controls =====
        left_panel = ttk.Frame(main_frame, width=380)
        left_panel.pack(side="left", fill="y", padx=5)
        left_panel.pack_propagate(False)
        
        # Motor status banner
        self.motor_status_frame = ttk.LabelFrame(left_panel, text="Selected Motor Status")
        self.motor_status_frame.pack(fill="x", pady=5)
        
        self.motor_info_label = ttk.Label(self.motor_status_frame, text="Motor 1 (RF)\nTarget: 0 RPM | Actual: 0 RPM",
                                         font=("Arial", 10, "bold"), foreground="blue")
        self.motor_info_label.pack(fill="x", padx=10, pady=5)
        
        # PID Controls for selected motor
        pid_frame = ttk.LabelFrame(left_panel, text="PID Tuning (Selected Motor)")
        pid_frame.pack(fill="x", pady=5)
        
        self.kp_var = self.create_param_control(pid_frame, "Kp (Proportional)", 'P', 0, 5.0, 0.8)
        self.ki_var = self.create_param_control(pid_frame, "Ki (Integral)", 'I', 0, 3.0, 0.5)
        self.kd_var = self.create_param_control(pid_frame, "Kd (Derivative)", 'D', 0, 1.0, 0.05)
        self.ff_var = self.create_param_control(pid_frame, "Feed-Forward", 'F', 0, 2.0, 0.85)
        
        # Preset buttons
        preset_frame = ttk.Frame(pid_frame)
        preset_frame.pack(fill="x", pady=5, padx=5)
        ttk.Button(preset_frame, text="Conservative", command=self.load_conservative).pack(side="left", padx=2)
        ttk.Button(preset_frame, text="Balanced", command=self.load_balanced).pack(side="left", padx=2)
        ttk.Button(preset_frame, text="Aggressive", command=self.load_aggressive).pack(side="left", padx=2)
        
        # Copy to all motors
        copy_frame = ttk.Frame(pid_frame)
        copy_frame.pack(fill="x", pady=5, padx=5)
        ttk.Button(copy_frame, text="📋 Copy to All Motors", 
                  command=self.copy_to_all_motors).pack(fill="x")
        
        # Target RPM Control
        target_frame = ttk.LabelFrame(left_panel, text="Target Control")
        target_frame.pack(fill="x", pady=5)
        
        self.target_var = tk.DoubleVar(value=0)
        
        target_slider = ttk.Scale(target_frame, from_=0, to=187, variable=self.target_var, 
                                  orient="horizontal", command=self.on_target_change)
        target_slider.pack(fill="x", padx=5)
        
        target_entry_frame = ttk.Frame(target_frame)
        target_entry_frame.pack(fill="x", padx=5, pady=5)
        
        ttk.Label(target_entry_frame, text="Target RPM:").pack(side="left")
        self.target_entry = ttk.Entry(target_entry_frame, textvariable=self.target_var, width=10)
        self.target_entry.pack(side="left", padx=5)
        self.target_entry.bind('<Return>', lambda e: self.send_target())
        
        ttk.Button(target_entry_frame, text="Set", command=self.send_target).pack(side="left")
        
        # Quick target buttons
        quick_frame = ttk.Frame(target_frame)
        quick_frame.pack(fill="x", padx=5, pady=5)
        for rpm in [0, 50, 100, 150]:
            ttk.Button(quick_frame, text=f"{rpm} RPM", width=10,
                      command=lambda r=rpm: self.set_quick_target(r)).pack(side="left", padx=2)
        
        # Quick Actions
        action_frame = ttk.LabelFrame(left_panel, text="Actions")
        action_frame.pack(fill="x", pady=5)
        
        ttk.Button(action_frame, text="Stop This Motor", 
                  command=lambda: self.send_cmd("X")).pack(fill="x", padx=5, pady=2)
        ttk.Button(action_frame, text="Stop All Motors", 
                  command=lambda: self.send_cmd("A")).pack(fill="x", padx=5, pady=2)
        ttk.Button(action_frame, text="Clear Plot", 
                  command=self.clear_plot).pack(fill="x", padx=5, pady=2)
        
        # Statistics
        stats_frame = ttk.LabelFrame(left_panel, text="Performance Metrics")
        stats_frame.pack(fill="both", expand=True, pady=5)
        
        self.stats_text = tk.Text(stats_frame, height=10, width=40, font=("Courier", 9))
        self.stats_text.pack(fill="both", expand=True, padx=5, pady=5)
        
        # File Operations
        file_frame = ttk.LabelFrame(left_panel, text="Save/Load")
        file_frame.pack(fill="x", pady=5)
        
        ttk.Button(file_frame, text="Save Motor PID", command=self.save_motor_pid).pack(fill="x", padx=5, pady=2)
        ttk.Button(file_frame, text="Load Motor PID", command=self.load_motor_pid).pack(fill="x", padx=5, pady=2)
        
        # ===== RIGHT PANEL: Plot =====
        right_panel = ttk.Frame(main_frame)
        right_panel.pack(side="right", fill="both", expand=True)
        
        plot_frame = ttk.LabelFrame(right_panel, text="Real-time Response (Selected Motor)")
        plot_frame.pack(fill="both", expand=True)
        
        # Create matplotlib figure
        self.fig = Figure(figsize=(8, 6))
        
        # RPM plot
        self.ax1 = self.fig.add_subplot(211)
        self.ax1.set_ylabel('RPM', fontsize=10)
        self.ax1.set_ylim(0, 200)
        self.ax1.grid(True, alpha=0.3)
        self.line_target, = self.ax1.plot([], [], 'b--', label='Target', linewidth=2)
        self.line_actual, = self.ax1.plot([], [], 'r-', label='Actual', linewidth=2)
        self.ax1.legend(loc='upper right')
        
        # PWM plot
        self.ax2 = self.fig.add_subplot(212)
        self.ax2.set_xlabel('Time (samples)', fontsize=10)
        self.ax2.set_ylabel('PWM', fontsize=10)
        self.ax2.set_ylim(0, 260)
        self.ax2.grid(True, alpha=0.3)
        self.line_pwm, = self.ax2.plot([], [], 'g-', label='PWM Output', linewidth=1.5)
        self.ax2.legend(loc='upper right')
        
        self.fig.tight_layout()
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        
        # ===== Status Bar =====
        status_frame = ttk.Frame(self.root)
        status_frame.pack(fill="x", padx=10, pady=5)
        
        self.live_status = ttk.Label(status_frame, text="Waiting for connection...", 
                                    font=("Arial", 10))
        self.live_status.pack(side="left")
        
        self.data_rate = ttk.Label(status_frame, text="", font=("Arial", 9))
        self.data_rate.pack(side="right")
    
    def create_param_control(self, parent, label, cmd_char, min_v, max_v, default):
        frame = ttk.Frame(parent)
        frame.pack(fill="x", pady=3, padx=5)
        
        ttk.Label(frame, text=label, width=18).pack(side="left")
        
        var = tk.DoubleVar(value=default)
        
        scale = ttk.Scale(frame, from_=min_v, to=max_v, variable=var, 
                         orient="horizontal", length=150)
        scale.pack(side="left", padx=5)
        
        entry = ttk.Entry(frame, textvariable=var, width=8)
        entry.pack(side="left", padx=2)
        
        def send():
            self.send_pid_command(cmd_char, var.get())
        
        btn = ttk.Button(frame, text="Set", command=send, width=5)
        btn.pack(side="left", padx=2)
        
        return var
    
    def get_ports(self):
        ports = serial.tools.list_ports.comports()
        return [p.device for p in ports]
    
    def refresh_ports(self):
        self.port_combo['values'] = self.get_ports()
        if self.port_combo['values']:
            self.port_combo.current(0)
    
    def on_motor_select(self):
        """Called when motor selection changes"""
        self.selected_motor = self.motor_var.get()
        self.update_motor_display()
        self.clear_plot()
        print(f"Selected: {self.motor_names[self.selected_motor]}")
    
    def update_motor_display(self):
        """Update UI to show selected motor info"""
        motor_idx = self.selected_motor
        data = self.motor_data[motor_idx]
        
        self.motor_status_frame.config(text=f"Selected Motor: {self.motor_names[motor_idx]}")
        
        # Update PID sliders to show current motor's values
        self.kp_var.set(data['kp'])
        self.ki_var.set(data['ki'])
        self.kd_var.set(data['kd'])
        self.ff_var.set(data['ff'])
        
        self.update_stats_display()
    
    def toggle_connection(self):
        if not self.is_connected:
            try:
                port = self.port_combo.get()
                if not port:
                    messagebox.showerror("Error", "Please select a port")
                    return
                
                self.ser = serial.Serial(port, 115200, timeout=0.1)
                time.sleep(2)  # Wait for Arduino reset
                
                # Enable CSV mode
                self.ser.write(b'G\n')
                time.sleep(0.1)
                
                self.is_connected = True
                self.running = True
                self.btn_connect.config(text="Disconnect")
                self.status_label.config(text="● Connected", foreground="green")
                
                # Start reading thread
                self.read_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
                self.read_thread.start()
                
                # Start plot update
                self.update_plot()
                
                messagebox.showinfo("Connected", f"Connected to {port}")
                
            except Exception as e:
                messagebox.showerror("Connection Error", str(e))
        else:
            self.running = False
            self.is_connected = False
            if self.ser:
                self.ser.close()
            self.btn_connect.config(text="Connect")
            self.status_label.config(text="● Disconnected", foreground="red")
    
    def send_pid_command(self, prefix, value):
        """Send PID command for selected motor"""
        if self.is_connected and self.ser:
            # Format: P<value> (sends to all motors - we'll isolate by testing one at a time)
            cmd = f"{prefix}{value}\n"
            self.ser.write(cmd.encode())
            
            # Store locally
            motor_idx = self.selected_motor
            if prefix == 'P': 
                self.motor_data[motor_idx]['kp'] = value
            elif prefix == 'I': 
                self.motor_data[motor_idx]['ki'] = value
            elif prefix == 'D': 
                self.motor_data[motor_idx]['kd'] = value
            elif prefix == 'F': 
                self.motor_data[motor_idx]['ff'] = value
            
            print(f"Sent: {cmd.strip()}")
    
    def send_cmd(self, cmd):
        if self.is_connected and self.ser:
            self.ser.write(f"{cmd}\n".encode())
    
    def send_target(self):
        target = self.target_var.get()
        self.send_pid_command('T', target)
    
    def set_quick_target(self, rpm):
        self.target_var.set(rpm)
        self.send_target()
    
    def on_target_change(self, value):
        pass
    
    def read_serial_loop(self):
        """Read CSV data from Arduino"""
        last_data_time = time.time()
        data_count = 0
        
        while self.running and self.is_connected:
            try:
                if self.ser and self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    
                    if line and not line.startswith('#'):
                        # Parse CSV: Target,Actual,PWM,Kp,Ki,Kd
                        parts = line.split(',')
                        if len(parts) >= 6:
                            try:
                                target = float(parts[0])
                                actual = float(parts[1])
                                pwm = float(parts[2])
                                kp = float(parts[3])
                                ki = float(parts[4])
                                kd = float(parts[5])
                                
                                # Store in selected motor's data
                                motor_idx = self.selected_motor
                                self.motor_data[motor_idx]['targets'].append(target)
                                self.motor_data[motor_idx]['actuals'].append(actual)
                                self.motor_data[motor_idx]['pwms'].append(pwm)
                                error = target - actual
                                self.motor_data[motor_idx]['errors'].append(error)
                                
                                # Update PID values in storage
                                self.motor_data[motor_idx]['kp'] = kp
                                self.motor_data[motor_idx]['ki'] = ki
                                self.motor_data[motor_idx]['kd'] = kd
                                
                                # Update status display
                                status = f"Target: {target:.1f} | Actual: {actual:.1f} | Error: {error:.1f} | PWM: {pwm:.0f}"
                                self.root.after(0, lambda s=status: self.live_status.config(text=s))
                                
                                # Calculate statistics
                                self.calculate_statistics(motor_idx)
                                
                                data_count += 1
                                
                            except ValueError:
                                pass
                
                # Update data rate every second
                current_time = time.time()
                if current_time - last_data_time >= 1.0:
                    rate = f"Data rate: {data_count} Hz"
                    self.root.after(0, lambda r=rate: self.data_rate.config(text=r))
                    data_count = 0
                    last_data_time = current_time
                
                time.sleep(0.01)
                
            except Exception as e:
                print(f"Read error: {e}")
                time.sleep(0.1)
    
    def calculate_statistics(self, motor_idx):
        """Calculate performance metrics for motor"""
        data = self.motor_data[motor_idx]
        
        if len(data['actuals']) < 10:
            return
        
        recent_targets = list(data['targets'])[-100:]
        recent_actuals = list(data['actuals'])[-100:]
        
        if not recent_targets or not recent_actuals:
            return
        
        target = recent_targets[-1]
        
        if target > 10:
            # Steady state error
            steady_actuals = recent_actuals[-20:]
            ss_error = abs(target - sum(steady_actuals)/len(steady_actuals))
            
            # Overshoot
            peak = max(recent_actuals)
            overshoot = max(0, (peak - target) / target * 100) if target > 0 else 0
            
            data['stats']['ss_error'] = ss_error
            data['stats']['overshoot'] = overshoot
            data['stats']['peak'] = peak
            
            self.root.after(0, self.update_stats_display)
    
    def update_stats_display(self):
        """Update statistics panel"""
        motor_idx = self.selected_motor
        data = self.motor_data[motor_idx]
        
        # Update motor info
        if data['actuals']:
            info_text = f"{self.motor_names[motor_idx]}\nTarget: {list(data['targets'])[-1]:.1f} RPM | Actual: {list(data['actuals'])[-1]:.1f} RPM"
            self.motor_info_label.config(text=info_text)
        
        stats_text = f"""
Current PID Values:
  Kp: {data['kp']:.3f}
  Ki: {data['ki']:.3f}
  Kd: {data['kd']:.4f}
  FF: {data['ff']:.3f}

Performance Metrics:
  SS Error: {data['stats']['ss_error']:.2f} RPM
  Overshoot: {data['stats']['overshoot']:.1f} %
  Peak RPM: {data['stats']['peak']:.1f}
  
Data Points: {len(data['actuals'])}
Motor: {self.motor_names[motor_idx]}
        """
        self.stats_text.delete(1.0, tk.END)
        self.stats_text.insert(1.0, stats_text.strip())
    
    def update_plot(self):
        """Update plot for selected motor"""
        if not self.is_connected:
            return
        
        try:
            motor_idx = self.selected_motor
            data = self.motor_data[motor_idx]
            
            # Update RPM plot
            x_data = list(range(len(data['targets'])))
            self.line_target.set_data(x_data, list(data['targets']))
            self.line_actual.set_data(x_data, list(data['actuals']))
            
            # Update PWM plot
            self.line_pwm.set_data(x_data, list(data['pwms']))
            
            # Adjust axes
            if x_data:
                self.ax1.set_xlim(0, max(100, len(x_data)))
                self.ax2.set_xlim(0, max(100, len(x_data)))
            
            if data['actuals']:
                max_rpm = max(max(data['targets']), max(data['actuals']))
                self.ax1.set_ylim(0, max(200, max_rpm * 1.1))
            
            self.canvas.draw()
            
        except Exception as e:
            print(f"Plot error: {e}")
        
        if self.is_connected:
            self.root.after(100, self.update_plot)
    
    def clear_plot(self):
        """Clear data for current motor"""
        motor_idx = self.selected_motor
        data = self.motor_data[motor_idx]
        
        data['targets'].clear()
        data['actuals'].clear()
        data['pwms'].clear()
        data['errors'].clear()
        
        for _ in range(300):
            data['targets'].append(0)
            data['actuals'].append(0)
            data['pwms'].append(0)
            data['errors'].append(0)
    
    def copy_to_all_motors(self):
        """Copy selected motor's PID to all motors"""
        motor_idx = self.selected_motor
        source_data = self.motor_data[motor_idx]
        
        for i in range(4):
            if i != motor_idx:
                self.motor_data[i]['kp'] = source_data['kp']
                self.motor_data[i]['ki'] = source_data['ki']
                self.motor_data[i]['kd'] = source_data['kd']
                self.motor_data[i]['ff'] = source_data['ff']
        
        # Send same values to Arduino (it will use them for all)
        self.send_pid_command('P', source_data['kp'])
        self.send_pid_command('I', source_data['ki'])
        self.send_pid_command('D', source_data['kd'])
        self.send_pid_command('F', source_data['ff'])
        
        messagebox.showinfo("Copied", f"Copied {self.motor_names[motor_idx]} PID to all motors!")
    
    def load_conservative(self):
        self.kp_var.set(0.8)
        self.ki_var.set(0.5)
        self.kd_var.set(0.08)
        self.send_pid_command('P', 0.8)
        self.send_pid_command('I', 0.5)
        self.send_pid_command('D', 0.08)
    
    def load_balanced(self):
        self.kp_var.set(1.0)
        self.ki_var.set(0.8)
        self.kd_var.set(0.08)
        self.send_pid_command('P', 1.0)
        self.send_pid_command('I', 0.8)
        self.send_pid_command('D', 0.08)
    
    def load_aggressive(self):
        self.kp_var.set(1.5)
        self.ki_var.set(0.8)
        self.kd_var.set(0.1)
        self.send_pid_command('P', 1.5)
        self.send_pid_command('I', 0.8)
        self.send_pid_command('D', 0.1)
    
    def save_motor_pid(self):
        """Save selected motor's PID to JSON"""
        motor_idx = self.selected_motor
        data = self.motor_data[motor_idx]
        
        filename = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
            initialfile=f"pid_{self.motor_names[motor_idx]}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        )
        
        if filename:
            config = {
                'motor': self.motor_names[motor_idx],
                'motor_index': motor_idx,
                'kp': data['kp'],
                'ki': data['ki'],
                'kd': data['kd'],
                'ff': data['ff'],
                'timestamp': datetime.now().isoformat(),
                'stats': data['stats']
            }
            
            with open(filename, 'w') as f:
                json.dump(config, f, indent=4)
            
            messagebox.showinfo("Saved", f"PID for {self.motor_names[motor_idx]} saved!")
    
    def load_motor_pid(self):
        """Load motor PID from JSON"""
        filename = filedialog.askopenfilename(
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        
        if filename:
            try:
                with open(filename, 'r') as f:
                    config = json.load(f)
                
                motor_idx = self.selected_motor
                self.motor_data[motor_idx]['kp'] = config['kp']
                self.motor_data[motor_idx]['ki'] = config['ki']
                self.motor_data[motor_idx]['kd'] = config['kd']
                self.motor_data[motor_idx]['ff'] = config['ff']
                
                # Update UI
                self.kp_var.set(config['kp'])
                self.ki_var.set(config['ki'])
                self.kd_var.set(config['kd'])
                self.ff_var.set(config['ff'])
                
                # Send to Arduino
                self.send_pid_command('P', config['kp'])
                self.send_pid_command('I', config['ki'])
                self.send_pid_command('D', config['kd'])
                self.send_pid_command('F', config['ff'])
                
                messagebox.showinfo("Loaded", f"PID loaded for {self.motor_names[motor_idx]}")
                
            except Exception as e:
                messagebox.showerror("Error", f"Failed to load:\n{e}")

def main():
    root = tk.Tk()
    app = SingleMotorPIDTuner(root)
    root.mainloop()

if __name__ == "__main__":
    main()
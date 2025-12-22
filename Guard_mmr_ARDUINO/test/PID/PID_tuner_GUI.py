"""
Arduino Uno PID Tuner - Real-time Visualization and Control
Compatible with single motor and 4-motor PID controllers
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

class PIDTunerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Arduino Uno PID Tuner")
        self.root.geometry("1200x800")
        
        # Serial connection
        self.ser = None
        self.is_connected = False
        self.read_thread = None
        self.running = False
        
        # Data storage (keep last 300 points = 30 seconds at 100ms)
        self.history_len = 300
        self.targets = collections.deque([0]*self.history_len, maxlen=self.history_len)
        self.actuals = collections.deque([0]*self.history_len, maxlen=self.history_len)
        self.errors = collections.deque([0]*self.history_len, maxlen=self.history_len)
        self.pwms = collections.deque([0]*self.history_len, maxlen=self.history_len)
        
        # Statistics
        self.stats = {
            'settling_time': 0,
            'overshoot': 0,
            'steady_state_error': 0,
            'peak_rpm': 0
        }
        
        # PID values
        self.current_kp = 0.0
        self.current_ki = 0.0
        self.current_kd = 0.0
        self.current_ff = 0.0
        
        self.setup_ui()
        
    def setup_ui(self):
        # ===== Top Frame: Connection =====
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
        
        self.status_label = ttk.Label(conn_frame, text="● Disconnected", foreground="red")
        self.status_label.pack(side="left", padx=10)
        
        # Quick Actions
        actions_frame = ttk.LabelFrame(top_frame, text="Quick Actions")
        actions_frame.pack(side="right", padx=5)
        
        ttk.Button(actions_frame, text="Stop Motor", command=lambda: self.send_cmd("X")).pack(side="left", padx=2)
        ttk.Button(actions_frame, text="Reset", command=lambda: self.send_cmd("H")).pack(side="left", padx=2)
        ttk.Button(actions_frame, text="Clear Plot", command=self.clear_plot).pack(side="left", padx=2)
        
        # ===== Main Content =====
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill="both", expand=True, padx=10, pady=5)
        
        # Left panel: Controls
        left_panel = ttk.Frame(main_frame, width=350)
        left_panel.pack(side="left", fill="y", padx=5)
        left_panel.pack_propagate(False)
        
        # PID Controls
        pid_frame = ttk.LabelFrame(left_panel, text="PID Tuning")
        pid_frame.pack(fill="x", pady=5)
        
        self.kp_var = self.create_param_control(pid_frame, "Kp (Proportional)", 'P', 0, 5.0, 0.8)
        self.ki_var = self.create_param_control(pid_frame, "Ki (Integral)", 'I', 0, 3.0, 0.5)
        self.kd_var = self.create_param_control(pid_frame, "Kd (Derivative)", 'D', 0, 1.0, 0.05)
        self.ff_var = self.create_param_control(pid_frame, "Feed-Forward", 'F', 0, 2.0, 0.85)
        
        # Preset buttons
        preset_frame = ttk.Frame(pid_frame)
        preset_frame.pack(fill="x", pady=5)
        ttk.Button(preset_frame, text="Conservative", command=self.load_conservative).pack(side="left", padx=2)
        ttk.Button(preset_frame, text="Balanced", command=self.load_balanced).pack(side="left", padx=2)
        ttk.Button(preset_frame, text="Aggressive", command=self.load_aggressive).pack(side="left", padx=2)
        
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
            ttk.Button(quick_frame, text=f"{rpm}", width=5,
                      command=lambda r=rpm: self.set_quick_target(r)).pack(side="left", padx=2)
        
        # Test Sequence
        test_frame = ttk.LabelFrame(left_panel, text="Test Sequence")
        test_frame.pack(fill="x", pady=5)
        
        ttk.Button(test_frame, text="Run Test Sequence (R)", 
                  command=lambda: self.send_cmd("R")).pack(fill="x", padx=5, pady=5)
        ttk.Label(test_frame, text="Runs: 50→100→150→100→50→0", 
                 font=("Arial", 8)).pack()
        
        # Statistics
        stats_frame = ttk.LabelFrame(left_panel, text="Performance Metrics")
        stats_frame.pack(fill="both", expand=True, pady=5)
        
        self.stats_text = tk.Text(stats_frame, height=10, width=40, font=("Courier", 9))
        self.stats_text.pack(fill="both", expand=True, padx=5, pady=5)
        
        # File Operations
        file_frame = ttk.LabelFrame(left_panel, text="Save/Load")
        file_frame.pack(fill="x", pady=5)
        
        ttk.Button(file_frame, text="Save PID Values", command=self.save_pid).pack(fill="x", padx=5, pady=2)
        ttk.Button(file_frame, text="Load PID Values", command=self.load_pid).pack(fill="x", padx=5, pady=2)
        
        # Right panel: Plot
        right_panel = ttk.Frame(main_frame)
        right_panel.pack(side="right", fill="both", expand=True)
        
        plot_frame = ttk.LabelFrame(right_panel, text="Real-time Response")
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
        
        # Bottom status bar
        status_frame = ttk.Frame(self.root)
        status_frame.pack(fill="x", padx=10, pady=5)
        
        self.live_status = ttk.Label(status_frame, text="Waiting for data...", 
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
            self.send_command(cmd_char, var.get())
        
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
                
            except Exception as e:
                messagebox.showerror("Connection Error", str(e))
        else:
            self.running = False
            self.is_connected = False
            if self.ser:
                self.ser.close()
            self.btn_connect.config(text="Connect")
            self.status_label.config(text="● Disconnected", foreground="red")
    
    def send_command(self, prefix, value):
        if self.is_connected and self.ser:
            cmd = f"{prefix}{value}\n"
            self.ser.write(cmd.encode())
            
            # Update local tracking
            if prefix == 'P': self.current_kp = value
            elif prefix == 'I': self.current_ki = value
            elif prefix == 'D': self.current_kd = value
            elif prefix == 'F': self.current_ff = value
            
            print(f"Sent: {cmd.strip()}")
    
    def send_cmd(self, cmd):
        if self.is_connected and self.ser:
            self.ser.write(f"{cmd}\n".encode())
    
    def send_target(self):
        target = self.target_var.get()
        self.send_command('T', target)
    
    def set_quick_target(self, rpm):
        self.target_var.set(rpm)
        self.send_target()
    
    def on_target_change(self, value):
        # Update entry as slider moves
        pass
    
    def read_serial_loop(self):
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
                                
                                # Update data
                                self.targets.append(target)
                                self.actuals.append(actual)
                                self.pwms.append(pwm)
                                error = target - actual
                                self.errors.append(error)
                                
                                # Update PID values
                                self.current_kp = kp
                                self.current_ki = ki
                                self.current_kd = kd
                                
                                # Update status
                                status = f"Target: {target:.1f} | Actual: {actual:.1f} | Error: {error:.1f} | PWM: {pwm:.0f}"
                                self.root.after(0, lambda s=status: self.live_status.config(text=s))
                                
                                # Calculate statistics
                                self.calculate_statistics()
                                
                                data_count += 1
                                
                            except ValueError:
                                pass
                
                # Update data rate
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
    
    def calculate_statistics(self):
        if len(self.actuals) < 10:
            return
        
        # Get recent data
        recent_targets = list(self.targets)[-100:]
        recent_actuals = list(self.actuals)[-100:]
        
        if not recent_targets or not recent_actuals:
            return
        
        # Current target
        target = recent_targets[-1]
        
        if target > 10:  # Only calculate when target is significant
            # Steady state error (last 20 samples)
            steady_actuals = recent_actuals[-20:]
            steady_error = abs(target - sum(steady_actuals)/len(steady_actuals))
            
            # Peak/overshoot
            peak = max(recent_actuals)
            overshoot = max(0, (peak - target) / target * 100) if target > 0 else 0
            
            # Update stats
            self.stats['steady_state_error'] = steady_error
            self.stats['overshoot'] = overshoot
            self.stats['peak_rpm'] = peak
            
            # Update display
            self.root.after(0, self.update_stats_display)
    
    def update_stats_display(self):
        stats_text = f"""
Current PID Values:
  Kp: {self.current_kp:.3f}
  Ki: {self.current_ki:.3f}
  Kd: {self.current_kd:.4f}
  FF: {self.current_ff:.3f}

Performance:
  SS Error: {self.stats['steady_state_error']:.2f} RPM
  Overshoot: {self.stats['overshoot']:.1f} %
  Peak RPM: {self.stats['peak_rpm']:.1f}
  
Data Points: {len(self.actuals)}
        """
        self.stats_text.delete(1.0, tk.END)
        self.stats_text.insert(1.0, stats_text.strip())
    
    def update_plot(self):
        if not self.is_connected:
            return
        
        try:
            # Update RPM plot
            x_data = list(range(len(self.targets)))
            self.line_target.set_data(x_data, list(self.targets))
            self.line_actual.set_data(x_data, list(self.actuals))
            
            # Update PWM plot
            self.line_pwm.set_data(x_data, list(self.pwms))
            
            # Adjust x-axis
            if x_data:
                self.ax1.set_xlim(0, max(100, len(x_data)))
                self.ax2.set_xlim(0, max(100, len(x_data)))
            
            # Auto-scale y-axis for RPM
            if self.actuals:
                max_rpm = max(max(self.targets), max(self.actuals))
                self.ax1.set_ylim(0, max(200, max_rpm * 1.1))
            
            self.canvas.draw()
            
        except Exception as e:
            print(f"Plot error: {e}")
        
        # Schedule next update
        if self.is_connected:
            self.root.after(100, self.update_plot)
    
    def clear_plot(self):
        self.targets.clear()
        self.actuals.clear()
        self.pwms.clear()
        self.errors.clear()
        for _ in range(self.history_len):
            self.targets.append(0)
            self.actuals.append(0)
            self.pwms.append(0)
            self.errors.append(0)
    
    def load_conservative(self):
        self.kp_var.set(0.8)
        self.ki_var.set(0.5)
        self.kd_var.set(0.08)
        self.send_command('P', 0.8)
        self.send_command('I', 0.5)
        self.send_command('D', 0.08)
        messagebox.showinfo("Preset", "Conservative preset loaded")
    
    def load_balanced(self):
        self.kp_var.set(1.0)
        self.ki_var.set(0.8)
        self.kd_var.set(0.08)
        self.send_command('P', 1.0)
        self.send_command('I', 0.8)
        self.send_command('D', 0.08)
        messagebox.showinfo("Preset", "Balanced preset loaded")
    
    def load_aggressive(self):
        self.kp_var.set(1.5)
        self.ki_var.set(0.8)
        self.kd_var.set(0.1)
        self.send_command('P', 1.5)
        self.send_command('I', 0.8)
        self.send_command('D', 0.1)
        messagebox.showinfo("Preset", "Aggressive preset loaded")
    
    def save_pid(self):
        filename = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
            initialfile=f"pid_config_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        )
        
        if filename:
            config = {
                'kp': self.kp_var.get(),
                'ki': self.ki_var.get(),
                'kd': self.kd_var.get(),
                'ff': self.ff_var.get(),
                'timestamp': datetime.now().isoformat(),
                'notes': 'Tuned with Arduino Uno PID Tuner'
            }
            
            with open(filename, 'w') as f:
                json.dump(config, f, indent=4)
            
            messagebox.showinfo("Saved", f"PID values saved to:\n{filename}")
    
    def load_pid(self):
        filename = filedialog.askopenfilename(
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        
        if filename:
            try:
                with open(filename, 'r') as f:
                    config = json.load(f)
                
                self.kp_var.set(config['kp'])
                self.ki_var.set(config['ki'])
                self.kd_var.set(config['kd'])
                if 'ff' in config:
                    self.ff_var.set(config['ff'])
                
                # Send to Arduino
                self.send_command('P', config['kp'])
                self.send_command('I', config['ki'])
                self.send_command('D', config['kd'])
                if 'ff' in config:
                    self.send_command('F', config['ff'])
                
                messagebox.showinfo("Loaded", "PID values loaded and sent to Arduino")
                
            except Exception as e:
                messagebox.showerror("Error", f"Failed to load file:\n{e}")

def main():
    root = tk.Tk()
    app = PIDTunerApp(root)
    root.mainloop()

if __name__ == "__main__":
    main()
"""
RTH Analysis Tool - Analyze when DJI triggers Return-To-Home based on battery levels.

This script logs and plots battery-related telemetry to help deduce when
the flight mode switches to RTH due to low battery.

Authors: Edouard G.A. Rolland
"""

import time
import sys
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
from datetime import datetime
from djiInterface import DJIInterface

# Configuration
MAX_POINTS = 600  # 10 minutes at 1Hz
UPDATE_INTERVAL = 1000  # milliseconds

class RTHAnalyzer:
    def __init__(self, dji_interface):
        self.dji = dji_interface
        
        # Data storage (time series)
        self.timestamps = deque(maxlen=MAX_POINTS)
        self.battery_level = deque(maxlen=MAX_POINTS)
        self.battery_needed_rth = deque(maxlen=MAX_POINTS)
        self.battery_needed_land = deque(maxlen=MAX_POINTS)
        self.remaining_flight_time = deque(maxlen=MAX_POINTS)
        self.time_to_rth = deque(maxlen=MAX_POINTS)
        self.time_to_land = deque(maxlen=MAX_POINTS)
        self.total_time = deque(maxlen=MAX_POINTS)
        self.distance_to_home = deque(maxlen=MAX_POINTS)
        self.max_radius = deque(maxlen=MAX_POINTS)
        self.altitude = deque(maxlen=MAX_POINTS)
        
        # Flight mode tracking
        self.flight_modes = deque(maxlen=MAX_POINTS)
        self.rth_status = deque(maxlen=MAX_POINTS)
        
        # RTH trigger events
        self.rth_events = []  # List of (timestamp, battery_level, reason)
        self.last_flight_mode = None
        
        self.start_time = time.time()
        
    def update_data(self):
        """Fetch new telemetry data and store it."""
        telemetry = self.dji.getTelemetry()
        if not telemetry:
            return False
        
        elapsed = time.time() - self.start_time
        self.timestamps.append(elapsed)
        
        # Battery data
        self.battery_level.append(self.dji.getBatteryLevel())
        self.battery_needed_rth.append(self.dji.getBatteryNeededToGoHome())
        self.battery_needed_land.append(self.dji.getBatteryNeededToLand())
        
        # Time data
        self.remaining_flight_time.append(self.dji.getRemainingFlightTime())
        self.time_to_rth.append(self.dji.getTimeNeededToGoHome())
        self.time_to_land.append(self.dji.getTimeNeededToLand())
        self.total_time.append(self.dji.getTotalTime())
        
        # Position data
        self.distance_to_home.append(self.dji.getDistanceToHome())
        self.max_radius.append(self.dji.getMaxRadiusCanFlyAndGoHome())
        location = self.dji.getLocation()
        self.altitude.append(location.get('altitude', 0) if location else 0)
        
        # Flight mode and RTH status
        flight_mode = self.dji.getFlightMode()
        rth_status = "N/A"  # Not available in current telemetry
        self.flight_modes.append(flight_mode)
        self.rth_status.append(str(rth_status))
        
        # Detect RTH trigger
        if self.last_flight_mode is not None and self.last_flight_mode != flight_mode:
            if 'GO_HOME' in str(flight_mode).upper() or 'RTH' in str(flight_mode).upper():
                event = {
                    'time': elapsed,
                    'battery': self.dji.getBatteryLevel(),
                    'battery_needed_rth': self.dji.getBatteryNeededToGoHome(),
                    'battery_needed_land': self.dji.getBatteryNeededToLand(),
                    'distance': self.dji.getDistanceToHome(),
                    'altitude': location.get('altitude', 0) if location else 0,
                    'rth_status': rth_status,
                    'from_mode': self.last_flight_mode,
                    'to_mode': flight_mode
                }
                self.rth_events.append(event)
                print(f"\n{'='*60}")
                print(f"RTH TRIGGERED at {elapsed:.1f}s!")
                print(f"  Battery: {event['battery']}%")
                print(f"  Battery needed for RTH: {event['battery_needed_rth']}%")
                print(f"  Battery needed to land: {event['battery_needed_land']}%")
                print(f"  Distance to home: {event['distance']:.1f}m")
                print(f"  Altitude: {event['altitude']:.1f}m")
                print(f"  RTH Status: {event['rth_status']}")
                print(f"  Mode change: {event['from_mode']} -> {event['to_mode']}")
                print(f"{'='*60}\n")
        
        self.last_flight_mode = flight_mode
        return True

    def create_plot(self):
        """Create the real-time plot."""
        self.fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
        self.fig.suptitle('DJI RTH Analysis - Battery & Flight Mode Monitoring', fontsize=14)
        
        # Plot 1: Battery levels
        self.ax1 = axes[0]
        self.ax1.set_ylabel('Battery %')
        self.ax1.set_ylim(0, 100)
        self.ax1.grid(True, alpha=0.3)
        
        # Plot 2: Time metrics
        self.ax2 = axes[1]
        self.ax2.set_ylabel('Time (seconds)')
        self.ax2.grid(True, alpha=0.3)
        
        # Plot 3: Distance and altitude
        self.ax3 = axes[2]
        self.ax3.set_ylabel('Distance/Altitude (m)')
        self.ax3.grid(True, alpha=0.3)
        
        # Plot 4: Max radius
        self.ax4 = axes[3]
        self.ax4.set_ylabel('Max Radius (m)')
        self.ax4.set_xlabel('Time (seconds)')
        self.ax4.grid(True, alpha=0.3)
        
        # Initialize empty lines
        self.lines = {}
        
        # Battery plot lines
        self.lines['battery'], = self.ax1.plot([], [], 'b-', linewidth=2, label='Battery Level')
        self.lines['batt_rth'], = self.ax1.plot([], [], 'r--', linewidth=1.5, label='Battery Needed RTH')
        self.lines['batt_land'], = self.ax1.plot([], [], 'orange', linestyle='--', linewidth=1.5, label='Battery Needed Land')
        self.ax1.legend(loc='upper right')
        
        # Time plot lines
        self.lines['remaining'], = self.ax2.plot([], [], 'g-', linewidth=2, label='Remaining Flight Time')
        self.lines['time_rth'], = self.ax2.plot([], [], 'r-', linewidth=1.5, label='Time to RTH')
        self.lines['time_land'], = self.ax2.plot([], [], 'orange', linewidth=1.5, label='Time to Land')
        self.lines['total_time'], = self.ax2.plot([], [], 'm--', linewidth=1.5, label='Total Time (RTH+Land)')
        self.ax2.legend(loc='upper right')
        
        # Distance plot lines
        self.lines['distance'], = self.ax3.plot([], [], 'b-', linewidth=2, label='Distance to Home')
        self.lines['altitude'], = self.ax3.plot([], [], 'g-', linewidth=1.5, label='Altitude')
        self.ax3.legend(loc='upper right')
        
        # Max radius
        self.lines['max_radius'], = self.ax4.plot([], [], 'purple', linewidth=2, label='Max Radius Can Fly')
        self.ax4.legend(loc='upper right')
        
        # Flight mode text
        self.mode_text = self.ax1.text(0.02, 0.95, '', transform=self.ax1.transAxes, 
                                        fontsize=10, verticalalignment='top',
                                        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        plt.tight_layout()
        return self.fig

    def animate(self, frame):
        """Animation update function."""
        if not self.update_data():
            return list(self.lines.values())
        
        if len(self.timestamps) < 2:
            return list(self.lines.values())
        
        t = list(self.timestamps)
        
        # Update battery lines
        self.lines['battery'].set_data(t, list(self.battery_level))
        self.lines['batt_rth'].set_data(t, list(self.battery_needed_rth))
        self.lines['batt_land'].set_data(t, list(self.battery_needed_land))
        
        # Update time lines
        self.lines['remaining'].set_data(t, list(self.remaining_flight_time))
        self.lines['time_rth'].set_data(t, list(self.time_to_rth))
        self.lines['time_land'].set_data(t, list(self.time_to_land))
        self.lines['total_time'].set_data(t, list(self.total_time))
        
        # Update distance lines
        self.lines['distance'].set_data(t, list(self.distance_to_home))
        self.lines['altitude'].set_data(t, list(self.altitude))
        
        # Update max radius
        self.lines['max_radius'].set_data(t, list(self.max_radius))
        
        # Update axes limits
        for ax in [self.ax1, self.ax2, self.ax3, self.ax4]:
            ax.set_xlim(max(0, t[-1] - 300), t[-1] + 10)  # Show last 5 minutes
            ax.relim()
            ax.autoscale_view(scalex=False)
        
        # Update flight mode text
        if self.flight_modes:
            mode = self.flight_modes[-1]
            rth_stat = self.rth_status[-1] if self.rth_status else 'N/A'
            self.mode_text.set_text(f'Flight Mode: {mode}\nRTH Status: {rth_stat}')
            
            # Highlight if in RTH mode
            if 'GO_HOME' in str(mode).upper() or 'RTH' in str(mode).upper():
                self.mode_text.set_bbox(dict(boxstyle='round', facecolor='red', alpha=0.8))
            else:
                self.mode_text.set_bbox(dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        return list(self.lines.values())

    def run(self):
        """Start the real-time plotting."""
        print("Starting RTH Analysis...")
        print("This tool monitors battery levels and flight mode to detect RTH triggers.")
        print("Press Ctrl+C to stop and save results.\n")
        
        self.create_plot()
        
        ani = animation.FuncAnimation(
            self.fig, self.animate, interval=UPDATE_INTERVAL, blit=False, cache_frame_data=False
        )
        
        try:
            plt.show()
        except KeyboardInterrupt:
            pass
        finally:
            self.save_results()
    
    def save_results(self):
        """Save the collected data and analysis."""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Save RTH events
        if self.rth_events:
            print("\n" + "="*60)
            print("RTH EVENTS SUMMARY")
            print("="*60)
            for i, event in enumerate(self.rth_events):
                print(f"\nEvent {i+1}:")
                print(f"  Time: {event['time']:.1f}s")
                print(f"  Battery: {event['battery']}%")
                print(f"  Battery needed RTH: {event['battery_needed_rth']}%")
                print(f"  Battery needed Land: {event['battery_needed_land']}%")
                print(f"  Distance: {event['distance']:.1f}m")
                print(f"  Mode: {event['from_mode']} -> {event['to_mode']}")
        else:
            print("\nNo RTH events detected during this session.")
        
        # Save data to CSV - only if we have complete data
        min_len = min(
            len(self.timestamps), len(self.battery_level), len(self.battery_needed_rth),
            len(self.battery_needed_land), len(self.remaining_flight_time), len(self.time_to_rth),
            len(self.time_to_land), len(self.total_time), len(self.distance_to_home),
            len(self.altitude), len(self.max_radius),
            len(self.flight_modes), len(self.rth_status)
        )
        
        if min_len > 0:
            # Save CSV file
            csv_filename = f"rth_analysis_{timestamp}.csv"
            with open(csv_filename, 'w') as f:
                f.write("time,battery,batt_needed_rth,batt_needed_land,remaining_time,")
                f.write("time_rth,time_land,total_time,distance,altitude,max_radius,")
                f.write("flight_mode,rth_status\n")
                
                for i in range(min_len):
                    f.write(f"{self.timestamps[i]:.1f},")
                    f.write(f"{self.battery_level[i]},")
                    f.write(f"{self.battery_needed_rth[i]},")
                    f.write(f"{self.battery_needed_land[i]},")
                    f.write(f"{self.remaining_flight_time[i]},")
                    f.write(f"{self.time_to_rth[i]},")
                    f.write(f"{self.time_to_land[i]},")
                    f.write(f"{self.total_time[i]},")
                    f.write(f"{self.distance_to_home[i]:.1f},")
                    f.write(f"{self.altitude[i]:.1f},")
                    f.write(f"{self.max_radius[i]},")
                    f.write(f"{self.flight_modes[i]},")
                    f.write(f"{self.rth_status[i]}\n")
            
            print(f"\nCSV data saved to: {csv_filename}")
            
            # Save TXT summary file for easy reading
            txt_filename = f"rth_analysis_{timestamp}.txt"
            with open(txt_filename, 'w') as f:
                f.write("=" * 70 + "\n")
                f.write(f"RTH ANALYSIS REPORT - {timestamp}\n")
                f.write("=" * 70 + "\n\n")
                
                # Flight summary
                flight_duration = self.timestamps[-1] if self.timestamps else 0
                f.write("FLIGHT SUMMARY\n")
                f.write("-" * 40 + "\n")
                f.write(f"  Duration:              {flight_duration:.1f} seconds ({flight_duration/60:.1f} min)\n")
                f.write(f"  Data points collected: {min_len}\n")
                f.write(f"  Start battery:         {self.battery_level[0] if self.battery_level else 'N/A'}%\n")
                f.write(f"  End battery:           {self.battery_level[-1] if self.battery_level else 'N/A'}%\n")
                f.write(f"  Max distance from home:{max(self.distance_to_home) if self.distance_to_home else 'N/A':.1f} m\n")
                f.write(f"  Max altitude:          {max(self.altitude) if self.altitude else 'N/A':.1f} m\n")
                f.write("\n")
                
                # RTH Events
                f.write("RTH EVENTS\n")
                f.write("-" * 40 + "\n")
                if self.rth_events:
                    for i, event in enumerate(self.rth_events):
                        f.write(f"\nEvent {i+1}:\n")
                        f.write(f"  Time:                  {event['time']:.1f}s\n")
                        f.write(f"  Battery:               {event['battery']}%\n")
                        f.write(f"  Battery needed RTH:    {event['battery_needed_rth']}%\n")
                        f.write(f"  Battery needed Land:   {event['battery_needed_land']}%\n")
                        f.write(f"  Distance to home:      {event['distance']:.1f}m\n")
                        f.write(f"  Altitude:              {event['altitude']:.1f}m\n")
                        f.write(f"  Mode change:           {event['from_mode']} -> {event['to_mode']}\n")
                else:
                    f.write("  No RTH events detected during this flight.\n")
                f.write("\n")
                
                # Battery analysis at key points
                f.write("BATTERY ANALYSIS\n")
                f.write("-" * 40 + "\n")
                
                # Find when battery thresholds were crossed
                for threshold in [50, 40, 30, 25, 20, 15, 10]:
                    for i, batt in enumerate(self.battery_level):
                        if batt <= threshold:
                            f.write(f"  Battery hit {threshold}% at:\n")
                            f.write(f"    Time:                {self.timestamps[i]:.1f}s\n")
                            f.write(f"    Distance to home:    {self.distance_to_home[i]:.1f}m\n")
                            f.write(f"    Altitude:            {self.altitude[i]:.1f}m\n")
                            f.write(f"    Battery needed RTH:  {self.battery_needed_rth[i]}%\n")
                            f.write(f"    Battery needed Land: {self.battery_needed_land[i]}%\n")
                            f.write(f"    Flight mode:         {self.flight_modes[i]}\n")
                            f.write("\n")
                            break
                
                # Flight mode changes
                f.write("FLIGHT MODE CHANGES\n")
                f.write("-" * 40 + "\n")
                last_mode = None
                for i, mode in enumerate(self.flight_modes):
                    if mode != last_mode:
                        f.write(f"  {self.timestamps[i]:.1f}s: {last_mode or 'START'} -> {mode}\n")
                        f.write(f"    Battery: {self.battery_level[i]}%, Distance: {self.distance_to_home[i]:.1f}m\n")
                        last_mode = mode
                f.write("\n")
                
                f.write("=" * 70 + "\n")
                f.write("END OF REPORT\n")
                f.write("=" * 70 + "\n")
            
            print(f"TXT report saved to: {txt_filename}")
        else:
            print("\nNo data collected to save.")


def main():
    IP_RC = "10.184.11.117"  # Default IP
    
    if len(sys.argv) > 1:
        IP_RC = sys.argv[1]
    
    print(f"Connecting to drone at {IP_RC}...")
    dji = DJIInterface(IP_RC)
    
    print("Starting telemetry stream...")
    dji.startTelemetryStream()
    
    # Wait for initial connection
    time.sleep(2)
    
    if not dji.getTelemetry():
        print("Warning: No telemetry data received. Make sure the drone is connected.")
    
    analyzer = RTHAnalyzer(dji)
    
    try:
        analyzer.run()
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        dji.stopTelemetryStream()
        print("Done.")


if __name__ == '__main__':
    main()

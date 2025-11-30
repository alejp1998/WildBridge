#!/usr/bin/env python3
"""
RTH Prediction Animation
Visualizes the battery level vs RTH threshold with live regression
to show how the RTH time prediction evolves during flight.
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import glob
import argparse
import os

# RTH trigger margin (battery must be >= batt_needed_rth + margin)
RTH_TRIGGER_MARGIN = 2


def load_flight_data(csv_file):
    """Load and prepare flight data from CSV."""
    df = pd.read_csv(csv_file)
    
    # Detect RTH event
    rth_idx = None
    for i in range(1, len(df)):
        mode = df['flight_mode'].iloc[i] if 'flight_mode' in df.columns else ''
        prev_mode = df['flight_mode'].iloc[i-1] if 'flight_mode' in df.columns else ''
        if 'GO_HOME' in str(mode) and 'GO_HOME' not in str(prev_mode):
            rth_idx = i
            break
    
    rth_time = df['time'].iloc[rth_idx] if rth_idx else None
    
    return df, rth_time, rth_idx


def create_rth_animation(csv_file, output_file=None, speed=1.0, show=True):
    """
    Create an animation showing battery vs RTH threshold with live regression.
    
    Args:
        csv_file: Path to flight data CSV
        output_file: If provided, save animation as MP4/GIF
        speed: Animation speed multiplier (higher = faster)
        show: Whether to display the animation
    """
    print(f"Loading flight data from: {csv_file}")
    df, rth_time, rth_idx = load_flight_data(csv_file)
    
    if rth_time is None:
        print("No RTH event found in this flight!")
        return
    
    print(f"RTH detected at {rth_time:.1f}s")
    
    # Prepare data
    times = df['time'].values
    battery = df['battery'].values
    batt_needed = df['batt_needed_rth'].values
    rth_threshold = batt_needed + RTH_TRIGGER_MARGIN
    
    # Calculate actual time to RTH for ground truth
    actual_time_to_rth = rth_time - times
    actual_time_to_rth[times > rth_time] = 0
    
    # Set up the figure - single plot with white background
    fig, ax_main = plt.subplots(figsize=(12, 7))
    
    # White background styling
    fig.patch.set_facecolor('white')
    ax_main.set_facecolor('white')
    ax_main.tick_params(colors='black')
    ax_main.xaxis.label.set_color('black')
    ax_main.yaxis.label.set_color('black')
    ax_main.title.set_color('black')
    for spine in ax_main.spines.values():
        spine.set_color('black')
    
    # Main plot setup
    ax_main.set_xlim(0, max(times) * 1.05)
    ax_main.set_ylim(0, 105)
    ax_main.set_xlabel('Time (seconds)', fontsize=12)
    ax_main.set_ylabel('Battery %', fontsize=12)
    ax_main.set_title('Battery Level vs RTH Threshold - Live Prediction', fontsize=14, fontweight='bold')
    ax_main.grid(True, alpha=0.3, color='gray')
    
    # Ground truth lines (full data, semi-transparent)
    ax_main.plot(times, battery, 'blue', alpha=0.15, linewidth=1, label='_nolegend_')
    ax_main.plot(times, rth_threshold, 'red', alpha=0.15, linewidth=1, linestyle='--', label='_nolegend_')
    ax_main.axvline(x=rth_time, color='green', alpha=0.2, linewidth=2, linestyle=':')
    
    # Animated lines
    line_battery, = ax_main.plot([], [], 'blue', linewidth=2.5, label='Battery Level')
    line_threshold, = ax_main.plot([], [], 'red', linewidth=2, linestyle='--', label='RTH Threshold (batt_needed + 2%)')
    line_regression, = ax_main.plot([], [], 'orange', linewidth=2, linestyle='-', label='Battery Regression')
    hline_current_threshold = ax_main.axhline(y=0, color='purple', alpha=0, linewidth=2, linestyle='-', label='Current Threshold for Intersection')
    point_current, = ax_main.plot([], [], 'ko', markersize=10, markeredgecolor='blue', markeredgewidth=2)
    point_intersection, = ax_main.plot([], [], 'go', markersize=12, markeredgecolor='darkgreen', markeredgewidth=2, label='Predicted Intersection')
    vline_predicted = ax_main.axvline(x=0, color='orange', alpha=0, linewidth=2, linestyle='--', label='Predicted RTH Time')
    vline_actual = ax_main.axvline(x=rth_time, color='green', alpha=0, linewidth=2, linestyle=':', label='Actual RTH Time')
    
    ax_main.legend(loc='upper right', facecolor='white', edgecolor='gray', labelcolor='black')
    
    # Text annotation for prediction info
    info_text = ax_main.text(0.02, 0.02, '', transform=ax_main.transAxes, fontsize=11,
                             verticalalignment='bottom', fontfamily='monospace',
                             bbox=dict(boxstyle='round', facecolor='white', edgecolor='gray', alpha=0.9))
    
    # Storage for predictions
    predicted_times = []
    
    def init():
        """Initialize animation."""
        line_battery.set_data([], [])
        line_threshold.set_data([], [])
        line_regression.set_data([], [])
        point_current.set_data([], [])
        point_intersection.set_data([], [])
        hline_current_threshold.set_ydata([0, 0])
        hline_current_threshold.set_alpha(0)
        vline_predicted.set_alpha(0)
        vline_actual.set_alpha(0)
        info_text.set_text('')
        return (line_battery, line_threshold, line_regression, point_current, 
                point_intersection, hline_current_threshold, info_text)
    
    def animate(frame):
        """Update animation for each frame."""
        nonlocal predicted_times
        
        # Map frame to data index (skip some frames for speed)
        idx = min(int(frame * speed), len(times) - 1)
        
        if idx < 10:
            # Not enough data yet
            line_battery.set_data(times[:idx+1], battery[:idx+1])
            line_threshold.set_data(times[:idx+1], rth_threshold[:idx+1])
            point_current.set_data([times[idx]], [battery[idx]])
            predicted_times.append(np.nan)
            info_text.set_text(f'Time: {times[idx]:.1f}s\nCollecting data...')
            return (line_battery, line_threshold, line_regression, point_current,
                    point_intersection, hline_current_threshold, info_text)
        
        # Current data up to this point
        t_current = times[:idx+1]
        b_current = battery[:idx+1]
        thresh_current = rth_threshold[:idx+1]
        
        # Update battery and threshold lines
        line_battery.set_data(t_current, b_current)
        line_threshold.set_data(t_current, thresh_current)
        point_current.set_data([times[idx]], [battery[idx]])
        
        # Linear regression on ALL battery points so far
        time_until_rth = float('inf')
        predicted_rth_time = float('inf')
        drain_rate = 0
        
        try:
            coeffs = np.polyfit(t_current, b_current, 1)
            slope = coeffs[0]
            intercept = coeffs[1]
            drain_rate = -slope * 60  # %/min
            
            # Extend regression line to show prediction
            current_threshold = rth_threshold[idx]
            current_time = times[idx]
            
            if slope < 0:
                # Calculate predicted RTH time
                predicted_rth_time = (current_threshold - intercept) / slope
                time_until_rth = max(0, predicted_rth_time - current_time)
                
                # Draw regression line extending to prediction
                t_extend = np.linspace(0, min(predicted_rth_time + 50, max(times) * 1.1), 100)
                b_extend = slope * t_extend + intercept
                line_regression.set_data(t_extend, b_extend)
                
                # Show horizontal line at current threshold used for intersection
                hline_current_threshold.set_ydata([current_threshold, current_threshold])
                hline_current_threshold.set_alpha(0.7)
                
                # Mark intersection point
                predicted_battery_at_rth = slope * predicted_rth_time + intercept
                point_intersection.set_data([predicted_rth_time], [predicted_battery_at_rth])
                
                # Show vertical line at predicted RTH
                vline_predicted.set_xdata([predicted_rth_time, predicted_rth_time])
                vline_predicted.set_alpha(0.7)
                
                predicted_times.append(time_until_rth)
            else:
                line_regression.set_data([], [])
                point_intersection.set_data([], [])
                hline_current_threshold.set_alpha(0)
                vline_predicted.set_alpha(0)
                predicted_times.append(np.nan)
                
        except Exception as e:
            line_regression.set_data([], [])
            point_intersection.set_data([], [])
            hline_current_threshold.set_alpha(0)
            predicted_times.append(np.nan)
        
        # Show actual RTH line after it happens
        if times[idx] >= rth_time:
            vline_actual.set_alpha(0.8)
        
        # Calculate stats for info text
        actual_remaining = max(0, rth_time - times[idx])
        margin = battery[idx] - rth_threshold[idx]
        
        if time_until_rth != float('inf') and actual_remaining > 0:
            error = time_until_rth - actual_remaining
            info_str = (f'Time: {times[idx]:.0f}s  |  Battery: {battery[idx]:.0f}%  |  '
                       f'Margin: {margin:.1f}%  |  Drain: {drain_rate:.2f}%/min\n'
                       f'Predicted RTH: {time_until_rth:.0f}s  |  '
                       f'Actual RTH: {actual_remaining:.0f}s  |  '
                       f'Error: {error:+.0f}s')
        elif actual_remaining <= 0:
            info_str = (f'Time: {times[idx]:.0f}s  |  Battery: {battery[idx]:.0f}%  |  '
                       f'RTH TRIGGERED!')
        else:
            info_str = (f'Time: {times[idx]:.0f}s  |  Battery: {battery[idx]:.0f}%  |  '
                       f'Margin: {margin:.1f}%')
        
        info_text.set_text(info_str)
        
        return (line_battery, line_threshold, line_regression, point_current,
                point_intersection, info_text)
    
    # Calculate number of frames
    n_frames = int(len(times) / speed) + 1
    
    # Create animation
    print(f"Creating animation with {n_frames} frames...")
    anim = animation.FuncAnimation(fig, animate, init_func=init,
                                   frames=n_frames, interval=50, blit=False)
    
    plt.tight_layout()
    
    # Save if output file specified
    if output_file:
        print(f"Saving animation to {output_file}...")
        if output_file.endswith('.gif'):
            anim.save(output_file, writer='pillow', fps=20)
        else:
            anim.save(output_file, writer='ffmpeg', fps=20, dpi=100)
        print(f"Animation saved: {output_file}")
    
    # Show animation
    if show:
        plt.show()
    
    plt.close()


def main():
    parser = argparse.ArgumentParser(description='RTH Prediction Animation')
    parser.add_argument('--file', '-f', type=str, help='Specific CSV file to analyze')
    parser.add_argument('--output', '-o', type=str, help='Output file (MP4 or GIF)')
    parser.add_argument('--speed', '-s', type=float, default=2.0, help='Animation speed multiplier (default: 2.0)')
    parser.add_argument('--no-show', action='store_true', help='Do not display animation (only save)')
    parser.add_argument('--list', '-l', action='store_true', help='List available flight files')
    
    args = parser.parse_args()
    
    # Find CSV files
    csv_files = sorted(glob.glob('rth_analysis_*.csv'))
    
    if args.list:
        print("Available flight files:")
        for i, f in enumerate(csv_files):
            print(f"  {i+1}. {f}")
        return
    
    if args.file:
        if os.path.exists(args.file):
            csv_file = args.file
        else:
            print(f"File not found: {args.file}")
            return
    else:
        # Use the most recent file with RTH event
        csv_file = None
        for f in reversed(csv_files):
            df = pd.read_csv(f)
            if 'flight_mode' in df.columns:
                if df['flight_mode'].str.contains('GO_HOME').any():
                    csv_file = f
                    break
        
        if csv_file is None:
            print("No flight files with RTH events found!")
            print("Available files:", csv_files)
            return
    
    create_rth_animation(csv_file, args.output, args.speed, not args.no_show)


if __name__ == "__main__":
    main()

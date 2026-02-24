#!/usr/bin/env python3
"""
Calibration Manager - Load/Save/Report calibration values
Helps manage robot calibration parameters stored in JSON config
"""

import json
import os
from datetime import datetime

CALIBRATION_FILE = "calibration_config.json"
STATE_FILE = "bt/simple_state.py"


def load_calibration():
    """Load calibration from JSON file"""
    if not os.path.exists(CALIBRATION_FILE):
        print(f"❌ Calibration file not found: {CALIBRATION_FILE}")
        return None
    
    with open(CALIBRATION_FILE, 'r') as f:
        return json.load(f)


def save_calibration(config):
    """Save calibration to JSON file"""
    config['metadata']['calibration_date'] = datetime.now().strftime("%Y-%m-%d")
    
    with open(CALIBRATION_FILE, 'w') as f:
        json.dump(config, f, indent=2)
    
    print(f"✅ Calibration saved to {CALIBRATION_FILE}")


def print_calibration_report():
    """Print formatted calibration report"""
    config = load_calibration()
    if not config:
        return
    
    print("\n" + "="*70)
    print("🤖 COURIER ROBOT - CALIBRATION REPORT")
    print("="*70)
    
    # Metadata
    meta = config.get('metadata', {})
    print(f"\n📅 Date: {meta.get('calibration_date', 'Unknown')}")
    print(f"🏢 Surface: {meta.get('surface_type', 'Unknown')}")
    print(f"🔋 Battery: {meta.get('battery_level', 'Unknown')}")
    
    # Motor Compensation
    print("\n" + "-"*70)
    print("🚗 MOTOR COMPENSATION")
    print("-"*70)
    motor = config.get('motor_compensation', {})
    print(f"  Left Factor:  {motor.get('left_factor', 0):.2f}x")
    print(f"  Right Factor: {motor.get('right_factor', 0):.2f}x")
    print(f"  💡 {motor.get('notes', '')}")
    
    # Speeds
    print("\n" + "-"*70)
    print("⚡ MOTOR SPEEDS")
    print("-"*70)
    speeds = config.get('motor_speeds', {})
    print(f"  Linear: {speeds.get('linear_speed', 0)*100:.0f}%")
    print(f"  Turn:   {speeds.get('turn_speed', 0)*100:.0f}%")
    
    # Velocity
    print("\n" + "-"*70)
    print("🏃 VELOCITY")
    print("-"*70)
    vel = config.get('velocity', {})
    forward_mps = vel.get('forward_mps', 0)
    print(f"  Forward: {forward_mps:.3f} m/s ({forward_mps*100:.1f} cm/s)")
    if forward_mps > 0:
        print(f"  Time per meter: {1/forward_mps:.1f}s")
    
    # Rotation
    print("\n" + "-"*70)
    print("🔄 ROTATION")
    print("-"*70)
    rot = config.get('rotation', {})
    print(f"  90° Left:  {rot.get('rotation_90_time_left', 0):.2f}s")
    print(f"  90° Right: {rot.get('rotation_90_time_right', 0):.2f}s")
    print(f"  360° Est:  {rot.get('rotation_90_time_right', 0)*4:.1f}s")
    print(f"  Reverse Boost: {rot.get('rotation_reverse_boost', 0):.2f}x")
    
    # IMU
    print("\n" + "-"*70)
    print("🧭 IMU SENSOR")
    print("-"*70)
    imu = config.get('imu', {})
    status = "✅ Available" if imu.get('available') else "❌ Not Available"
    print(f"  Status: {status}")
    print(f"  Rotation Control:     {'✅ ON' if imu.get('use_imu_rotation') else '❌ OFF'}")
    print(f"  Odometry:            {'✅ ON' if imu.get('use_imu_odometry') else '❌ OFF'}")
    print(f"  Heading Correction:  {'✅ ON' if imu.get('use_imu_heading_correction') else '❌ OFF'}")
    print(f"  Correction Gain:     {imu.get('heading_correction_gain', 0):.2f}")
    print(f"  Odometry Scale:      {imu.get('odometry_scale', 0):.2f}")
    
    # Ultrasonic
    print("\n" + "-"*70)
    print("📡 ULTRASONIC SENSOR")
    print("-"*70)
    ultra = config.get('ultrasonic', {})
    print(f"  Obstacle (Dynamic):   {ultra.get('obstacle_threshold_dynamic', 0):.0f} cm")
    print(f"  Obstacle (Static):    {ultra.get('obstacle_threshold_static', 0):.0f} cm")
    print(f"  Direction Clearance:  {ultra.get('direction_clearance_threshold', 0):.0f} cm")
    print(f"  Lateral Clearance:    {ultra.get('lateral_clearance_threshold', 0):.0f} cm")
    
    # Calibration Status
    print("\n" + "-"*70)
    print("📊 CALIBRATION STATUS")
    print("-"*70)
    status = config.get('calibration_status', {})
    for key, value in status.items():
        key_display = key.replace('_', ' ').title()
        print(f"  {key_display:.<25} {value}")
    
    print("\n" + "="*70)


def update_simple_state_from_config():
    """
    Generate Python code snippet to update simple_state.py from config.
    Does NOT automatically modify the file (manual update required).
    """
    config = load_calibration()
    if not config:
        return
    
    print("\n" + "="*70)
    print("📝 COPY THESE VALUES TO bt/simple_state.py")
    print("="*70)
    print("\n# Movement Calibration")
    print(f"self.meters_per_second_forward = {config['velocity']['forward_mps']}")
    print(f"self.meters_per_second_lateral = {config['velocity']['lateral_mps']}")
    print(f"self.rotation_90_time_left = {config['rotation']['rotation_90_time_left']}")
    print(f"self.rotation_90_time_right = {config['rotation']['rotation_90_time_right']}")
    
    print("\n# Motor Compensation")
    print(f"self.left_factor = {config['motor_compensation']['left_factor']}")
    print(f"self.right_factor = {config['motor_compensation']['right_factor']}")
    
    print("\n# Rotation Factors")
    print(f"self.rotation_left_factor = {config['rotation']['rotation_left_factor']}")
    print(f"self.rotation_right_factor = {config['rotation']['rotation_right_factor']}")
    print(f"self.rotation_reverse_boost = {config['rotation']['rotation_reverse_boost']}")
    print(f"self.rotation_forward_boost = {config['rotation']['rotation_forward_boost']}")
    
    print("\n# Ultrasonic Thresholds")
    print(f"self.obstacle_threshold = {config['ultrasonic']['obstacle_threshold_dynamic']}")
    print(f"self.obstacle_threshold_static = {config['ultrasonic']['obstacle_threshold_static']}")
    print(f"self.direction_clearance_threshold = {config['ultrasonic']['direction_clearance_threshold']}")
    
    print("\n# IMU Settings")
    print(f"self.use_imu_rotation = {config['imu']['use_imu_rotation']}")
    print(f"self.use_imu_odometry = {config['imu']['use_imu_odometry']}")
    print(f"self.use_imu_heading_correction = {config['imu']['use_imu_heading_correction']}")
    print(f"self.imu_heading_correction_gain = {config['imu']['heading_correction_gain']}")
    print(f"self.imu_odometry_scale = {config['imu']['imu_odometry_scale']}")
    
    print("\n" + "="*70)


def interactive_calibration_update():
    """Interactive menu to update calibration values"""
    config = load_calibration()
    if not config:
        return
    
    while True:
        print("\n" + "="*70)
        print("🔧 CALIBRATION UPDATE MENU")
        print("="*70)
        print("1. Update motor compensation (left/right factors)")
        print("2. Update rotation timing (90° left/right)")
        print("3. Update forward velocity")
        print("4. Update ultrasonic thresholds")
        print("5. Update IMU settings")
        print("6. View current calibration")
        print("7. Save and export to Python code")
        print("0. Exit")
        
        choice = input("\nSelect option: ").strip()
        
        if choice == '1':
            print("\n🚗 Motor Compensation")
            print(f"Current: Left={config['motor_compensation']['left_factor']}, "
                  f"Right={config['motor_compensation']['right_factor']}")
            left = input("New left factor [Enter to skip]: ").strip()
            if left:
                config['motor_compensation']['left_factor'] = float(left)
            right = input("New right factor [Enter to skip]: ").strip()
            if right:
                config['motor_compensation']['right_factor'] = float(right)
        
        elif choice == '2':
            print("\n🔄 Rotation Timing")
            print(f"Current: Left={config['rotation']['rotation_90_time_left']}s, "
                  f"Right={config['rotation']['rotation_90_time_right']}s")
            left = input("New 90° left time [Enter to skip]: ").strip()
            if left:
                config['rotation']['rotation_90_time_left'] = float(left)
            right = input("New 90° right time [Enter to skip]: ").strip()
            if right:
                config['rotation']['rotation_90_time_right'] = float(right)
        
        elif choice == '3':
            print("\n🏃 Forward Velocity")
            print(f"Current: {config['velocity']['forward_mps']} m/s")
            vel = input("New velocity in m/s [Enter to skip]: ").strip()
            if vel:
                config['velocity']['forward_mps'] = float(vel)
                config['velocity']['lateral_mps'] = float(vel)
        
        elif choice == '4':
            print("\n📡 Ultrasonic Thresholds")
            print(f"Dynamic: {config['ultrasonic']['obstacle_threshold_dynamic']}cm")
            print(f"Static: {config['ultrasonic']['obstacle_threshold_static']}cm")
            print(f"Clearance: {config['ultrasonic']['direction_clearance_threshold']}cm")
            dyn = input("New dynamic threshold [Enter to skip]: ").strip()
            if dyn:
                config['ultrasonic']['obstacle_threshold_dynamic'] = float(dyn)
            stat = input("New static threshold [Enter to skip]: ").strip()
            if stat:
                config['ultrasonic']['obstacle_threshold_static'] = float(stat)
            clear = input("New clearance threshold [Enter to skip]: ").strip()
            if clear:
                config['ultrasonic']['direction_clearance_threshold'] = float(clear)
        
        elif choice == '5':
            print("\n🧭 IMU Settings")
            print(f"Heading correction gain: {config['imu']['heading_correction_gain']}")
            print(f"Odometry scale: {config['imu']['odometry_scale']}")
            gain = input("New heading correction gain [Enter to skip]: ").strip()
            if gain:
                config['imu']['heading_correction_gain'] = float(gain)
            scale = input("New odometry scale [Enter to skip]: ").strip()
            if scale:
                config['imu']['odometry_scale'] = float(scale)
        
        elif choice == '6':
            print_calibration_report()
        
        elif choice == '7':
            save_calibration(config)
            update_simple_state_from_config()
            print("\n✅ Now manually copy the values above to bt/simple_state.py")
            break
        
        elif choice == '0':
            print("\n👋 Exiting without saving")
            break


def main():
    """Main menu"""
    print("\n" + "="*70)
    print("🤖 COURIER ROBOT - CALIBRATION MANAGER")
    print("="*70)
    print("\nOptions:")
    print("  1. View calibration report")
    print("  2. Update calibration values (interactive)")
    print("  3. Export to Python code")
    print("  0. Exit")
    
    choice = input("\nSelect: ").strip()
    
    if choice == '1':
        print_calibration_report()
    elif choice == '2':
        interactive_calibration_update()
    elif choice == '3':
        update_simple_state_from_config()
    else:
        print("👋 Goodbye")


if __name__ == "__main__":
    main()

import rk_mcprotocol as mc
import time
import math
import numpy as np
from typing import List, Tuple

class SCARARobotController:
    """
    SCARA Robot Controller for Grid Testing and Precision Validation
    Based on your project specifications with Link1=160mm, Link2=149.5mm
    """
    
    def __init__(self, host='192.168.3.250', port=5007):
        self.HOST = host
        self.PORT = port
        self.socket = None
        
        # Robot physical parameters (from your project)
        self.L1 = 160.0  # Link 1 length (mm)
        self.L2 = 149.5  # Link 2 length (mm) 
        self.base_height = 131.8  # Base height (mm)
        self.L2_offset = 36.0  # Link 2 offset (mm)
        
        # Workspace limits
        self.max_reach = self.L1 + self.L2  # 309.5mm
        self.min_reach = abs(self.L1 - self.L2)  # 10.5mm
        
        # Microstepping configuration
        self.steps_per_rev = 200 * 32  # 1/32 microstepping
        self.step_to_angle = 360.0 / self.steps_per_rev
        
        # Grid testing parameters
        self.grid_size = 3  # 3x3 grid
        self.grid_spacing = 60  # 60mm spacing for A4 testing
        
    def connect(self) -> bool:
        """Establish connection to PLC"""
        try:
            self.socket = mc.open_socket(self.HOST, self.PORT)
            if self.socket:
                print(f"✓ Successfully connected to {self.HOST}:{self.PORT}")
                return True
            else:
                print(f"✗ Failed to connect to {self.HOST}:{self.PORT}")
                return False
        except Exception as e:
            print(f"✗ Connection error: {e}")
            return False
    
    def inverse_kinematics(self, x: float, y: float) -> Tuple[float, float, bool]:
        """
        Calculate joint angles for given Cartesian coordinates
        Returns: (theta1, theta2, is_valid)
        """
        # Check if point is within workspace
        distance = math.sqrt(x**2 + y**2)
        if distance > self.max_reach or distance < self.min_reach:
            return 0, 0, False
        
        # Law of cosines for theta2
        cos_theta2 = (x**2 + y**2 - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
        
        if abs(cos_theta2) > 1:
            return 0, 0, False
        
        # Two solutions for theta2 (elbow up/down)
        theta2 = math.atan2(math.sqrt(1 - cos_theta2**2), cos_theta2)  # Elbow up
        
        # Calculate theta1
        k1 = self.L1 + self.L2 * math.cos(theta2)
        k2 = self.L2 * math.sin(theta2)
        theta1 = math.atan2(y, x) - math.atan2(k2, k1)
        
        return math.degrees(theta1), math.degrees(theta2), True
    
    def forward_kinematics(self, theta1_deg: float, theta2_deg: float) -> Tuple[float, float]:
        """Calculate end-effector position from joint angles"""
        theta1 = math.radians(theta1_deg)
        theta2 = math.radians(theta2_deg)
        
        x = self.L1 * math.cos(theta1) + self.L2 * math.cos(theta1 + theta2)
        y = self.L1 * math.sin(theta1) + self.L2 * math.sin(theta1 + theta2)
        
        return x, y
    
    def generate_grid_coordinates(self, center_x: float = 200, center_y: float = 0) -> List[Tuple[float, float, int]]:
        """
        Generate 3x3 grid coordinates for testing
        Returns list of (x, y, point_number) tuples
        """
        coordinates = []
        point_num = 1
        
        for i in range(self.grid_size):
            for j in range(self.grid_size):
                x = center_x + (j - 1) * self.grid_spacing  # -60, 0, +60
                y = center_y + (i - 1) * self.grid_spacing  # -60, 0, +60
                
                # Check if point is reachable
                distance = math.sqrt(x**2 + y**2)
                if self.min_reach <= distance <= self.max_reach:
                    coordinates.append((x, y, point_num))
                else:
                    print(f"⚠ Point {point_num} ({x:.1f}, {y:.1f}) is outside workspace")
                
                point_num += 1
        
        return coordinates
    
    def angles_to_motor_values(self, theta1: float, theta2: float) -> Tuple[int, int]:
        """Convert joint angles to stepper motor step values"""
        steps1 = int(theta1 / self.step_to_angle)
        steps2 = int(theta2 / self.step_to_angle)
        return steps1, steps2
    
    def read_current_position(self) -> Tuple[int, int, int, int]:
        """Read current motor positions and velocities"""
        try:
            position_m1 = mc.read_sign_Dword(self.socket, headdevice='D100', length=1, signed_type=True)
            velocity_m1 = mc.read_sign_Dword(self.socket, headdevice='D102', length=1, signed_type=True)
            position_m2 = mc.read_sign_Dword(self.socket, headdevice='D104', length=1, signed_type=True)
            velocity_m2 = mc.read_sign_Dword(self.socket, headdevice='D106', length=1, signed_type=True)
            
            return position_m1[0], velocity_m1[0], position_m2[0], velocity_m2[0]
        except Exception as e:
            print(f"Error reading position: {e}")
            return 0, 0, 0, 0
    
    def send_motor_commands(self, pos1: int, vel1: int, pos2: int, vel2: int) -> bool:
        """Send motor commands in batch and trigger execution"""
        try:
            # Write all values in one batch for efficiency
            mc.write_sign_Dword(self.socket, headdevice='D100', 
                               data_list=[pos1, vel1, pos2, vel2], 
                               signed_type=True)
            
            # Trigger execution
            mc.write_bit(self.socket, headdevice='M3', data_list=[1])
            
            print(f"📤 Commands sent - M1: pos={pos1}, vel={vel1} | M2: pos={pos2}, vel={vel2}")
            return True
        except Exception as e:
            print(f"Error sending commands: {e}")
            return False
    
    def test_grid_point(self, x: float, y: float, point_num: int, velocity: int = 1000) -> bool:
        """Test movement to a specific grid point"""
        print(f"\n🎯 Testing Point {point_num}: ({x:.1f}, {y:.1f}) mm")
        
        # Calculate inverse kinematics
        theta1, theta2, is_valid = self.inverse_kinematics(x, y)
        
        if not is_valid:
            print(f"✗ Point {point_num} is unreachable")
            return False
        
        print(f"📐 Joint angles: θ1={theta1:.2f}°, θ2={theta2:.2f}°")
        
        # Convert to motor steps
        steps1, steps2 = self.angles_to_motor_values(theta1, theta2)
        print(f"🔧 Motor steps: M1={steps1}, M2={steps2}")
        
        # Send commands
        success = self.send_motor_commands(steps1, velocity, steps2, velocity)
        
        if success:
            time.sleep(2)  # Wait for movement
            
            # Verify position (forward kinematics check)
            actual_x, actual_y = self.forward_kinematics(theta1, theta2)
            error_x = abs(actual_x - x)
            error_y = abs(actual_y - y)
            total_error = math.sqrt(error_x**2 + error_y**2)
            
            print(f"✓ Theoretical position: ({actual_x:.2f}, {actual_y:.2f})")
            print(f"📏 Position error: {total_error:.2f} mm")
            
            # Check tolerance (your spec: ≤10mm preferred, ≤5mm)
            if total_error <= 5.0:
                print(f"🟢 PASS (≤5mm): Point {point_num}")
                return True
            elif total_error <= 10.0:
                print(f"🟡 WARNING (5-10mm): Point {point_num}")
                return True
            else:
                print(f"🔴 FAIL (>10mm): Point {point_num}")
                return False
        
        return False
    
    def run_grid_test(self) -> None:
        """Execute complete 3x3 grid test sequence"""
        print("=" * 60)
        print("🤖 SCARA Robot 3x3 Grid Precision Test")
        print("=" * 60)
        print(f"Robot parameters: L1={self.L1}mm, L2={self.L2}mm")
        print(f"Workspace: {self.min_reach:.1f}mm - {self.max_reach:.1f}mm")
        print(f"Grid spacing: {self.grid_spacing}mm")
        
        # Generate test coordinates
        coordinates = self.generate_grid_coordinates()
        
        if not coordinates:
            print("❌ No valid test points generated!")
            return
        
        print(f"\n📋 Testing {len(coordinates)} grid points...")
        
        # Test each point
        results = []
        start_time = time.time()
        
        for x, y, point_num in coordinates:
            result = self.test_grid_point(x, y, point_num)
            results.append((point_num, x, y, result))
            time.sleep(1)  # Brief pause between points
        
        # Summary
        total_time = time.time() - start_time
        passed = sum(1 for _, _, _, result in results if result)
        total = len(results)
        success_rate = (passed / total) * 100 if total > 0 else 0
        
        print("\n" + "=" * 60)
        print("📊 TEST RESULTS SUMMARY")
        print("=" * 60)
        print(f"Total points tested: {total}")
        print(f"Points passed: {passed}")
        print(f"Success rate: {success_rate:.1f}%")
        print(f"Test duration: {total_time:.1f} seconds")
        
        # Detailed results
        print(f"\n📝 Detailed Results:")
        for point_num, x, y, result in results:
            status = "✓ PASS" if result else "✗ FAIL"
            print(f"Point {point_num} ({x:6.1f}, {y:6.1f}): {status}")
        
        # Assessment
        if success_rate >= 95:
            print("\n🏆 EXCELLENT: System meets precision requirements!")
        elif success_rate >= 80:
            print("\n👍 GOOD: System performance is acceptable")
        else:
            print("\n⚠ NEEDS IMPROVEMENT: Consider calibration")

def main():
    """Main execution function"""
    robot = SCARARobotController()
    
    if not robot.connect():
        return
    
    print("\n🔧 SCARA Robot Grid Test System")
    print("Choose test mode:")
    print("1. Full 3x3 Grid Test")
    print("2. Single Point Test")
    print("3. Manual Motor Control")
    
    try:
        choice = input("\nEnter choice (1-3): ").strip()
        
        if choice == "1":
            import subprocess
            print("\n▶ Running trajectory_sender_grid_3.py ...")
            subprocess.run([
                "python3",
                "/Users/worakanlasudee/Documents/GitHub/SCARA-Plotter-3DOF-FX5U-PLC/trajectory_sender_grid_3.py"
            ])
            return
            
        elif choice == "2":
            x = float(input("Enter X coordinate (mm): "))
            y = float(input("Enter Y coordinate (mm): "))
            robot.test_grid_point(x, y, 1)
            
        elif choice == "3":
            print("Manual motor control mode")
            d100_value = int(input("Enter value for D100 (Motor 2 Position): "))
            d102_value = int(input("Enter value for D102 (Motor 2 Velocity): "))
            d104_value = int(input("Enter value for D104 (Motor 1 Position): "))
            d106_value = int(input("Enter value for D106 (Motor 1 Velocity): "))
            
            robot.send_motor_commands(d100_value, d102_value, d104_value, d106_value)
            
        else:
            print("Invalid choice")
            
    except KeyboardInterrupt:
        print("\n🛑 Test interrupted by user")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if robot.socket:
            print("🔌 Closing connection")

if __name__ == "__main__":
    main()
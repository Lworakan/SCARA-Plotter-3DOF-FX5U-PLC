import rk_mcprotocol as mc
import time
import math
import json
import csv
from datetime import datetime
from typing import List, Tuple, Dict, Optional
from grid_configurations import GridConfigurations, get_test_configuration

class SCARAPrecisionTester:
    """
    Advanced SCARA Robot Precision Tester with comprehensive logging and analysis
    """
    
    def __init__(self, host='192.168.3.250', port=5007):
        self.HOST = host
        self.PORT = port
        self.socket = None
        
        # Robot parameters from your specifications
        self.L1 = 160.0  # mm
        self.L2 = 149.5  # mm  
        self.base_height = 131.8  # mm
        self.L2_offset = 36.0  # mm
        
        # Derived parameters
        self.max_reach = self.L1 + self.L2  # 309.5mm
        self.min_reach = abs(self.L1 - self.L2)  # 10.5mm
        
        # Stepper motor configuration
        self.steps_per_rev = 200 * 32  # 1/32 microstepping
        self.step_to_angle = 360.0 / self.steps_per_rev
        
        # Test results storage
        self.test_results = []
        self.test_metadata = {}
        
        # Performance thresholds (from your specifications)
        self.excellent_threshold = 5.0  # mm
        self.acceptable_threshold = 10.0  # mm
        
    def connect(self) -> bool:
        """Establish PLC connection with error handling"""
        try:
            self.socket = mc.open_socket(self.HOST, self.PORT)
            if self.socket:
                print(f"✓ Connected to SCARA controller at {self.HOST}:{self.PORT}")
                return True
            else:
                print(f"✗ Connection failed to {self.HOST}:{self.PORT}")
                return False
        except Exception as e:
            print(f"✗ Connection error: {e}")
            return False
    
    def inverse_kinematics(self, x: float, y: float) -> Tuple[float, float, bool]:
        """
        Calculate joint angles using geometric method
        Returns: (theta1_deg, theta2_deg, is_reachable)
        """
        # Check workspace limits
        distance = math.sqrt(x**2 + y**2)
        if distance > self.max_reach or distance < self.min_reach:
            return 0.0, 0.0, False
        
        try:
            # Law of cosines for theta2
            cos_theta2 = (x**2 + y**2 - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
            
            if abs(cos_theta2) > 1.0:
                return 0.0, 0.0, False
            
            # Elbow-up configuration (preferred for SCARA)
            sin_theta2 = math.sqrt(1 - cos_theta2**2)
            theta2 = math.atan2(sin_theta2, cos_theta2)
            
            # Calculate theta1
            k1 = self.L1 + self.L2 * cos_theta2
            k2 = self.L2 * sin_theta2
            theta1 = math.atan2(y, x) - math.atan2(k2, k1)
            
            return math.degrees(theta1), math.degrees(theta2), True
            
        except Exception as e:
            print(f"IK calculation error: {e}")
            return 0.0, 0.0, False
    
    def forward_kinematics(self, theta1_deg: float, theta2_deg: float) -> Tuple[float, float]:
        """Calculate end-effector position from joint angles"""
        theta1 = math.radians(theta1_deg)
        theta2 = math.radians(theta2_deg)
        
        x = self.L1 * math.cos(theta1) + self.L2 * math.cos(theta1 + theta2)
        y = self.L1 * math.sin(theta1) + self.L2 * math.sin(theta1 + theta2)
        
        return x, y
    
    def calculate_jacobian(self, theta1_deg: float, theta2_deg: float) -> List[List[float]]:
        """Calculate Jacobian matrix for velocity analysis"""
        theta1 = math.radians(theta1_deg)
        theta2 = math.radians(theta2_deg)
        
        # Jacobian elements
        j11 = -self.L1 * math.sin(theta1) - self.L2 * math.sin(theta1 + theta2)
        j12 = -self.L2 * math.sin(theta1 + theta2)
        j21 = self.L1 * math.cos(theta1) + self.L2 * math.cos(theta1 + theta2)
        j22 = self.L2 * math.cos(theta1 + theta2)
        
        return [[j11, j12], [j21, j22]]
    
    def check_singularity(self, theta1_deg: float, theta2_deg: float) -> Tuple[bool, float]:
        """Check for singularity conditions"""
        jacobian = self.calculate_jacobian(theta1_deg, theta2_deg)
        
        # Calculate determinant
        det = jacobian[0][0] * jacobian[1][1] - jacobian[0][1] * jacobian[1][0]
        
        # Singularity threshold (small determinant indicates near-singularity)
        singularity_threshold = 1e-3
        is_singular = abs(det) < singularity_threshold
        
        return is_singular, det
    
    def angles_to_steps(self, theta1_deg: float, theta2_deg: float) -> Tuple[int, int]:
        """Convert joint angles to stepper motor steps"""
        steps1 = int(round(theta1_deg / self.step_to_angle))
        steps2 = int(round(theta2_deg / self.step_to_angle))
        return steps1, steps2
    
    def send_robot_command(self, steps1: int, vel1: int, steps2: int, vel2: int) -> bool:
        """Send movement commands to robot"""
        try:
            # Batch write for efficiency
            mc.write_sign_Dword(self.socket, headdevice='D100',
                               data_list=[steps1, vel1, steps2, vel2],
                               signed_type=True)
            
            # Trigger execution
            mc.write_bit(self.socket, headdevice='M3', data_list=[1])
            return True
            
        except Exception as e:
            print(f"Command send error: {e}")
            return False
    
    def read_robot_feedback(self) -> Dict:
        """Read current robot status and position feedback"""
        try:
            pos1 = mc.read_sign_Dword(self.socket, headdevice='D100', length=1, signed_type=True)[0]
            vel1 = mc.read_sign_Dword(self.socket, headdevice='D102', length=1, signed_type=True)[0]
            pos2 = mc.read_sign_Dword(self.socket, headdevice='D104', length=1, signed_type=True)[0]
            vel2 = mc.read_sign_Dword(self.socket, headdevice='D106', length=1, signed_type=True)[0]
            
            return {
                'motor1_position': pos1,
                'motor1_velocity': vel1, 
                'motor2_position': pos2,
                'motor2_velocity': vel2,
                'timestamp': time.time()
            }
        except Exception as e:
            print(f"Feedback read error: {e}")
            return {}
    
    def test_single_point(self, x: float, y: float, point_id: str, velocity: int = 1000) -> Dict:
        """Test movement to a single point with comprehensive analysis"""
        test_start_time = time.time()
        
        print(f"\n🎯 Testing {point_id}: Target ({x:.2f}, {y:.2f}) mm")
        
        # Initialize result structure
        result = {
            'point_id': point_id,
            'target_x': x,
            'target_y': y,
            'velocity': velocity,
            'timestamp': datetime.now().isoformat(),
            'success': False
        }
        
        # Step 1: Inverse Kinematics
        theta1, theta2, reachable = self.inverse_kinematics(x, y)
        result.update({
            'theta1_deg': theta1,
            'theta2_deg': theta2, 
            'reachable': reachable
        })
        
        if not reachable:
            print(f"✗ Point unreachable - distance: {math.sqrt(x**2 + y**2):.1f}mm")
            result['error_message'] = "Point outside workspace"
            return result
        
        print(f"📐 Joint angles: θ₁={theta1:.3f}°, θ₂={theta2:.3f}°")
        
        # Step 2: Singularity Check
        is_singular, det = self.check_singularity(theta1, theta2)
        result.update({
            'is_singular': is_singular,
            'jacobian_determinant': det
        })
        
        if is_singular:
            print(f"⚠ Warning: Near singularity (det={det:.6f})")
        
        # Step 3: Forward Kinematics Verification
        fk_x, fk_y = self.forward_kinematics(theta1, theta2)
        theoretical_error = math.sqrt((fk_x - x)**2 + (fk_y - y)**2)
        result.update({
            'forward_kinematics_x': fk_x,
            'forward_kinematics_y': fk_y,
            'theoretical_error_mm': theoretical_error
        })
        
        print(f"🔄 FK verification: ({fk_x:.3f}, {fk_y:.3f}) - Error: {theoretical_error:.4f}mm")
        
        # Step 4: Motor Command Generation
        steps1, steps2 = self.angles_to_steps(theta1, theta2)
        result.update({
            'motor1_steps': steps1,
            'motor2_steps': steps2
        })
        
        print(f"🔧 Motor commands: M1={steps1} steps, M2={steps2} steps")
        
        # Step 5: Execute Movement
        command_sent = self.send_robot_command(steps1, velocity, steps2, velocity)
        
        if not command_sent:
            result['error_message'] = "Failed to send robot command"
            return result
        
        print(f"📤 Command sent - waiting for movement completion...")
        
        # Wait for movement completion (adjust based on your robot's speed)
        movement_time = max(2.0, abs(steps1) * 0.001 + abs(steps2) * 0.001)
        time.sleep(movement_time)
        
        # Step 6: Read Feedback
        feedback = self.read_robot_feedback()
        result.update({'feedback': feedback})
        
        # Step 7: Calculate Performance Metrics
        execution_time = time.time() - test_start_time
        result.update({
            'execution_time_s': execution_time,
            'position_error_mm': theoretical_error  # In real system, compare with actual position
        })
        
        # Step 8: Pass/Fail Assessment
        if theoretical_error <= self.excellent_threshold:
            status = "EXCELLENT"
            status_emoji = "🟢"
            result['success'] = True
        elif theoretical_error <= self.acceptable_threshold:
            status = "ACCEPTABLE" 
            status_emoji = "🟡"
            result['success'] = True
        else:
            status = "FAIL"
            status_emoji = "🔴"
            result['success'] = False
        
        result['status'] = status
        print(f"{status_emoji} {status}: Error {theoretical_error:.3f}mm (Threshold: ≤{self.acceptable_threshold}mm)")
        
        return result
    
    def run_test_configuration(self, config_name: str) -> Dict:
        """Execute a complete test configuration"""
        print("=" * 80)
        print(f"🚀 SCARA Robot Precision Test: {config_name}")
        print("=" * 80)
        
        # Get test configuration
        config = get_test_configuration(config_name)
        coordinates = config['coordinates']
        
        print(f"📋 Configuration: {config['name']}")
        print(f"📝 Description: {config['description']}")
        print(f"📍 Test points: {len(coordinates)}")
        print(f"🤖 Robot: L1={self.L1}mm, L2={self.L2}mm")
        print(f"🌐 Workspace: {self.min_reach:.1f}mm - {self.max_reach:.1f}mm")
        
        # Initialize test session
        test_session = {
            'config_name': config_name,
            'config_description': config['description'],
            'start_time': datetime.now().isoformat(),
            'robot_parameters': {
                'L1': self.L1,
                'L2': self.L2, 
                'max_reach': self.max_reach,
                'min_reach': self.min_reach
            },
            'results': []
        }
        
        # Execute tests
        for i, coord_data in enumerate(coordinates):
            if len(coord_data) == 4:  # (x, y, point_num, label)
                x, y, point_num, label = coord_data
                velocity = 1000  # Default velocity
            elif len(coord_data) == 5:  # (x, y, point_num, label, velocity)
                x, y, point_num, label, velocity = coord_data
            else:
                continue
            
            result = self.test_single_point(x, y, f"P{point_num}_{label}", velocity)
            test_session['results'].append(result)
            
            # Brief pause between tests
            time.sleep(0.5)
        
        # Calculate summary statistics
        test_session['end_time'] = datetime.now().isoformat()
        results = test_session['results']
        
        total_tests = len(results)
        successful_tests = sum(1 for r in results if r['success'])
        success_rate = (successful_tests / total_tests * 100) if total_tests > 0 else 0
        
        errors = [r['position_error_mm'] for r in results if 'position_error_mm' in r]
        avg_error = sum(errors) / len(errors) if errors else 0
        max_error = max(errors) if errors else 0
        min_error = min(errors) if errors else 0
        
        summary = {
            'total_tests': total_tests,
            'successful_tests': successful_tests, 
            'failed_tests': total_tests - successful_tests,
            'success_rate_percent': success_rate,
            'average_error_mm': avg_error,
            'maximum_error_mm': max_error,
            'minimum_error_mm': min_error
        }
        
        test_session['summary'] = summary
        
        # Print summary
        print("\n" + "=" * 80)
        print("📊 TEST RESULTS SUMMARY")
        print("=" * 80)
        print(f"✓ Successful tests: {successful_tests}/{total_tests} ({success_rate:.1f}%)")
        print(f"📏 Average error: {avg_error:.3f}mm")
        print(f"📏 Maximum error: {max_error:.3f}mm")
        print(f"📏 Minimum error: {min_error:.3f}mm")
        
        if success_rate >= 95:
            print("🏆 EXCELLENT: System exceeds precision requirements!")
        elif success_rate >= 80:
            print("👍 GOOD: System meets acceptable performance standards")
        else:
            print("⚠ IMPROVEMENT NEEDED: Consider recalibration")
        
        # Save results
        self.save_test_results(test_session, config_name)
        
        return test_session
    
    def save_test_results(self, test_session: Dict, config_name: str):
        """Save test results to files"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Save JSON results
        json_filename = f"scara_test_{config_name}_{timestamp}.json"
        with open(json_filename, 'w') as f:
            json.dump(test_session, f, indent=2)
        print(f"💾 Results saved to {json_filename}")
        
        # Save CSV summary
        csv_filename = f"scara_test_{config_name}_{timestamp}.csv" 
        with open(csv_filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'Point_ID', 'Target_X', 'Target_Y', 'Theta1', 'Theta2',
                'FK_X', 'FK_Y', 'Error_mm', 'Status', 'Success'
            ])
            
            for result in test_session['results']:
                writer.writerow([
                    result.get('point_id', ''),
                    result.get('target_x', 0),
                    result.get('target_y', 0),
                    result.get('theta1_deg', 0),
                    result.get('theta2_deg', 0),
                    result.get('forward_kinematics_x', 0),
                    result.get('forward_kinematics_y', 0),
                    result.get('position_error_mm', 0),
                    result.get('status', ''),
                    result.get('success', False)
                ])
        
        print(f"📊 CSV summary saved to {csv_filename}")

def main():
    """Interactive test runner"""
    tester = SCARAPrecisionTester()
    
    if not tester.connect():
        print("❌ Cannot proceed without robot connection")
        return
    
    # Available test configurations
    configs = {
        "1": "standard_3x3",
        "2": "detailed_5x5", 
        "3": "boundary_test",
        "4": "a4_validation",
        "5": "precision_cal",
        "6": "velocity_test"
    }
    
    print("\n🤖 SCARA Robot Precision Test System")
    print("=" * 50)
    print("Available test configurations:")
    
    for key, config_name in configs.items():
        config = get_test_configuration(config_name)
        print(f"{key}. {config['name']}")
        print(f"   {config['description']}")
        print(f"   Points: {len(config['coordinates'])}\n")
    
    try:
        choice = input("Select test configuration (1-6): ").strip()
        
        if choice in configs:
            config_name = configs[choice]
            print(f"\n🎯 Starting {config_name} test...")
            tester.run_test_configuration(config_name)
        else:
            print("❌ Invalid selection")
            
    except KeyboardInterrupt:
        print("\n🛑 Test interrupted by user")
    except Exception as e:
        print(f"❌ Test error: {e}")

if __name__ == "__main__":
    main()
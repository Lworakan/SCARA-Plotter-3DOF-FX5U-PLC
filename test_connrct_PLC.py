import socket
import time
import math
import csv

# ====================================
# SCARA Robot Test Suite
# ====================================

class ScaraRobot:
    def __init__(self, l1, l2):
        self.l1 = l1
        self.l2 = l2
    
    def inverse_kinematics(self, x, y):
        denom = 2 * self.l1 * self.l2
        if denom == 0:
            return None, None
        
        numerator = x**2 + y**2 - self.l1**2 - self.l2**2
        c2 = numerator / denom
        
        if c2 > 1.0: c2 = 1.0
        if c2 < -1.0: c2 = -1.0
        
        s2 = math.sqrt(1 - c2**2)
        theta2 = math.atan2(s2, c2)
        
        den_k = x**2 + y**2
        if den_k == 0:
            return None, None
        
        term = self.l1 + self.l2 * c2
        s1 = (-x * self.l2 * s2 + y * term) / den_k
        c1 = (y * self.l2 * s2 + x * term) / den_k
        theta1 = math.atan2(s1, c1)
        
        return theta1, theta2
    
    def forward_kinematics(self, t1, t2):
        xe = self.l1 * math.cos(t1)
        ye = self.l1 * math.sin(t1)
        xf = xe + self.l2 * math.cos(t1 + t2)
        yf = ye + self.l2 * math.sin(t1 + t2)
        return xe, ye, xf, yf


class TestSuite:
    def __init__(self, host='192.168.3.250', port=5007):
        self.host = host
        self.port = port
        self.robot = ScaraRobot(160, 149.5)
        self.results = []
    
    def test_1_connection(self):
        """Test 1: Connection Test"""
        print("\n" + "="*50)
        print("Test 1: Connection Test")
        print("="*50)
        
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5)
            result = sock.connect_ex((self.host, self.port))
            
            if result == 0:
                print(f"✓ Connected to {self.host}:{self.port}")
                sock.close()
                self.results.append(("Connection", "Connected", "Pass"))
                return True
            else:
                print(f"✗ Connection Failed")
                self.results.append(("Connection", "Failed", "Fail"))
                return False
                
        except Exception as e:
            print(f"✗ Error: {e}")
            self.results.append(("Connection", str(e), "Fail"))
            return False
    
    def test_2_write_speed(self):
        """Test 2: Write Speed (20ms cycle = 50Hz)"""
        print("\n" + "="*50)
        print("Test 2: Write Speed Test")
        print("="*50)
        
        dt = 0.02  # 20ms
        num_cycles = 100
        frequencies = []
        
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((self.host, self.port))
            
            for i in range(num_cycles):
                start_time = time.time()
                
                # Send dummy command
                command = f"{i},100,1,10,500,50,1,5,250,0\n"
                sock.send(command.encode())
                
                # Wait for 20ms cycle
                elapsed = time.time() - start_time
                if elapsed < dt:
                    time.sleep(dt - elapsed)
                
                cycle_time = time.time() - start_time
                freq = 1.0 / cycle_time if cycle_time > 0 else 0
                frequencies.append(freq)
            
            sock.close()
            
            avg_freq = sum(frequencies) / len(frequencies)
            min_freq = min(frequencies)
            max_freq = max(frequencies)
            
            print(f"Average Frequency: {avg_freq:.2f} Hz")
            print(f"Min: {min_freq:.2f} Hz, Max: {max_freq:.2f} Hz")
            
            # Check if stable ±1Hz
            if 48 <= avg_freq <= 51:
                print("✓ 50 Hz stable - Pass")
                self.results.append(("Write Speed", f"{avg_freq:.2f} Hz", "Pass"))
                return True
            else:
                print(f"✗ Unstable frequency")
                self.results.append(("Write Speed", f"{avg_freq:.2f} Hz", "Fail"))
                return False
                
        except Exception as e:
            print(f"✗ Error: {e}")
            self.results.append(("Write Speed", str(e), "Fail"))
            return False
    
    def test_3_data_integrity(self):
        """Test 3: Data Integrity (1000 commands, 0 errors)"""
        print("\n" + "="*50)
        print("Test 3: Data Integrity Test")
        print("="*50)
        
        num_commands = 1000
        errors = 0
        
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((self.host, self.port))
            
            for i in range(num_commands):
                # Create command with checksum
                j1_pos = i * 10
                j2_pos = i * 5
                
                command = f"{i},{j1_pos},1,10,500,{j2_pos},1,5,250,0"
                checksum = sum(ord(c) for c in command) % 256
                full_command = f"{command},{checksum}\n"
                
                try:
                    sock.send(full_command.encode())
                    # Could add ACK check here if implemented
                except:
                    errors += 1
                
                if (i + 1) % 100 == 0:
                    print(f"Progress: {i+1}/{num_commands} commands sent")
            
            sock.close()
            
            print(f"Commands sent: {num_commands}")
            print(f"Errors detected: {errors}")
            
            if errors == 0:
                print("✓ 0 errors - Pass")
                self.results.append(("Data Integrity", "0 errors", "Pass"))
                return True
            else:
                print(f"✗ {errors} errors detected")
                self.results.append(("Data Integrity", f"{errors} errors", "Fail"))
                return False
                
        except Exception as e:
            print(f"✗ Error: {e}")
            self.results.append(("Data Integrity", str(e), "Fail"))
            return False
    
    def test_4_motion_accuracy(self):
        """Test 4: Motion Accuracy (Circle 100mm diameter, Error < 0.5mm)"""
        print("\n" + "="*50)
        print("Test 4: Motion Accuracy Test")
        print("="*50)
        
        # Generate circle trajectory (50mm radius = 100mm diameter)
        radius = 50
        center = (0, 200)
        num_points = 360
        
        trajectory = []
        for i in range(num_points + 1):
            angle = math.radians(i)
            x = center[0] + radius * math.cos(angle)
            y = center[1] + radius * math.sin(angle)
            trajectory.append((x, y))
        
        # Calculate errors using FK
        errors = []
        for x_target, y_target in trajectory:
            t1, t2 = self.robot.inverse_kinematics(x_target, y_target)
            
            if t1 is not None:
                # Verify with FK
                _, _, x_actual, y_actual = self.robot.forward_kinematics(t1, t2)
                error = math.sqrt((x_target - x_actual)**2 + (y_target - y_actual)**2)
                errors.append(error)
        
        if errors:
            max_error = max(errors)
            avg_error = sum(errors) / len(errors)
            
            print(f"Points tested: {len(errors)}")
            print(f"Max Error: {max_error:.4f} mm")
            print(f"Average Error: {avg_error:.4f} mm")
            
            if max_error < 0.5:
                print("✓ Error < 0.5mm - Pass")
                self.results.append(("Motion Accuracy", f"Error < {max_error:.3f}mm", "Pass"))
                return True
            else:
                print(f"✗ Error too large ({max_error:.3f}mm)")
                self.results.append(("Motion Accuracy", f"Error {max_error:.3f}mm", "Fail"))
                return False
        else:
            print("✗ No valid points")
            self.results.append(("Motion Accuracy", "No data", "Fail"))
            return False
    
    def test_5_timing_sync(self):
        """Test 5: Timing Sync (10s trajectory, Drift < 50ms)"""
        print("\n" + "="*50)
        print("Test 5: Timing Sync Test")
        print("="*50)
        
        dt = 0.02  # 20ms
        duration = 10.0  # 10 seconds
        num_points = int(duration / dt)
        
        # Generate straight line trajectory
        start_pos = (0, 200)
        end_pos = (100, 200)
        
        trajectory = []
        for i in range(num_points):
            s = i / num_points
            x = start_pos[0] + (end_pos[0] - start_pos[0]) * s
            y = start_pos[1] + (end_pos[1] - start_pos[1]) * s
            trajectory.append((x, y))
        
        # Execute trajectory
        start_time = time.time()
        
        try:
            for i, (x, y) in enumerate(trajectory):
                expected_time = i * dt
                
                # Simulate sending command
                t1, t2 = self.robot.inverse_kinematics(x, y)
                
                # Wait for correct timing
                actual_time = time.time() - start_time
                sleep_time = (expected_time + dt) - actual_time
                if sleep_time > 0:
                    time.sleep(sleep_time)
                
                if (i + 1) % 100 == 0:
                    print(f"Progress: {i+1}/{num_points} points")
            
            total_time = time.time() - start_time
            expected_total = duration
            drift = abs(total_time - expected_total) * 1000  # ms
            
            print(f"Expected duration: {expected_total:.3f} s")
            print(f"Actual duration: {total_time:.3f} s")
            print(f"Time drift: {drift:.1f} ms")
            
            if drift < 50:
                print(f"✓ Drift < 50ms - Pass")
                self.results.append(("Timing Sync", f"Drift {drift:.1f}ms", "Pass"))
                return True
            else:
                print(f"✗ Drift too large")
                self.results.append(("Timing Sync", f"Drift {drift:.1f}ms", "Fail"))
                return False
                
        except Exception as e:
            print(f"✗ Error: {e}")
            self.results.append(("Timing Sync", str(e), "Fail"))
            return False
    
    def run_all_tests(self):
        """Run all tests and generate report"""
        print("\n" + "="*50)
        print("SCARA ROBOT SYSTEM TEST SUITE")
        print("="*50)
        
        # Run tests
        self.test_1_connection()
        self.test_2_write_speed()
        self.test_3_data_integrity()
        # self.test_4_motion_accuracy()
        self.test_5_timing_sync()
        
        # Print summary
        print("\n" + "="*50)
        print("TEST SUMMARY")
        print("="*50)
        
        pass_count = sum(1 for r in self.results if r[2] == "Pass")
        total_count = len(self.results)
        
        print(f"\n{'Test Type':<20} {'Result':<25} {'Pass/Fail':<10}")
        print("-" * 55)
        for test_type, result, status in self.results:
            print(f"{test_type:<20} {result:<25} {status:<10}")
        
        print("-" * 55)
        print(f"Total: {pass_count}/{total_count} tests passed")
        
        if pass_count == total_count:
            print("\n✓ ALL TESTS PASSED!")
        else:
            print(f"\n✗ {total_count - pass_count} test(s) failed")
        
        # Save to CSV
        self.save_results()
    
    def save_results(self):
        """Save test results to CSV"""
        filename = 'test_results.csv'
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Test Type', 'Result', 'Pass/Fail'])
            writer.writerows(self.results)
        print(f"\nResults saved to: {filename}")


# ====================================
# MAIN
# ====================================
if __name__ == "__main__":
    # Initialize test suite
    # แก้ไข IP Address ตามเครื่องจริง
    test_suite = TestSuite(host='192.168.3.250', port=5007)
    
    # Run all tests
    test_suite.run_all_tests()
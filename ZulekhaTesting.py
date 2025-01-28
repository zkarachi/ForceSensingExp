#!/usr/bin/env python
import rospy
from geometry_msgs.msg import WrenchStamped
import csv
import os
import datetime
import sys

class FTSensor:
    def __init__(self):
        self.calibrating = True
        self.ready_to_record = False
        self.calibration_count = 0
        self.force_offset = {'x': 0, 'y': 0, 'z': 0}
        self.torque_offset = {'x': 0, 'y': 0, 'z': 0}
        rospy.loginfo("\nStarting calibration. Please do not apply any force...")
        
    def calibrate(self, force, torque):
        if self.calibration_count < 50:
            rospy.loginfo(f"Calibrating... {self.calibration_count}/50")
            self.force_offset['x'] += force.x / 50
            self.force_offset['y'] += force.y / 50
            self.force_offset['z'] += force.z / 50
            self.torque_offset['x'] += torque.x / 50
            self.torque_offset['y'] += torque.y / 50
            self.torque_offset['z'] += torque.z / 50
            self.calibration_count += 1
            if self.calibration_count == 50:
                self.calibrating = False
                rospy.loginfo("\nCalibration complete! Press Enter to start recording data...")
                sys.stdin.readline()
                rospy.loginfo("Starting to record data...")
                self.ready_to_record = True

def ft_sensor_callback(data):
    if ft_sensor.calibrating:
        ft_sensor.calibrate(data.wrench.force, data.wrench.torque)
        return

    if not ft_sensor.ready_to_record:
        return

    force = data.wrench.force
    torque = data.wrench.torque
    
    force_x = force.x - ft_sensor.force_offset['x']
    force_y = force.y - ft_sensor.force_offset['y'] 
    force_z = force.z - ft_sensor.force_offset['z']
    torque_x = torque.x - ft_sensor.torque_offset['x']
    torque_y = torque.y - ft_sensor.torque_offset['y']
    torque_z = torque.z - ft_sensor.torque_offset['z']

    timestamp = datetime.datetime.fromtimestamp(rospy.get_time()).strftime('%Y-%m-%d %H:%M:%S.%f')
    rospy.loginfo(f"Force: x={force_x:.2f}, y={force_y:.2f}, z={force_z:.2f}")
    rospy.loginfo(f"Torque: x={torque_x:.2f}, y={torque_y:.2f}, z={torque_z:.2f}")

    with open(csv_file, 'a') as file:
        writer = csv.writer(file)
        writer.writerow([timestamp, force_x, force_y, force_z, torque_x, torque_y, torque_z])

def main():
    global csv_file, ft_sensor
    rospy.init_node('ft_sensor_subscriber', anonymous=True)
    
    ft_sensor = FTSensor()
    script_dir = os.path.dirname(os.path.abspath(__file__))
    csv_file = os.path.join(script_dir, "ft_sensor_datatest2.csv")
    
    with open(csv_file, 'w') as file:
        writer = csv.writer(file)
        writer.writerow(["Timestamp", "Force_x", "Force_y", "Force_z", "Torque_x", "Torque_y", "Torque_z"])
        
    rospy.Subscriber('NordboLRS6_node/wrench', WrenchStamped, ft_sensor_callback)
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
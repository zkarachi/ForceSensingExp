#!/usr/bin/env python
import rospy
from geometry_msgs.msg import WrenchStamped
import csv
import os
import datetime
import sys

class FTSensor:
   def __init__(self):
       self.last_time = rospy.get_time()
       self.collecting = False
       print("Press Enter to start collecting data...")
       sys.stdin.readline()
       self.collecting = True
       print("Data collection started. Press Enter to stop...")

def ft_sensor_callback(data):
   if not ft_sensor.collecting:
       return

   current_time = rospy.get_time()
   if current_time - ft_sensor.last_time >= 2.0:
       force = data.wrench.force
       torque = data.wrench.torque
       timestamp = datetime.datetime.fromtimestamp(current_time).strftime('%Y-%m-%d %H:%M:%S.%f')
       
       rospy.loginfo(f"Force: x={force.x:.3f}, y={force.y:.3f}, z={force.z:.3f}")
       rospy.loginfo(f"Torque: x={torque.x:.3f}, y={torque.y:.3f}, z={torque.z:.3f}")
       
       with open(csv_file, 'a') as file:
           writer = csv.writer(file)
           writer.writerow([timestamp, 
                          round(force.x, 3), 
                          round(force.y, 3), 
                          round(force.z, 3),
                          round(torque.x, 3), 
                          round(torque.y, 3), 
                          round(torque.z, 3)])
       
       ft_sensor.last_time = current_time

def main():
   global csv_file, ft_sensor
   rospy.init_node('ft_sensor_subscriber', anonymous=True)
   ft_sensor = FTSensor()
   
   script_dir = os.path.dirname(os.path.abspath(__file__))
   csv_file = os.path.join(script_dir, "ft_sensor_datatest_angle50_t2.csv")
   
   with open(csv_file, 'w') as file:
       writer = csv.writer(file)
       writer.writerow(["Timestamp", "Force_x", "Force_y", "Force_z", "Torque_x", "Torque_y", "Torque_z"])
   
   rospy.Subscriber('NordboLRS6_node/wrench', WrenchStamped, ft_sensor_callback)
   sys.stdin.readline()
   ft_sensor.collecting = False
   print("Data collection stopped.")
   rospy.signal_shutdown("Data collection complete")

if __name__ == "__main__":
   try:
       main()
   except rospy.ROSInterruptException:
       pass
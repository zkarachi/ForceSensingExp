
# import rospy
# from geometry_msgs.msg import WrenchStamped
# import csv
# import os
# import datetime

# def ft_sensor_callback(data):
#    force = data.wrench.force
#    torque = data.wrench.torque
#    timestamp = datetime.datetime.fromtimestamp(rospy.get_time()).strftime('%Y-%m-%d %H:%M:%S.%f')
   
#    rospy.loginfo(f"Timestamp: {timestamp}")
#    rospy.loginfo(f"Force: x={force.x:.3f}, y={force.y:.3f}, z={force.z:.3f}")
#    rospy.loginfo(f"Torque: x={torque.x:.3f}, y={torque.y:.3f}, z={torque.z:.3f}")
   
#    with open(csv_file, 'a') as file:
#        writer = csv.writer(file)
#        writer.writerow([timestamp, 
#                        round(force.x, 3), 
#                        round(force.y, 3), 
#                        round(force.z, 3),
#                        round(torque.x, 3), 
#                        round(torque.y, 3), 
#                        round(torque.z, 3)])

# def main():
#    rospy.init_node('ft_sensor_subscriber', anonymous=True)
#    global csv_file
#    script_dir = os.path.dirname(os.path.abspath(__file__))
#    csv_file = os.path.join(script_dir, "ft_sensor_datatest1.csv")
   
#    with open(csv_file, 'w') as file:
#        writer = csv.writer(file)
#        writer.writerow(["Timestamp", "Force_x", "Force_y", "Force_z", "Torque_x", "Torque_y", "Torque_z"])
   
#    rospy.Subscriber('NordboLRS6_node/wrench', WrenchStamped, ft_sensor_callback)
#    rospy.spin()

# if __name__ == "__main__":
#    try:
#        main()
#    except rospy.ROSInterruptException:
#        pass
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import WrenchStamped
import csv
import os
import datetime

class FTSensor:
   def __init__(self):
       self.last_time = rospy.get_time()

def ft_sensor_callback(data):
   current_time = rospy.get_time()
   if current_time - ft_sensor.last_time >= 5.0:  # Check if 5 seconds have passed
       force = data.wrench.force
       torque = data.wrench.torque
       timestamp = datetime.datetime.fromtimestamp(current_time).strftime('%Y-%m-%d %H:%M:%S.%f')
       
       rospy.loginfo(f"Timestamp: {timestamp}")
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
   csv_file = os.path.join(script_dir, "ft_sensor_datatest1.csv")
   
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
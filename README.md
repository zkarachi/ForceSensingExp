# ForceSensingExp

Turning on FT sensor
- Grab the battery pack a plug the sensor in and turn the battery pack on
- Also check if the sensor is connected via ethernet
- Go to the home directory we are now gonna source our enviroment:
    - Home/ZilinSi/Delta-Sensor/ros_ws
    - enter the command: source devel/setup.bash
    - Now your enviroment has been sourced
    - then launch the node by:
        - cd into Home/ZilinSi/Delta-Sensor/ros_ws/src/nrs_node/launch
        - enter the command: roslaunch nordbolrs6.launch
        - you have now launched the FT sensor node, it should say your sensor has been activated.
        - Copy the IP address into a browser and check you are getting sensor readings
        - Errors:
          - If the sensor is not activated here are a few steps to take:
              - Check the ethernet is connected, sometimes plugging it back in can work
              - Check you hav clicked on the correct ethernet connection on the computer
              - Turn the sensor on and off
    - If the sensor is activated DO NOT exit out of that terminal you need to keep that running
    - Open a NEW terminal whenever you want to run a script and source that terminal everytime
  
           


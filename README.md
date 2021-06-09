- sensor in the body of the extinguisher (3-axis)

-> I'd like to receive Boolean events for this. If you can determine the position of the extinguisher given some (x) value, if it passes that value and the extinguisher is now "Horizontal", send me an "orientationInvalid" event. If the extinguisher passes (x) value again and is now "Vertical", send me an "orientationValid" event. 

- I don't care to know the actual axis values

- This data will only be sent while the handle is pressed down.

- sensor in the hose (IMU)

- When the handle is pressed, consider the tip of the hose at origin (0,0,0). I want to receive the relative (x,y,z) values to the origin and tilt (yaw, pitch, roll) as the hose is moving around. I will mimic these values from the Magic Leap headset to make the spray look like it is following the same planes.

- This data will only be sent while the handle is pressed down.
for the pin pull, I want to receive the "pull" and "inserted" events all the time, as long as the device is on and connected to the server. 


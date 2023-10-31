# JIEUN's CODE

## Achievements with the code
1. Backup Code
  - Extracts colour successfully and computes the centre
  - Use releative position of the centre to image pixel size to rotate the robot accordingly
  - Implemented proportional velocity control (pixel distance between centre of marker and centre of image)
  - This was created earlier in the project before exploring more advanced methods such as Visual Servoing, PID Controllers, and exploring other feature extraction methods.

2. Controller Class
  - Contains two methods of Controllers
  - User can set "PID" or "IBVS" mode to execute two different approaches of velocity control of the robot
  - This class was used for the fully integrated code in [TurtleBot Code/Integrated Code](TurtleBot%20Code/Integrated%20Code)

3. FeatureExtraction Class
  - As another member was the main contributor to exploring future extraction, a more simple method was applied here for it to be used for controller class development.
  - This class was not used for the fully integrated code
  - This class is still useful and would be the backup class for feature extraction.

4. PID Tuning
  - Scripts for PID Tuning was created for each Kp, Ki, Kd tests.
  - Error was plotted over time, observing K damping effects. 

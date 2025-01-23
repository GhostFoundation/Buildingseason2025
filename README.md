# Buildingseason2025
Ghost foundation Buildingseason repo of 2025

# table of contents
1. [Info about flowcharts](#info-about-flowcharts)
2. [Change Log format](#change-log-format)
3. [Change Log](#change-log)


# info about flowcharts
Please use these symbols with their meaning:
![flowchartRules](https://github.com/user-attachments/assets/d9ad3125-5951-4d97-b8a5-b935cfcd5127)

If you finished your flowchart please add it to this folder: [Flowcharts](https://github.com/GhostFoundation/Buildingseason2025/tree/main/flowcharts)


### Change Log format:
DD-MM-YYYY

Responsible: [Name of the Person]

- Task: Brief description of the task performed.
- Details: Additional context if required.
- Info: Anything someone else might need to think about when continuing the work (optional).

# Change Log

14-01-2025 

Responsible: Gijs van Maanen 
- Task: Create basic drivecode with gyro
- Details: swervedrive works, fieldorientated driving (gyro) works
- Info: max velocity is set at 3m/s for training purposes real max speed is 7.22 m/s


18-01-2025

Responsible: Gijs van Maanen
- Taks: Implement apriltag detection with the orange pi (photonvision)
- Details: implemented apriltag detection in branch: [apriltag-vision-by-Gijs](https://github.com/GhostFoundation/Buildingseason2025/tree/apriltag-vision-by-Gijs) decided to not do algea detection since it requires a third camera and we can only get the following info: where is the algea in 2d and how many algea are visible. This info doesnt seem too important, will be discussed with others.
- Info: Still has to be tested.

21-01-2025

Responsible: Gijs van Maanen

- Task: Merged apriltag branch with main
- Details: Tested apriltag vision, it works so i merged it with main

23-01-2025

Responsible: Gijs van Maanen

- Task: Make sure the robot can run Pathplanner autonomous commands and tune the PID of rotation and translation.
- Details: Paths can now be made in pathplanner and selected on the smartdashboard. PID is tuned.
- Info: Autonomous paths where the robot is driving while simultaniously turning will not work for some reason. The robot does turn but forgets where the robot is supposed to end. I will look into this.


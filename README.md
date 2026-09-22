# Autonomous-Trash-Drone-Proj
This is a project me and a friend are working on! Pretty generic idea but teaches so much and is so complex when you really start from scratch. Not everything is finalized yet so many updates will be made until we finish the project and it works as intended

A little about what it will do
- General idea is that it is a drone that can autonomously find, pick up, and throw away trash by itself with no human interference.
- It also has a manual mode but that isn't the main focus to the drone.
- It contains a custom SLAM system that acts like a Roomba but in the air.
- Custom trained trash computer vision but used photos from existing models.
- Connects to phone wirelessly hopefully approx. 50 meters
- More coming as we continue to update the drone!

Why we made it?
- We thought it would be a simple first project together...
- Turns out it was way more complicated than we thought but we put in the hours and got to the point we're at now!
- It's also just very cool to be able to think of something and then know how to make it.

# Features
- Custom SLAM (lidar-based, built from scratch) for GPS-free localization and mapping
- Autonomous coverage planning — systematically explores and covers an area like a Roomba
- Loop closure + drift correction for reliable long-flight accuracy
- Computer vision: detects trash, trash cans, and water hazards via a custom-trained model
- Two-jointed robotic arm + gripper for autonomous pickup and bin delivery
- Fully autonomous flight control (arm, takeoff, navigation, landing) with battery-based auto-return
- Phone-based live dashboard: camera feed, telemetry, manual override, emergency RTL — no app needed
- Runs entirely on a Raspberry Pi 5, no internet or cloud required

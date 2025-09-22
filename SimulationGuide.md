//initial draft created via Gemini2.5
Need to edit to better match what changes we did
NOTE I also want to include "first time setup" steps for advantagescope, so anyone can read and follow along.
Tada, this is essentially what you did with me when i was trying to figure it out :D

# Guide to Epilogue and Simulation in FRC Code

This document explains the key components your fellow student added to the robot code to enable data logging with **AdvantageScope** and to set up a basic **robot simulation**.

---

## 1. Data Logging with Epilogue

The code uses WPILib's **Epilogue** library to automatically record data from the robot during a match or practice session. This data is saved to a file (`.wpilog`), which you can then open and analyze with the **AdvantageScope** software. 

### How it Works
The system is mostly set up in two places: `Robot.java` and `Drive.java`.
The other areas are just as important, but the best examples are here!

### In `Robot.java`
1.  **Enabling Logging**: The `@Logged` annotation is added to the `Robot` class. This tells the Epilogue library to start looking for data to log. The `name` parameter (`@Logged(name = "...")`) is just a label that shows up in the log file, making it easier to identify.
2.  **Starting the Log**: In the `Robot()` constructor, two lines of code are crucial:
    * `DataLogManager.start();` starts the process of writing data to a file.
    * `Epilogue.bind(this);` connects the Epilogue library to the `Robot` class, which is the starting point for all the logging.

### In `Drive.java`
The `Drive` class is where the more specific logging is configured.

* **Class-level Logging**: The `@Logged` annotation is placed on the `Drive` class itself. This is a shortcut that tells Epilogue to automatically log all the public variables within the `Drive` class, such as the motor controllers (`leftLeader`, `rightLeader`) and the sensors (`leftEncoder`, `rightEncoder`, `gyro`). This saves you from having to add a `@Logged` annotation to each one individually.
* **Selective Logging**: The `@NotLogged` annotation is used on variables that don't need to be logged, such as the `DifferentialDrive` object and all the simulation components. This is a good practice because it keeps the log files smaller and avoids recording redundant data.
* **Method-level Logging**: The `getPose()` method has a `@Logged` annotation. This tells Epilogue to log the value returned by this method every time it is called. This is how the robot's position and orientation data (`Pose2d`) gets recorded and shows up in AdvantageScope.

---

## 2. Drivetrain Simulation

The `Drive` subsystem includes code for running a **virtual simulation** of the robot's drivetrain. This allows you to test code without a physical robot connected.

### Key Components
The simulation uses these components:
* **`DifferentialDrivetrainSim`**: This is the core physics model of the robot. It uses physical parameters like mass, gear ratio, and wheel size to calculate how the robot would move in a simulated environment based on the motor inputs. It's marked with `@NotLogged` because we don't need to log the simulation model itself.
* **Simulated Sensors**: The `ADXRS450_GyroSim` and `EncoderSim` objects are "fake" sensors that get their data from the `DifferentialDrivetrainSim` model. This allows the robot code to run as if there were real sensors connected. They are also marked with `@NotLogged`.

### How it Works
The `simulationPeriodic()` method is called automatically by WPILib during simulation mode. Inside this method:
1.  **Inputs are Set**: The simulated drivetrain (`sim`) is given the motor power values.
2.  **Physics are Updated**: The `sim.update(.02)` command runs the physics model for a 20-millisecond step, simulating one robot loop.
3.  **Simulated Sensors are Updated**: The simulated encoders and gyro are updated with the new position and angle data from the physics model. The rest of the code then uses these simulated sensor values just as it would use real sensor data.
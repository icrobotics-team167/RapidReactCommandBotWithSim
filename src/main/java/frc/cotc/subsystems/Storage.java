// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.subsystems;

// We use these to include the logging library for AdvantageScope
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.cotc.Constants.StorageConstants;

/**
 * The storage subsystem for the robot, which includes a motor to move balls into the shooter and a
 * sensor to detect whether a ball is present. 
 * 
 * We added the @Logged annotation to the class. 
 * This tells the Epilogue logging library to automatically log all public variables and methods in this class.
 */
@Logged
public class Storage extends SubsystemBase {
  private final PWMSparkMax motor = new PWMSparkMax(StorageConstants.MOTOR_PORT);
  @NotLogged // We'll log a more meaningful boolean instead
  private final DigitalInput ballSensor = new DigitalInput(StorageConstants.BALL_SENSOR_PORT);

  // Expose trigger from subsystem to improve readability and ease
  // inter-subsystem communications
  /** Whether the ball storage is full. */
  @Logged(name = "Has Cargo")
  @SuppressWarnings("checkstyle:MemberName")
  public final Trigger hasCargo = new Trigger(ballSensor::get);

  /** Create a new Storage subsystem. */
  public Storage() {
    // Set default command to turn off the storage motor and then idle
    setDefaultCommand(runOnce(motor::disable).andThen(run(() -> {})).withName("Idle"));
  }

  /** Returns a command that runs the storage motor indefinitely. */
  public Command runCommand() {
    return run(() -> motor.set(1)).withName("run");
  }
}

// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.subsystems;

import static frc.cotc.Constants.IntakeConstants;
// We use these to include the logging library for AdvantageScope
import edu.wpi.first.epilogue.Logged;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The Intake subsystem controls the robot's intake mechanism, which is used to pick up and hold
 * game pieces. It contains methods for deploying and retracting the intake, as well as running the
 * intake motor.
 * We added the @Logged annotation to the class.
 * This tells the Epilogue logging library to automatically log all public variables and methods in this class.
 */
@Logged
public class Intake extends SubsystemBase {
  private final PWMSparkMax motor = new PWMSparkMax(IntakeConstants.MOTOR_PORT);

  // Double solenoid connected to two channels of a PCM with the default CAN ID
  private final DoubleSolenoid pistons =
      new DoubleSolenoid(
          PneumaticsModuleType.CTREPCM,
          IntakeConstants.SOLENOID_PORTS[0],
          IntakeConstants.SOLENOID_PORTS[1]);

  /** Returns a command that deploys the intake, and then runs the intake motor indefinitely. */
  public Command intakeCommand() {
    return runOnce(() -> pistons.set(DoubleSolenoid.Value.kForward))
        .andThen(run(() -> motor.set(1.0)))
        .withName("Intake");
  }

  /** Returns a command that turns off and retracts the intake. */
  public Command retractCommand() {
    return runOnce(
            () -> {
              motor.disable();
              pistons.set(DoubleSolenoid.Value.kReverse);
            })
        .withName("Retract");
  }
}

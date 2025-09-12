// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.cotc.Constants.AutoConstants;
import frc.cotc.Constants.OIConstants;
import frc.cotc.Constants.ShooterConstants;
import frc.cotc.subsystems.Drive;
import frc.cotc.subsystems.Intake;
import frc.cotc.subsystems.Pneumatics;
import frc.cotc.subsystems.Shooter;
import frc.cotc.subsystems.Storage;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
@Logged(name = "Rapid React Command Robot Container")
public class RapidReactCommandBot {
  // The robot's subsystems
  private final Drive drive = new Drive();
  private final Intake intake = new Intake();
  private final Storage storage = new Storage();
  private final Shooter shooter = new Shooter();
  private final Pneumatics pneumatics = new Pneumatics();

  // The driver's controller
  CommandXboxController driverController =
      new CommandXboxController(OIConstants.DRIVER_CONTROLLER_PORT);

  /**
   * Use this method to define bindings between conditions and commands. These are useful for
   * automating robot behaviors based on button and sensor input.
   *
   * <p>Should be called in the robot class constructor.
   *
   * <p>Event binding methods are available on the {@link Trigger} class.
   */
  public void configureBindings() {
    // Automatically run the storage motor whenever the ball storage is not full,
    // and turn it off whenever it fills. Uses subsystem-hosted trigger to
    // improve readability and make inter-subsystem communication easier.
    storage.hasCargo.whileFalse(storage.runCommand());

    // Automatically disable and retract the intake whenever the ball storage is full.
    storage.hasCargo.onTrue(intake.retractCommand());

    // Control the drive with split-stick arcade controls
    drive.setDefaultCommand(
        drive.arcadeDriveCommand(
            () -> -driverController.getLeftY(), () -> -driverController.getRightX()));

    // Deploy the intake with the X button
    driverController.x().onTrue(intake.intakeCommand());
    // Retract the intake with the Y button
    driverController.y().onTrue(intake.retractCommand());

    // Fire the shooter with the A button
    driverController
        .a()
        .onTrue(
            parallel(
                    shooter.shootCommand(ShooterConstants.SHOOTER_TARGET_RPS), storage.runCommand())
                // Since we composed this inline we should give it a name
                .withName("Shoot"));

    // Toggle compressor with the Start button
    driverController.start().toggleOnTrue(pneumatics.disableCompressorCommand());
  }

  /**
   * Use this to define the command that runs during autonomous.
   *
   * <p>Scheduled during {@link Robot#autonomousInit()}.
   */
  public Command getAutonomousCommand() {
    // Drive forward for 2 meters at half speed with a 3 second timeout
    return drive
        .driveDistanceCommand(AutoConstants.DRIVE_DISTANCE_METERS, AutoConstants.DRIVE_SPEED)
        .withTimeout(AutoConstants.TIMEOUT_SECONDS);
  }
}

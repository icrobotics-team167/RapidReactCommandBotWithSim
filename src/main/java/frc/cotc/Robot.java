// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc; // package declaration specifies the package name for this class, which is

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.cotc.subsystems.*;
import java.util.Set;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 *
 * <p>Note: Here we added the @Logged annotation to the class. This tells the Epilogue logging
 * library to automatically log all public variables and methods in this Robot class.
 */
@Logged(name = "Rapid React Command Robot")
public class Robot extends TimedRobot {
  public final Drive drive = new Drive();
  public final Intake intake = new Intake();
  public final Storage storage = new Storage();
  public final Shooter shooter = new Shooter();
  public final Pneumatics pneumatics = new Pneumatics();

  private final SendableChooser<Command> autoChooser;

  /**
   * This method is run when the robot is first started up and should be used for any initialization
   * code.
   */
  public Robot() {
    // The driver's controller
    CommandXboxController driverController =
        new CommandXboxController(Constants.OIConstants.DRIVER_CONTROLLER_PORT);

    // Automatically run the storage motor whenever the ball storage is not full,
    // and turn it off whenever it fills. Uses subsystem-hosted trigger to
    // improve readability and make inter-subsystem communication easier.
    storage.hasCargo.whileFalse(storage.runCommand());

    // Automatically disable and retract the intake whenever the ball storage is full.
    storage.hasCargo.onTrue(intake.retractCommand());

    // Control the drive with split-stick arcade controls on an Xbox controller
    drive.setDefaultCommand(
        drive.arcadeDriveCommand(
            () -> -driverController.getLeftY(), () -> -driverController.getRightX()));

    // Deploy the intake with the X button
    driverController.x().onTrue(intake.intakeCommand());
    // Retract the intake with the Y button
    driverController.y().onTrue(intake.retractCommand());

    // Fire the shooter with the A button on an Xbox controller
    driverController
        .a()
        .onTrue(
            parallel(
                    shooter.shootCommand(Constants.ShooterConstants.SHOOTER_TARGET_RPS),
                    storage.runCommand())
                // Since we composed this inline we should give it a name
                .withName("Shoot"));

    // Toggle compressor with the Start button
    driverController.start().toggleOnTrue(pneumatics.disableCompressorCommand());

    // Initialize data logging.
    DataLogManager.start();
    // Connect the Epilogue logger to the current Robot Instance
    // This is saved in a .wpilog file on the robot, and can be opened in AdvantageScope
    Epilogue.bind(this);

    autoChooser = new SendableChooser<>();
    autoChooser.addOption("None", none());
    autoChooser.addOption("DriveForward", drive.driveDistanceCommand(4, 1));
    SmartDashboard.putData(autoChooser);

    RobotModeTriggers.autonomous().whileTrue(deferredProxy(autoChooser::getSelected));
  }

  /**
   * This method is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic methods, but before LiveWindow and SmartDashboard
   * integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }
}

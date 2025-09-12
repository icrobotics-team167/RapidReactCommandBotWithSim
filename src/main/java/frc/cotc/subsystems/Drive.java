// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.cotc.Constants.DriveConstants;
import java.util.function.DoubleSupplier;

@Logged
public class Drive extends SubsystemBase {
  // The motors on the left side of the drive.
  private final PWMSparkMax leftLeader = new PWMSparkMax(DriveConstants.LEFT_MOTOR_1_PORT);
  private final PWMSparkMax leftFollower = new PWMSparkMax(DriveConstants.LEFT_MOTOR_2_PORT);

  // The motors on the right side of the drive.
  private final PWMSparkMax rightLeader = new PWMSparkMax(DriveConstants.RIGHT_MOTOR_1_PORT);
  private final PWMSparkMax rightFollower = new PWMSparkMax(DriveConstants.RIGHT_MOTOR_2_PORT);

  // The robot's drive
  @NotLogged // Would duplicate motor data, there's no point sending it twice
  private final DifferentialDrive drive = new DifferentialDrive(leftLeader::set, rightLeader::set);

  // The left-side drive encoder
  private final Encoder leftEncoder =
      new Encoder(
          DriveConstants.LEFT_ENCODER_PORTS[0],
          DriveConstants.LEFT_ENCODER_PORTS[1],
          DriveConstants.LEFT_ENCODER_REVERSED);

  @NotLogged private final EncoderSim leftEncoderSim = new EncoderSim(leftEncoder);

  // The right-side drive encoder
  private final Encoder rightEncoder =
      new Encoder(
          DriveConstants.RIGHT_ENCODER_PORTS[0],
          DriveConstants.RIGHT_ENCODER_PORTS[1],
          DriveConstants.RIGHT_ENCODER_REVERSED);

  @NotLogged private final EncoderSim rightEncoderSim = new EncoderSim(rightEncoder);

  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();
  @NotLogged private final ADXRS450_GyroSim gyroSim = new ADXRS450_GyroSim(gyro);
  private final ProfiledPIDController controller =
      new ProfiledPIDController(
          DriveConstants.TURN_P,
          DriveConstants.TURN_I,
          DriveConstants.TURN_D,
          new TrapezoidProfile.Constraints(
              DriveConstants.MAX_TURN_RATE_DEG_PER_S,
              DriveConstants.MAX_TURN_ACCELERATION_DEG_PER_S_SQUARED));
  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          DriveConstants.ksVolts,
          DriveConstants.kvVoltSecondsPerDegree,
          DriveConstants.kaVoltSecondsSquaredPerDegree);

  private final DifferentialDriveOdometry odometry =
      new DifferentialDriveOdometry(new Rotation2d(), 0, 0, new Pose2d());

  /** Creates a new Drive subsystem. */
  public Drive() {
    SendableRegistry.addChild(drive, leftLeader);
    SendableRegistry.addChild(drive, rightLeader);

    leftLeader.addFollower(leftFollower);
    rightLeader.addFollower(rightFollower);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    rightLeader.setInverted(true);

    // Sets the distance per pulse for the encoders
    leftEncoder.setDistancePerPulse(DriveConstants.ENCODER_DISTANCE_PER_PULSE);
    rightEncoder.setDistancePerPulse(DriveConstants.ENCODER_DISTANCE_PER_PULSE);

    // Set the controller to be continuous (because it is an angle controller)
    controller.enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    controller.setTolerance(
        DriveConstants.TURN_TOLERANCE_DEG, DriveConstants.TURN_RATE_TOLERANCE_DEG_PER_S);
  }

  @NotLogged
  private final DifferentialDrivetrainSim sim =
      new DifferentialDrivetrainSim(
          DCMotor.getNEO(4), 6, 10, Units.lbsToKilograms(150), Units.inchesToMeters(3), 1, null);

  @Override
  public void periodic() {
    odometry.update(
        gyro.getRotation2d(),
        new DifferentialDriveWheelPositions(leftEncoder.getDistance(), rightEncoder.getDistance()));
  }

  @Logged
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  @Override
  public void simulationPeriodic() {
    sim.setInputs(leftLeader.get() * 12, rightLeader.get() * 12);
    sim.update(.02);
    leftEncoderSim.setCount(
        (int) (sim.getLeftPositionMeters() / DriveConstants.ENCODER_DISTANCE_PER_PULSE));
    rightEncoderSim.setCount(
        (int) (sim.getRightPositionMeters() / DriveConstants.ENCODER_DISTANCE_PER_PULSE));
    gyroSim.setAngle(-sim.getHeading().getDegrees());
  }

  /**
   * Returns a command that drives the robot with arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public Command arcadeDriveCommand(DoubleSupplier fwd, DoubleSupplier rot) {
    // A split-stick arcade command, with forward/backward controlled by the left
    // hand, and turning controlled by the right.
    return run(() -> drive.arcadeDrive(fwd.getAsDouble(), rot.getAsDouble()))
        .withName("arcadeDrive");
  }

  /**
   * Returns a command that drives the robot forward a specified distance at a specified speed.
   *
   * @param distanceMeters The distance to drive forward in meters
   * @param speed The fraction of max speed at which to drive
   */
  public Command driveDistanceCommand(double distanceMeters, double speed) {
    return runOnce(
            () -> {
              // Reset encoders at the start of the command
              leftEncoder.reset();
              rightEncoder.reset();
            })
        // Drive forward at specified speed
        .andThen(run(() -> drive.arcadeDrive(speed, 0)))
        // End command when we've traveled the specified distance
        .until(
            () -> Math.max(leftEncoder.getDistance(), rightEncoder.getDistance()) >= distanceMeters)
        // Stop the drive when the command ends
        .finallyDo(interrupted -> drive.stopMotor());
  }

  /**
   * Returns a command that turns to robot to the specified angle using a motion profile and PID
   * controller.
   *
   * @param angleDeg The angle to turn to
   */
  public Command turnToAngleCommand(double angleDeg) {
    return startRun(
            () -> controller.reset(gyro.getRotation2d().getDegrees()),
            () ->
                drive.arcadeDrive(
                    0,
                    controller.calculate(gyro.getRotation2d().getDegrees(), angleDeg)
                        // Divide feedforward voltage by battery voltage to normalize it to [-1, 1]
                        + feedforward.calculate(controller.getSetpoint().velocity)
                            / RobotController.getBatteryVoltage()))
        .until(controller::atGoal)
        .finallyDo(() -> drive.arcadeDrive(0, 0));
  }
}

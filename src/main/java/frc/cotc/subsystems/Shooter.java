// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
//We use these to include the logging library for AdvantageScope
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
//use these for simulating sensor data
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.cotc.Constants.ShooterConstants;

/**
 * The shooter subsystem for the robot, which includes a flywheel shooter and a feeder motor. We
 * added the @Logged annotation to the class. This tells the Epilogue logging library to
 * automatically log all public variables and methods in this class.
 */
@Logged
public class Shooter extends SubsystemBase {
  private final PWMSparkMax shooterMotor = new PWMSparkMax(ShooterConstants.SHOOTER_MOTOR_PORT);
  private final PWMSparkMax feederMotor = new PWMSparkMax(ShooterConstants.FEEDER_MOTOR_PORT);
  private final Encoder shooterEncoder =
      new Encoder(
          ShooterConstants.ENCODER_PORTS[0],
          ShooterConstants.ENCODER_PORTS[1],
          ShooterConstants.ENCODER_REVERSED);
  private final SimpleMotorFeedforward shooterFeedforward =
      new SimpleMotorFeedforward(
          ShooterConstants.S_VOLTS, ShooterConstants.V_VOLT_SECONDS_PER_ROTATION);
  private final PIDController shooterFeedback = new PIDController(ShooterConstants.P, 0.0, 0.0);

  @NotLogged
  private final DCMotorSim shooterSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getNEO(2), 1, 1), DCMotor.getNEO(2));

  @NotLogged private final EncoderSim encoderSim = new EncoderSim(shooterEncoder);

  /** The shooter subsystem for the robot. */
  public Shooter() {
    shooterFeedback.setTolerance(ShooterConstants.SHOOTER_TOLERANCE_RPS);
    shooterEncoder.setDistancePerPulse(ShooterConstants.ENCODER_DISTANCE_PER_PULSE);

    // Set default command to turn off both the shooter and feeder motors, and then idle
    setDefaultCommand(
        runOnce(
                () -> {
                  shooterMotor.disable();
                  feederMotor.disable();
                })
            .andThen(run(() -> {}))
            .withName("Idle"));
  }
  /*
   * This method will be called once per scheduler run during simulation.
   * It updates our simulated sensor values for the simulation environment.
   */
  @Override
  public void simulationPeriodic() {
    shooterSim.setInputVoltage(shooterMotor.get() * 12);
    shooterSim.update(.02);
    encoderSim.setCount(
        (int) (shooterSim.getAngularPositionRotations() * ShooterConstants.ENCODER_CPR));
  }

  /**
   * Returns a command to shoot the balls currently stored in the robot. Spins the shooter flywheel
   * up to the specified setpoint, and then runs the feeder motor.
   *
   * @param setpointRotationsPerSecond The desired shooter velocity
   */
  public Command shootCommand(double setpointRotationsPerSecond) {
    return parallel(
            // Run the shooter flywheel at the desired setpoint using feedforward and feedback
            run(
                () -> {
                  shooterMotor.set(
                      shooterFeedforward.calculate(setpointRotationsPerSecond)
                          + shooterFeedback.calculate(
                              shooterEncoder.getRate(), setpointRotationsPerSecond));
                }),

            // Wait until the shooter has reached the setpoint, and then run the feeder
            waitUntil(shooterFeedback::atSetpoint).andThen(() -> feederMotor.set(1)))
        .withName("Shoot");
  }
}

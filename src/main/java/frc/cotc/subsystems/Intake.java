// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.cotc.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.cotc.Constants.IntakeConstants;



@Logged
public class Intake extends SubsystemBase
{
    private final PWMSparkMax motor = new PWMSparkMax(IntakeConstants.MOTOR_PORT);
    
    // Double solenoid connected to two channels of a PCM with the default CAN ID
    private final DoubleSolenoid pistons =
            new DoubleSolenoid(
                    PneumaticsModuleType.CTREPCM,
                    IntakeConstants.SOLENOID_PORTS[0],
                    IntakeConstants.SOLENOID_PORTS[1]);
    
    
    /** Returns a command that deploys the intake, and then runs the intake motor indefinitely. */
    public Command intakeCommand()
    {
        return runOnce(() -> pistons.set(DoubleSolenoid.Value.kForward))
                .andThen(run(() -> motor.set(1.0)))
                .withName("Intake");
    }
    
    
    /** Returns a command that turns off and retracts the intake. */
    public Command retractCommand()
    {
        return runOnce(
                () -> {
                    motor.disable();
                    pistons.set(DoubleSolenoid.Value.kReverse);
                })
                .withName("Retract");
    }
}

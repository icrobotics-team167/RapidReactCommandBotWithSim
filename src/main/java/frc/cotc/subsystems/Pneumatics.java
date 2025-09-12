// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.cotc.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



/** Subsystem for managing the compressor, pressure sensor, etc. */
@Logged
public class Pneumatics extends SubsystemBase
{
    // External analog pressure sensor
    // product-specific voltage->pressure conversion, see product manual
    // in this case, 250(V/5)-25
    // the scale parameter in the AnalogPotentiometer constructor is scaled from 1 instead of 5,
    // so if r is the raw AnalogPotentiometer output, the pressure is 250r-25
    static final double SCALE = 250;
    static final double OFFSET = -25;
    private final AnalogPotentiometer pressureTransducer =
            new AnalogPotentiometer(/* the AnalogIn port*/ 2, SCALE, OFFSET);
    
    // Compressor connected to a PCM with a default CAN ID (0)
    private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    
    
    /**
     * Query the analog pressure sensor.
     *
     * @return the measured pressure, in PSI
     */
    public double getPressure()
    {
        // Get the pressure (in PSI) from an analog pressure sensor connected to the RIO.
        return pressureTransducer.get();
    }
    
    
    /**
     * Disable the compressor closed-loop for as long as the command runs.
     *
     * <p>Structured this way as the compressor is enabled by default.
     *
     * @return command
     */
    public Command disableCompressorCommand()
    {
        return startEnd(
                // Disable closed-loop mode on the compressor.
                compressor::disable,
                // Enable closed-loop mode based on the digital pressure switch connected to the
                // PCM/PH.
                // The switch is open when the pressure is over ~120 PSI.
                compressor::enableDigital)
                .withName("Compressor Disabled");
    }
}

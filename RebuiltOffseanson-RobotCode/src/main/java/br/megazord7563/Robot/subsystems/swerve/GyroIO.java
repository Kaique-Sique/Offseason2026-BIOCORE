// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord7563.Robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO 
{
    public class GyroIOinputs 
    {
        boolean connected = false;
        Rotation2d robotRotation2d = new Rotation2d();
    }

    public default void initialize() {}
 
    public default void resetPosition() {}

    public default void setYaw(Double rz) {}
    
    public default void updateInputs(GyroIOinputs inputs) {}
}
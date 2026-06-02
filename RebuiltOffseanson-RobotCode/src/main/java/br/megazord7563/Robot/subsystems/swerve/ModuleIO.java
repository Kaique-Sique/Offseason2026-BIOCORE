// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package br.megazord7563.Robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import org.littletonrobotics.junction.AutoLog;

import br.megazord7563.lib.Priority;
import br.megazord7563.lib.Telemetry;

public interface ModuleIO {
  
  @AutoLog
  public class ModuleIOInputs {
    @Telemetry(priority = Priority.LOW, basePath = "Swerve/Module", key = "DriveConnected")
    boolean driveConnected = false;

    @Telemetry(priority = Priority.LOW, basePath = "Swerve/Module", key = "DrivePositionRad")
    double drivePositionRad = 0.0;

    @Telemetry(priority = Priority.LOW, basePath = "Swerve/Module", key = "DriveSpeedRads")
    double driveSpeedRads = 0.0;

    @Telemetry(priority = Priority.LOW, basePath = "Swerve/Module", key = "DriveCurrent")
    double driveCurrent = 0.0;

    @Telemetry(priority = Priority.LOW, basePath = "Swerve/Module", key = "DriveTemperature")
    double driveTemperature = 0.0;

    @Telemetry(priority = Priority.LOW, basePath = "Swerve/Module", key = "DriveAppliedVolts")
    double driveAppliedVolts = 0.0;

    @Telemetry(priority = Priority.LOW, basePath = "Swerve/Module", key = "TurningConnected")
    boolean turningConnected = false;

    @Telemetry(priority = Priority.LOW, basePath = "Swerve/Module", key = "TurningPositionRad")
    double turningPositionRad = 0.0;

    @Telemetry(priority = Priority.LOW, basePath = "Swerve/Module", key = "TurningSpeedRads")
    double turningSpeedRads = 0.0;

    @Telemetry(priority = Priority.LOW, basePath = "Swerve/Module", key = "TurningCurrent")
    double turningCurrent = 0.0;

    @Telemetry(priority = Priority.LOW, basePath = "Swerve/Module", key = "TurningTemperature")
    double turningTemperature = 0.0;

    @Telemetry(priority = Priority.LOW, basePath = "Swerve/Module", key = "TurnAppliedVolts")
    double turnAppliedVolts = 0.0;

    @Telemetry(priority = Priority.LOW, basePath = "Swerve/Module", key = "TurnRotation")
    Rotation2d turnRotation2d = Rotation2d.kZero;

    @Telemetry(priority = Priority.LOW, basePath = "Swerve/Module", key = "TurnAbsoluteRotation")
    Rotation2d turnAbsoluteRotation2d = Rotation2d.kZero;

    SwerveModuleState moduleState = new SwerveModuleState();
    SwerveModulePosition modulePosition = new SwerveModulePosition();

    DriveMode driveMode = DriveMode.BREAK;
    TurnMode turnMode = TurnMode.COAST;

    @Telemetry(priority = Priority.LOW, basePath = "Swerve/Module", key = "ChassisAngularOffset")
    double m_chassisAngularOffset = 0.0;
  }

  /**
   * outputs is a class that defines the outputs of the module. It is used to set
   * the desired state
   * of the module in the SwerveModule class.
   */
  public class ModuleIOOutputs {
    /* define module goals */
    DriveMode driveMode = DriveMode.BREAK;
    TurnMode turnMode = TurnMode.COAST;

    public double driveCharacterizationOutput = 0.0;
    public double driveVelocityRadPerSec = 0.0;

    public Rotation2d turnRotation = Rotation2d.kZero;
  }

  /**
   * DriveMode is an enum that represents the different modes of the drive motor.
   * It is used to set
   * the desired state of the drive motor in the SwerveModule class.
   */
  public enum DriveMode {
    BREAK,
    COAST,
    DRIVE,
    CHARACTERIZATION
  }

  /**
   * TurnMode is an enum that represents the different modes of the turning motor.
   * It is used to set
   * the desired state of the turning motor in the SwerveModule class.
   */
  public enum TurnMode {
    BREAK,
    COAST,
    DRIVE,
    CHARACTERIZATION
  }

  /**
   * updateInputs is a method that updates the inputs of the module. It is called
   * periodically in
   * the SwerveModule class to update the state of the module.
   *
   * @param inputs
   */
  public default void updateInputs(ModuleIOInputs inputs) {
  }

  /**
   * applyOutputs is a method that applies the outputs of the module. It is called
   * in the
   * SwerveModule class to set the desired state of the module.
   *
   * @param outs
   */
  public default void applyOutputs(ModuleIOOutputs outs) {
  }

  /**
   * resetDriveEncoders is a method that resets the drive encoders of the module.
   * It is called in
   * the SwerveModule class to reset the drive encoders when the module is
   * initialized.
   */
  public default void resetDriveEncoders() {
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord7563.Robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;
import br.megazord7563.Robot.Constants.ModuleConstants;
import br.megazord7563.Robot.Utils.Conversions;
import br.megazord7563.Robot.subsystems.swerve.ModuleIO.DriveMode;
import br.megazord7563.Robot.subsystems.swerve.ModuleIO.ModuleIOOutputs;
import br.megazord7563.Robot.subsystems.swerve.ModuleIO.TurnMode;
import br.megazord7563.lib.TelemetryManager;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
  ModuleIO moduleIO;
  ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

  private Alert driveMotorAlert;
  private Alert turnMotorAlert;

  ModuleIOOutputs outs = new ModuleIOOutputs();

  String modulePose;

  SwerveModuleState m_desiredState;

  public SwerveModule(ModuleIO moduleIO, String modulePose) {
    this.moduleIO = moduleIO;
    this.modulePose = modulePose;

    driveMotorAlert = new Alert("Drive Motor Disconnected on Module " + modulePose, AlertType.kWarning);
    turnMotorAlert = new Alert("Turn Motor Disconnected on Module " + modulePose, AlertType.kWarning);

    TelemetryManager.getInstance().registerInputs(inputs, modulePose);
  }

  @Override
  public void periodic() {
    moduleIO.updateInputs(inputs);
    moduleIO.applyOutputs(outs);

    if (!inputs.driveConnected) {
      driveMotorAlert.set(true);
    } else {
      driveMotorAlert.set(false);
    }

    if (!inputs.turningConnected) {
      turnMotorAlert.set(true);
    } else {
      turnMotorAlert.set(false);
    }

    

    Logger.processInputs("Drive/Module" + modulePose, inputs);
  }

  /**
   * Sets the desired state for the module.
   * 
   * @param state Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState state) {
    m_desiredState = state;

    SwerveModuleState correctState = new SwerveModuleState();

    correctState = state;

    correctState.optimize(inputs.turnRotation2d);
    correctState.cosineScale(inputs.turnRotation2d);
    correctState.angle.plus(Rotation2d.fromRadians(inputs.m_chassisAngularOffset));

    outs.driveMode = DriveMode.DRIVE;
    outs.turnMode = TurnMode.DRIVE;
    
    double driveVelocityRPS = Conversions.MPSToRPS(state.speedMetersPerSecond,
        ModuleConstants.kWheelCircumferenceMeters,
        1.0);

    outs.driveVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(driveVelocityRPS * 60);
    outs.turnRotation = correctState.angle;
    
    moduleIO.applyOutputs(outs);
  }

  /**
   * Returns the current position of the module.
   * Drive position in meters and turning position in radians.
   * 
   * @return The current state of the module.
   *
   */
  public SwerveModulePosition getPosition() {
    moduleIO.updateInputs(inputs);
    return new SwerveModulePosition(
        Units.radiansToRotations(inputs.drivePositionRad) *
            ModuleConstants.kWheelCircumferenceMeters,
        inputs.turnRotation2d.minus(Rotation2d.fromDegrees(inputs.m_chassisAngularOffset)));
  }

  /**
   * Returns the current state of the module.
   * 
   * Drive speed in meters per second and turning position in radians.
   * 
   * @return The current state of the module.
   *
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(Units.radiansPerSecondToRotationsPerMinute(inputs.driveSpeedRads) / 60,
        inputs.turnRotation2d.minus(Rotation2d.fromDegrees(inputs.m_chassisAngularOffset)));
  }

  /**
   * set coast to turn motor and break to drive
   */
  public void stopMotors() {
    outs.driveMode = DriveMode.BREAK;
    outs.turnMode = TurnMode.COAST;
  }

  /**
   * Reset drive encoders
   */
  public void resetDriveEncoders() {
    moduleIO.resetDriveEncoders();
  }

  /**
   * set coast to both motors
   */
  public void disableMotors() {
    outs.driveMode = DriveMode.COAST;
    outs.turnMode = TurnMode.COAST;
  }

  /**
   * 
   * @return turningMotor velocity as double
   */
  public double getTurningVelocity() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.turningSpeedRads) / 60;
  }

  /**
   * 
   * @return driveMotor current as double
   */
  public double getDriveCurrent() {
    return inputs.driveCurrent;
  }

  /**
   * 
   * @return turningMotor current as double
   */
  public double getTurnCurrent() {
    return inputs.turningCurrent;
  }

  /**
   * 
   * @return driveMotor temperature as double
   */
  public double getDriveTemperature() {
    return inputs.driveTemperature;
  }

  /**
   * 
   * @return turningMotor temperature as double
   */

  public double getDriveVoltage() {
    return inputs.driveAppliedVolts;
  }

  /**
   * 
   * @return turningMotor voltage as double
   */
  public boolean isTurnMotorConnected() {
    return inputs.turningConnected;
  }

  /**
   * 
   * @return true if the drive motor is connected, false otherwise
   */
  public boolean isDriveMotorConnected() {
    return inputs.driveConnected;
  }
}
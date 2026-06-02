// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord7563.Robot.subsystems.swerve;

import br.megazord7563.Robot.Constants.ModuleConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * This is a placeholder class for the ModuleIOSim class. It is used to simulate
 * the ModuleIO class in the SwerveModuleSim class.
 * It is not used in the actual robot code, but it is used in the simulation
 * code.
 */
public class ModuleIOSim implements ModuleIO {
    /**
     * DCMotorSim is a class that simulates a DC motor. It takes in a DCMotor model
     * and a time step,
     * and it outputs the voltage applied to the motor based on the current state of
     * the motor and the
     * desired state of the motor. It is used to simulate the drive and turn motors
     * of the swerve module
     * in the SwerveModuleSim class.
     */
     private final DCMotor driveMotorModel = DCMotor.getKrakenX60(1);
    private final DCMotor turnMotorModel = DCMotor.getKrakenX44(1);
    private final DCMotorSim driveMotor = new DCMotorSim(LinearSystemId.createDCMotorSystem(driveMotorModel,  0.001, ModuleConstants.kDriveMotorGearRatio), driveMotorModel);
    private final DCMotorSim turnMotor = new DCMotorSim(LinearSystemId.createDCMotorSystem(turnMotorModel,  0.001, ModuleConstants.kTurningMotorGearRatio), turnMotorModel);

    /**
     * Module goals. These are the desired states of the module that are set in the
     * SwerveModule class.
     * They are used to calculate the outputs of the module in the SwerveModuleSim
     * class.
     */
    private TurnMode turnMode = TurnMode.COAST;
    private DriveMode driveMode = DriveMode.BREAK;

    /**
     * Module states. These are the actual states of the module that are read in the
     * SwerveModuleSim class.
     * They are used to calculate the outputs of the module in the SwerveModuleSim
     * class.
     */
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    /**
     * PID Controllers
     */
    private PIDController turnPID = new PIDController(10.0, 0, 0);
    private PIDController drivePID = new PIDController(0.1, 0, 0);

    public ModuleIOSim() {
        //turnPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        if (driveMode == DriveMode.DRIVE) {
            driveAppliedVolts = drivePID.calculate(driveMotor.getAngularVelocityRadPerSec());
        }
        if (turnMode == TurnMode.DRIVE) {
            turnAppliedVolts = turnPID.calculate(
                    new Rotation2d(turnMotor.getAngularPositionRad())
                            .getRadians());
        }

        driveMotor.setInputVoltage(driveAppliedVolts);
        turnMotor.setInputVoltage(turnAppliedVolts);

        inputs.driveConnected = true;
        inputs.drivePositionRad = driveMotor.getAngularPositionRad();
        inputs.driveSpeedRads = driveMotor.getAngularVelocityRadPerSec();
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveCurrent = driveMotor.getCurrentDrawAmps();
        inputs.driveTemperature = 0.0; // Temperature simulation can be added if needed

        inputs.turningConnected = true;
        inputs.turningPositionRad = turnMotor.getAngularPositionRad();
        inputs.turningSpeedRads = turnMotor.getAngularVelocityRadPerSec();
        inputs.turnAppliedVolts = turnAppliedVolts;
        inputs.turningCurrent = turnMotor.getCurrentDrawAmps();
        inputs.turningTemperature = 0.0; // Temperature simulation can be added if needed

        inputs.turnRotation2d = new Rotation2d(inputs.turningPositionRad);
        inputs.turnAbsoluteRotation2d = new Rotation2d(inputs.turningPositionRad);

        inputs.moduleState = new SwerveModuleState(inputs.driveSpeedRads, inputs.turnRotation2d);
        inputs.modulePosition = new SwerveModulePosition(inputs.drivePositionRad, inputs.turnRotation2d);

        inputs.driveMode = driveMode;
        inputs.turnMode = turnMode;

        driveMotor.update(0.02);
        turnMotor.update(0.02);
    }

    @Override
    public void applyOutputs(ModuleIOOutputs outputs) {
        // Set the drive motor voltage based on the drive mode
        switch (outputs.driveMode) {
            case BREAK:
                driveAppliedVolts = 0.0;
                break;
            case COAST:
                driveAppliedVolts = 0.0;
                break;
            case DRIVE:
                drivePID.setSetpoint(outputs.driveVelocityRadPerSec);
                break;
            case CHARACTERIZATION:
                driveAppliedVolts = outputs.driveCharacterizationOutput;
                break;
            default:
                driveAppliedVolts = 0.0;
                break;
        }

        // Set the turn motor voltage based on the turn mode
        switch (outputs.turnMode) {
            case BREAK:
                turnAppliedVolts = 0.0;
                break;
            case COAST:
                turnAppliedVolts = 0.0;
                break;
            case DRIVE:
                turnPID.setSetpoint(outputs.turnRotation.getRadians());
                
                break;
            default:
                turnAppliedVolts = 0.0;
                break;
        }

        turnMode = outputs.turnMode;
        driveMode = outputs.driveMode;
    }
}

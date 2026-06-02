// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package br.megazord7563.Robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static final class RobotConstants {
    public static final MODE robotMode = edu.wpi.first.wpilibj.RobotBase.isReal() ? MODE.REAL : MODE.SIM;

    public enum MODE {
      REAL,
      SIM
    }
  }

  public static final class ModuleConstants {

    public static final double kDrivingMotorFreeSpeedRps = KrakenMotorConstants.kFreeSpeedRpm / 60;

    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);// 0.1016m
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;// 0.1016*3.14=

    public static final double kDriveL3 =  6.12;
    public static final double kTurningL3 = 21.42857; // The steering gear ratio of the MK4i is 150/7:1.

    public static final double kDriveL4 = 1.0 / 5.14;
    public static final double kTurningL4 = 1.0 / 12.8; // The steering gear ratio of the MK4 is 12.8:1.

    public static final double kDriveMotorGearRatio = kDriveL3; // 1 / 5.14; //L4 L3=1/6.12
    public static final double kTurningMotorGearRatio = kTurningL3; // 1 /12.8; // 1/18.0

    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * (2 * Math.PI);

    // public static final double kTurningEncoderPositionFactor =2 * Math.PI; //
    // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;

    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        * kDriveMotorGearRatio;
    public static final double kDriveRPM = kDriveEncoderRPM2MeterPerSec * 60;
    public static final double kDriveRPMRatio = kDriveRPM / kDriveMotorGearRatio;
    public static final double kDriveRPMPi = kDriveRPMRatio / Math.PI;
    public static final double kDriveRPMSpeed = kDriveRPMPi / kWheelDiameterMeters;

    /**
     * MNL 09/11/2025 Tuner X Constants
     * Steer Gains
     * Ks = 0.1, Kv = 1.59, Ka = 0.0 , kp = 100.0, kD = 0.5, kI = 0.0
     * 
     * TEste 09/23/2025 Units =Rotations
     * Steer Ks Kv Ka Kp Kd
     * Valor médio 0.3064025 1.602225 0.01757525 64.55725 2.529675
     * 
     * Drive ID Ks Kv Ka Kp
     * valor médio 0.221835 0.7482675 0.0498445 0.5347
     */

    // 14,285%
    public static final double turningKS = 0.0; // 0.266;// 0.5;//1.5;//0.1;//<-MNL 11/11/2024 0.1; //0.32; // Add 0.1 V
                                                // output
                                                // to overcome static friction
    public static final double turningKV = 0.04284;//calculado // 2.547;// 1.63;//0.12; //1.51; // A velocity target of 1 rps results
                                                // in
                                                // 0.12 V output
    public static final double turningKA = 0.0;// 0.11; //0.27;
    public static final double kPTurning = 75.248;// 350.0;//100 ANTERIOR 0,5 //0,25 ; 0.2142875; 0.175; 0.125; 0.23
    public static final double kITurning = 0.0;
    public static final double kDTurning = 0;// 1.0;//0.5

    // The F parameter should only be set when using a velocity-based PID
    // controller,
    // and should be set to zero otherwise to avoid unwanted behavior.
    // public static final double kTurningFF = 0; //1.0/473.0; //

    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    /**********************************
     * Driving TALON Fx PID settings *
     *********************************/
    /*
     * 
     * /**
     * MNL 09/11/2025 Tuner X Constants
     * Steer Gains
     * Ks = 0.1, Kv = 1.59, Ka = 0.0 , kp = 100.0, kD = 0.5, kI = 0.0
     * 
     * Drive Gains
     * Ks = 0.0, Kv = 0.124, Ka = 0.00 , kp = 0.1, kD = 0.0, kI = 0.0
     * 
     * TEste 09/23/2025 Units =Rotations
     * Steer Ks Kv Ka Kp Kd
     * Valor médio 0.3064025 1.602225 0.01757525 64.55725 2.529675
     * 
     * Drive ID Ks Kv Ka Kp
     * valor médio 0.221835 0.7482675 0.0498445 0.5347
     */
    public static final double driveKS = 0;// 0.2306; ///2;// 0.221835;//0.5;//1.5<-MNL 11/11/2024 0.1; //0.32; // Add
                                           // 0.1 V output
                                           // to overcome static friction
    public static final double driveKV = 0.038;// calculado// 0.7649;// / 2 ;// 0.7482675;//0.124; //1.51; // A velocity target of 1 rps
                                           // results in
                                           // 0.12 V output
    public static final double driveKA = 0.0; // 0.27;

    // 11,69%
    public static final double kPdriving = 0.15;//pratic // 1.04; // 0.1;//<--MNL11/11/2024 0.0665;//ANTERIOR 0,0665 ; 0.05872615
                                                // // kP =
                                                // 0.11 An error of 1 rps results in 0.11 V output
    public static final double kIdriving = 0.0;
    public static final double kDdriving = 0.0;

    public static final double kFFdriving = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    /* Swerve Current Limiting */
    // TalonFX
    public static final int driveSupplyLowerLimit = 40; // supply current
    public static final int driveSupplyCurrent = 70; // supply current
    public static final int driveStatorCurrent = 120; // stator current

    public static final double driveCurrentThresholdTime = 1.0;
    public static final boolean driveEnableCurrentLimit = true;

    public static double kDriveClosedLoopRamp = 0.25;

    // SparkMax
    public static final int turningSupplyLowerLimit = 30; // supply current
    public static final int turningSupplyCurrent = 40; // supply current
    public static final int turningStatorCurrent = 60; // stator current

    public static final double turningCurrentThresholdTime = 1.0;
    public static final boolean turningEnableCurrentLimit = true;
    public static double kTurningClosedLoopRamp = 0.25;

    public static final double kTurningEncoderPositionPIDMinInput = 0;
    public static final double kTurningEncoderPositionPIDMaxInput = 2 * Math.PI;

    /* Neutral Modes */
    // public static final IdleMode kTurningMotorIdleMode = IdleMode.kCoast; //
    // template was coast SparkMax
    public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake; // TalonFx
    public static final NeutralModeValue turningNeutralMode = NeutralModeValue.Coast; // TalonFx

    // Spark Slot
    public static final ClosedLoopSlot pidSparkSlot = ClosedLoopSlot.kSlot0;

    // CANivore CANbus
    public static final CANBus swerveCAN = new CANBus("swerve");
  }

  // Drive Constants
  public static final class DriveConstants {

    // --------------------------------------------------------------------------
    // 1. DRIVE MODE ENUM 🚦
    // Defines the named speed settings for teleoperated control.
    // --------------------------------------------------------------------------
    public enum DriveMode {
      SLOW, // Very low speed for precise maneuvers (e.g., scoring)
      FAST, // Normal driving speed
      MAX, // Highest possible speed (e.g., traveling across the field)
      LIFT // Extremely slow speed for alignment or lifting

    }

    public enum LedMode {
      BLUE, // Very low speed for precise maneuvers (e.g., scoring)
      GREEN, // Normal driving speed
      PURPLE, // Highest possible speed (e.g., traveling across the field)
      YELLOW,
      RED,
      RED_WAVE,
      BLUE_WAVE,
      GREEN_WAVE,
      RAINBOW,
      ORANGE_WAVE,
      ORANGE,
      WHITE_WAVE,
      PURPLE_WAVE,
      BLACK,
      PINK,
      WHITE,
      hubActive,
      hubNotActive,
      amoustActive,
      AmoustNotActive // Extremely slow speed for alignment or lifting
    }

    /**************************************
     * Specify the kinematics of our robot*
     ************************************/
    public static final double kTrackWidth = 0.552;// Units.inchesToMeters(21);
    // Distance between right and left wheels
    public static final double kWheelBase = 0.539;// Units.inchesToMeters(21);
    // Distance between front and back wheels
    public static final double kDriveRadius = Math.hypot(kTrackWidth / 2, kWheelBase / 2);

    // Robot physical properties useful for path planning and control
    public static final double totalMassKg = 64;
    public static final double momentOfInertia = 4.5464; // IA calculou ->4.5464 kg·m², anterior 6.88;
    public static final double coeficientFriction = 1.2;
    public static final double driveGearRatio = 6.12; // 6.12:1 L3

    /**********************************************************************
     * Swerve Drive Object - It specifies the location of each swerve *
     * module on the robot this way the wpi library can construct the *
     * geometry of our robot setup and do all the calculations *
     * 
     * @see Modules Location: FL= +X,+Y; FR= +X,-Y; BL=-X, +Y; BR=-X, -Y,*
     **********************************************************************/
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2), // + - antes
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // + + antes
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // - - antes
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2) // - + antes
    );

    //
    /**
     * MNL 17/10/2024
     * kFrontLeftChassisAngularOffset = 0: This means that when your front left
     * module's turning motor is at its zero position,
     * the module is pointing straight forward relative to the chassis.
     * kFrontRightChassisAngularOffset = 0: This suggests your front right module is
     * also pointing forward when its turning motor
     * is at zero.
     * kBackLeftChassisAngularOffset = Math.PI: This means that your back left
     * module is pointing 180 degrees from the front left
     * module. So, when its turning motor is at zero, the module is pointing
     * straight backward relative to the chassis.
     * kBackRightChassisAngularOffset = Math.PI: This means that your back right
     * module is also pointing straight backward
     * when its turning motor is at zero.
     ******/

    public static final double kFrontLeftChassisAngularOffset = 0;//
    public static final double kFrontRightChassisAngularOffset = Math.PI;
    public static final double kBackLeftChassisAngularOffset = 0;
    public static final double kBackRightChassisAngularOffset = Math.PI;

    // Portas dos Kraken
    public static final int kFrontLeftDriveMotorPort = 3;
    public static final int kBackLeftDriveMotorPort = 6;
    public static final int kFrontRightDriveMotorPort = 9;
    public static final int kBackRightDriveMotorPort = 12;

    // Portas dos Sparks
    public static final int kFrontLeftTurningMotorPort = 4;
    public static final int kBackLeftTurningMotorPort = 7;
    public static final int kFrontRightTurningMotorPort = 10;
    public static final int kBackRightTurningMotorPort = 13;

    // CanCoder ports
    public static final int kFrontLeftDriveAbsoluteEncoderPort = 2;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 5;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 8;
    public static final int kBackRightDriveAbsoluteEncoderPort = 11;
    public static final int kPigeonPort = 30;// 7563

    /**
     * SINTONIA DAS RODAS
     * FL => FRONT LEFT
     * FR => FRONT RIGHT
     * BL => BACK LEFT
     * BR => BACK RIGHT
     */

    public static final Rotation2d angleOffsetFLTurning = Rotation2d.fromDegrees(35.5078125);// -4.85
    public static final Rotation2d angleOffsetFRTurning = Rotation2d.fromDegrees(111.62109375);// -77.43, -77.34375
    public static final Rotation2d angleOffsetBLTurning = Rotation2d.fromDegrees(-103.623046875);// 142.07
    public static final Rotation2d angleOffsetBRTurning = Rotation2d.fromDegrees(-53.0859375);// -91.14

    /*******************************************************
     * constante que limita a velocidade maxima drive teleop*
     ******************************************************/

    public static final double kPhysicalMaxSpeedMetersPerSecond = 5.2; // kRAKEN - GEAR RATIO L3
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;// 2*2* Math.PI; //

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;// anterior 1.052,
                                                                                                    // 1.175
                                                                                                    // 4,25531914893617
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 2;// anterior
                                                                                                                        // 2

    // Slew Rate adjustments
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 2;// anterior 2 - 2/12/2024 MNL
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 2;// anterior 2 2/12/2024 MNL

    // it's not currently used
    /*
     * public static final double kDirectionSlewRate = 1.2; // radians per second
     * public static final double kMagnitudeSlewRate = 1.8; // percent per second (1
     * = 100%)
     * public static final double kRotationalSlewRate = 2.0; // percent per second
     * (1 = 100%)
     */

    public static final String limelightFront = "limelight-one";
    public static final String limelightBack = "limelight-two";
    public static final String limelightLeft = "limelight-four";
    public static final String limelightRight = "limelight-three";
  }

  /*************************************************************
   * constante que limita a velocidade maxima drive autonomous *
   *************************************************************/
  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 4.0;//DriveConstants.kPhysicalMaxSpeedMetersPerSecond ; // 4
    public static final double kMaxAccelerationMetersPerSecondSquared = 3.0; // 3
    
   
    public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond;// div/2//10,20

    
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = 3*Math.PI ; // pi/2//4

    public static final double kPXController = 10;// 1.5
    public static final double kPYController = 10;// 1.5
    public static final double kPThetaController = 2 * Math.PI;

    public static final double kOffset = Units.inchesToMeters(9);
    public static final double kOffsetSide = Units.inchesToMeters(3);

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = 
        new TrapezoidProfile.Constraints(
                                          kMaxAngularSpeedRadiansPerSecond,
                                          kMaxAngularAccelerationRadiansPerSecondSquared);
  }

  /**
   * Path Planner Constants
   */
  public static final class PathPlannerConstants {
    public static final double kPTranslationPath = 3.5;// 3.5 Anterior 0.125,10
    public static final double kITranslationPath = 0.0;
    public static final double kDTranslationPath = 0.0;

    public static final double kPRotationPath = 1.9;// 2.0, 3.0; //3.0; 1.6
    public static final double kIRotationPath = 0.0;
    public static final double kDRotationPath = 0.000;

    public static final PIDConstants kPIDRotationPath = new PIDConstants(kPRotationPath, kIRotationPath,
        kDRotationPath);
    public static final PIDConstants kPIDTranslationPath = new PIDConstants(kPTranslationPath, kITranslationPath,
        kDTranslationPath);

    public static final PPHolonomicDriveController AutoConfig = new PPHolonomicDriveController(kPIDTranslationPath,
        kPIDRotationPath);

    public static final double maxAccelerationPath = 1;// 1.75//1.0//5.0 //3.0
    public static final double maxAngularVelocityRadPerSec = Units.degreesToRadians(360);
    public static final double maxAngularAccelerationRadPerSecSq = Units.degreesToRadians(540);

    public static RobotConfig robotConfig = new RobotConfig(DriveConstants.totalMassKg, // The mass of the robot,
                                                                                        // including bumpers and
                                                                                        // battery, in Kilograms.
        DriveConstants.momentOfInertia, // IA calculou ->4.5464 kg·m², anterior 6.88
        new ModuleConfig(ModuleConstants.kWheelDiameterMeters / 2,
            DriveConstants.kPhysicalMaxSpeedMetersPerSecond,
            DriveConstants.coeficientFriction,
            DCMotor.getKrakenX60(1)
                .withReduction(DriveConstants.driveGearRatio),
            ModuleConstants.driveSupplyCurrent,
            1),
        DriveConstants.kDriveKinematics.getModules());

  }
  /*******************************************************************
   * @param https://docs.revrobotics.com/brushless/neo/v1.1/neo-v1
   *
   *****************************************************************/

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  /***********************************************************************************************************************************
   * @param https://docs.wcproducts.com/kraken-x60/kraken-x60-motor/overview-and-features/motor-performance
   *
   * @param https://store.ctr-electronics.com/announcing-kraken-x60/?srsltid=AfmBOorh3sPSXQ-WmuWYeJlxrIkATC1wRVPc0V65woNtynzQ1Sil1Ueh
   *
   *************************************************************************************************************************************/
  public static final class KrakenMotorConstants {
    public static final double kFreeSpeedRpm = 6000;
  }

  /*
   * Subsystems Constants
   *
   * implement here constants for subsystems
   */
  public static final class SubsystemsConstants {
    public static final class IntakeConstants {
      public static final double kIntakeUpPosition = 0.0;
      public static final double kIntakeDownPosition = -50.0;

      /* Intake Constants */
      public static final int kRollersMotorID = 19;

      public static final boolean kRollersMotorEnableCurrentLimit = true;
      public static final int kRollersMotorGearRatio = 1;
      public static final double kRollersClosedLoopRamp = 8;
      public static final NeutralModeValue kRollersMotorNeutalMode = NeutralModeValue.Coast;
      public static final double kRollersMotorThresholdCurrent = 80;
      // Pid Constants
      public static final double KSrollers = 0;
      public static final double KVrollers = 0;
      public static final double kProllers = 1;
      public static final double kDrollers = 0.2;
      public static final double kIrollers = 0.1;

      /* Ariculator Constants */
      public static final int kArticulatorMotorID = 10;

      public static final NeutralModeValue kArticulatorMotorNeutalMode = NeutralModeValue.Brake;
      public static final double kArticulatorMotorThresholdCurrent = 20;
      public static final boolean kArticulatorMotorEnableCurrentLimit = true;
      public static final double kArticulatorMotorGearRatio = 1;
      public static final double kArticulatorClosedLoopRamp = 8;
      // Pid constants
      public static final double KSArticulator = 0;
      public static final double KVArticulator = 0;
      public static final double kPArticulator = 20;
      public static final double kIArticulator = 0.06;
      public static final double kDArticulator = 3;

      public static final double IntakeModeSpeed = 20;

      public static final double OutakeModeSpeed = -20;
    }

    public static final class shooterConstants {
      public static final class ShooterGeometryConstants {

        /* shooter pose on robot */
        public static final double Dx = -0.145;
        public static final double Dy = -0.15;
        public static final double Dz = 0.364;

        /******************************
         * Geometry Importants Values *
         ******************************/
        public static final Transform3d RobotToTurret3d =
            new Transform3d(new Translation3d(Dx, Dy, Dz), Rotation3d.kZero);

        public static final Transform2d RobotToTurret2d =
            new Transform2d(new Translation2d(Dx, Dy), new Rotation2d(0.0));
      }

      public static final class ShootCalculatorConstansts {
        public static InterpolatingDoubleTreeMap ToFMap = new InterpolatingDoubleTreeMap();

        public static InterpolatingDoubleTreeMap HoodAngleMap = new InterpolatingDoubleTreeMap();

        public static InterpolatingDoubleTreeMap FlywheelMap = new InterpolatingDoubleTreeMap();

        public static InterpolatingDoubleTreeMap getTofmap() {
          return ToFMap;
        }

        public static InterpolatingDoubleTreeMap getHoodanglemap() {
          return HoodAngleMap;
        }

        public static InterpolatingDoubleTreeMap getFlywheelmap() {
          return FlywheelMap;
        }

        public static final double minDistance = 1.34;
        public static final double maxDistance = 5.60;

        static {
          HoodAngleMap.put(1.0, 60.0);
          HoodAngleMap.put(1.5, 60.0);
          HoodAngleMap.put(2.0, 60.0);
          HoodAngleMap.put(2.5, 60.0);
          HoodAngleMap.put(3.0, 60.0);

          FlywheelMap.put(1.0, 846.0);
          FlywheelMap.put(1.5, 873.0);
          FlywheelMap.put(2.0, 935.0);
          FlywheelMap.put(2.5, 1026.0);
          FlywheelMap.put(3.0, 1149.0);

          ToFMap.put(5.68, 1.1);
          ToFMap.put(4.55, 1.0);
          ToFMap.put(3.15, 0.8);
          ToFMap.put(1.88, 0.60);
          ToFMap.put(1.38, 0.50);
        }
      }

      public static final class TurretConstants {
        /* to implement */
        /* motors infos */
        public static final int kTurretMotorId = 19;
        public static final NeutralModeValue kMotorNeutralMode = NeutralModeValue.Coast;

        /* current limit */
        public static final double kMotorThresholdCurrent = 40;
        public static final boolean kMotorEnableCurrentLimit = false;
        /* feedback sensor */
        public static final double kMotorGearRatio = 1;
        public static final boolean kContinuousWrap = false;

        /* PID constants */
        public static final double kDriveClosedLoopRamp = 0;
        public static final double KS = 0;
        public static final double KV = 0;
        public static final double kP = 1;
        public static final double kI = 0.04;
        public static final double kD = 0.3;

        /* soft limits configs */
        public static final boolean kForwardSoftLimitEnable = false;
        public static final boolean kReverseSoftLimitEnable = false;
        public static final double kForwardSoftLimitThreshold = 0;
        public static final double kReverseSoftLimitThreshold = 0;
      }

      public static final class CapoArticulatorConstants {
        /* to implement */
        /* motors infos */
        public static final int kCapoArticulatorMotorId = 10;
        public static final NeutralModeValue kMotorNeutalMode = NeutralModeValue.Coast;
        public static final MotorType kMotorType = MotorType.kBrushless;

        /* current limit */
        public static final double kMotorThresholdCurrent = 20;
        public static final boolean kMotorEnableCurrentLimit = false;

        /* feedback sensor */
        public static final int kMotorGearRatio = 1;
        public static final boolean kContinuousWrap = false;

        /* PID constants */
        public static final double KS = 0;
        public static final double KV = 0;
        public static final double kP = 0.04;
        public static final double kI = 0;
        public static final double kD = 0;

        /* soft limits */
        public static final boolean kForwardSoftLimitEnable = false;
        public static final boolean kReverseSoftLimitEnable = false;
        public static final double kForwardSoftLimitThreshold = 0;
        public static final double kReverseSoftLimitThreshold = 0;
        public static final double kDriveClosedLoopRamp = 0;
      }

      public static final class shooterWheelConstants {

        public static final int kFollowerMotorId = 11;
        /* to implement */
        public static final int kShooterWheelMotorId = 12;
        public static final NeutralModeValue kMotorNeutralMode = NeutralModeValue.Coast;
        public static final double kMotorThresholdCurrent = 80;
        public static final boolean kMotorEnableCurrentLimit = false;
        public static final boolean kContinuousWrap = false;
        public static final double kMotorGearRatio = 1;
        public static final double kSlot0kS = 0;
        public static final double kSlot0kV = 0;
        public static final double kSlot0kP = 0.5;
        public static final double kSlot0kI = 0;
        public static final double kSlot0kD = 0;
        public static final double kSlot1kS = 0;
        public static final double kSlot1kV = 0;
        public static final double kSlot1kP = 0.2;
        public static final double kSlot1kI = 0;
        public static final double kSlot1kD = 0;
      }
    }
  }

  /**
   * Holonomic Drive Controller Configuration
   *
   * @param kpTranslation translation proportional gain
   * @param kITranslation translation integral gain
   * @param kDTranslation translation derivative gain
   * @param kPRotation rotation proportional gain
   * @param kIRotation rotation integral gain
   * @param kDRotation rotation derivative gain
   * @param kPTheta trapezoid profiled theta proportional gain
   * @param kITheta trapezioid profiled theta integral gain
   * @param kDTheta trapezioid profiled theta derivative gain
   * @param maxAngularVelocity trapezoid constraints max angular velocity rad/s
   * @param maxAngularAcceleration trapezoid constraints max angular acceleration rad/s²
   * @return new HolonomicDriveController
   */
  public static HolonomicDriveController HolonomicControllerConfig(
      double kpTranslation,
      double kITranslation,
      double kDTranslation,
      double kPRotation,
      double kIRotation,
      double kDRotation,
      double kPTheta,
      double kITheta,
      double kDTheta,
      double maxAngularVelocity,
      double maxAngularAcceleration) {
    // MNL 09/26/2025 - HolonomicDriveController need to try out

    return new HolonomicDriveController(
        new PIDController(kpTranslation, kITranslation, kDTranslation), // translation constants
        new PIDController(kPRotation, kIRotation, kDRotation), // rotatoin constants
        new ProfiledPIDController(
            kPTheta,
            kITheta,
            kDTheta,
            // trapezoidal profiled PID controller

            // Here, our rotation profile constraints were a max velocity
            // of 1 rotation per second and a max acceleration of 180 degrees
            // per second squared.
            new TrapezoidProfile.Constraints(maxAngularVelocity, maxAngularAcceleration),
            0.02) // max angular velocity
        // and acceleration
        ); // Rotation PID constants)
  }
}

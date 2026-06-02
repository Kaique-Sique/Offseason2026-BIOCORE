// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord7563.Robot.subsystems.swerve;

import java.util.Comparator;
import java.util.List;
import java.util.Objects;
import java.util.Set;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import br.megazord7563.Robot.Constants.DriveConstants;
import br.megazord7563.Robot.Constants.PathPlannerConstants;
import br.megazord7563.Robot.Constants.RobotConstants;
import br.megazord7563.Robot.Constants.RobotConstants.MODE;
import br.megazord7563.Robot.subsystems.swerve.GyroIO.GyroIOinputs;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import br.megazord7563.Robot.LimelightHelpers;

public class SwerveSubsystem extends SubsystemBase {
  /******************************
   ** Swerve Modules Instances **
   *****************************/
  private SwerveModule frontLeftModule;
  private SwerveModule frontRightModule;
  private SwerveModule backLeftModule;
  private SwerveModule backRightModule;

  private SwerveModule[] modules;

  /************************
   ** Gyroscopic Instace **
   ************************/
  private GyroIO gyro;
  private GyroIOinputs gyroIOinputs = new GyroIOinputs();
  private Rotation2d gyroRotation2d = Rotation2d.kZero;
  double countLL = 0.0;

  private boolean isAllianceReset = false;
  private boolean blueAlliance = true;

  private final Alert gyroErrorAlert = new Alert("Gyro Error: Check connections!", AlertType.kWarning);

  SwerveModuleState[] states = new SwerveModuleState[] {
      new SwerveModuleState(),
      new SwerveModuleState(),
      new SwerveModuleState(),
      new SwerveModuleState()
  };

  enum SPEEDMode {
    SLOW(0.30),
    FAST(0.60),
    MAX(0.85);

    private double speed;

    private SPEEDMode(double speed) {
      this.speed = speed;
    }

    public double getSpeedValue() {
      return speed;
    }
  }

  private SPEEDMode speedMode = SPEEDMode.SLOW;

  // PathPlanner Config
  private RobotConfig config;
  private final SwerveDrivePoseEstimator m_poseEstimator;

  /**
   * NetworkTables publisher for Pose2d data.
   * Useful for debugging and visualization in tools like Shuffleboard or custom
   * dashboards.
   * Advantage Scope: Can visualize robot pose in real-time.
   */
  private StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
      .getStructTopic("MyPose", Pose2d.struct)
      .publish();

  private StructArrayPublisher<SwerveModuleState> publisherMeasured = NetworkTableInstance.getDefault()
      .getStructArrayTopic("MyMeasuredStates", SwerveModuleState.struct)
      .publish();

  private StructArrayPublisher<SwerveModuleState> publisherDesired = NetworkTableInstance.getDefault()
      .getStructArrayTopic("MyDesiredStates", SwerveModuleState.struct)
      .publish();

  private StructPublisher<ChassisSpeeds> publisherChassisSpeeds = NetworkTableInstance.getDefault()
      .getStructTopic("MyChassisSpeeds", ChassisSpeeds.struct)
      .publish();

  private StructPublisher<Rotation2d> publisherRotation2d = NetworkTableInstance.getDefault()
      .getStructTopic("MyRotation2d", Rotation2d.struct)
      .publish();

  private final SlewRateLimiter xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
  private final SlewRateLimiter turningLimiter = new SlewRateLimiter(
      DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

  /**
   * Standard deviations for the odometry and vision measurements.
   */
  private static final Matrix<N3, N1> odometryStdDevs = VecBuilder.fill(0.015, 0.015, (10 * Math.PI) / 180);// 5 graus
                                                                                                            // em
                                                                                                            // radianos
                                                                                                            // 5° ×
                                                                                                            // π/180
  private static final Matrix<N3, N1> visionStdDevs = VecBuilder.fill(0.15, 0.15, (5 * Math.PI) / 180);// 5 graus em
                                                                                                       // radianos 5° ×
                                                                                                       // π/180

  // Define the vision measurement standard deviations
  private static final Matrix<N3, N1> visionPoseStdDevs = VecBuilder.fill(0.7, 0.7, 9999999);

  /**
   * @param frontLeftModule
   * @param frontRightModule
   * @param backLeftModule
   * @param backRightModule
   */
  public SwerveSubsystem(
      SwerveModule frontLeftModule, 
      SwerveModule frontRightModule,
       SwerveModule backLeftModule,
      SwerveModule backRightModule, 
      GyroIO gyro) {

    this.frontLeftModule = frontLeftModule;
    this.frontRightModule = frontRightModule;
    this.backLeftModule = backLeftModule;
    this.backRightModule = backRightModule;

    this.gyro = gyro;
    gyro.initialize();


    modules = new SwerveModule[] { this.frontLeftModule, this.frontRightModule, this.backLeftModule,
        this.backRightModule };

    // Initialize pose estimator ALWAYS towards to red wall
    m_poseEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        gyroRotation2d,
        new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            backLeftModule.getPosition(),
            backRightModule.getPosition()
        },
        new Pose2d(),
        odometryStdDevs, // 5 graus em radianos 5° × π/180
        visionStdDevs);// 30 graus em radianos 30° × π/180

    try {
      // config = RobotConfig.fromGUISettings();

      // Manually create the Robot config
      config = PathPlannerConstants.robotConfig;

      AutoBuilder.configure(
          this::getPoseEstimator, // Robot pose supplier
          this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
          this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds.
          PathPlannerConstants.AutoConfig,
          config, // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              System.out.println("Auto builder ok!!");
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this // Reference to this subsystem to set requirements
      );
    } catch (Exception e) {
      // Handle exception as needed
      DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
    }
  }

  
  @Override
  public void periodic() {
    gyro.updateInputs(gyroIOinputs);

    if (!isAllianceReset && DriverStation.getAlliance().isPresent()) {
     
      Translation2d posPose = this.getPoseEstimator().getTranslation();
      m_poseEstimator.resetPosition(getGyroRotation2d(),
          this.getModulePositions(),
          new Pose2d(posPose,
              new Rotation2d(DriverStation.getAlliance().get() == Alliance.Blue ? 0.0 : Math.PI)));

      isAllianceReset = true;
    }


    if(countLL < 50)
    {
      seedLimelightHeading();
      countLL += 1;
    }

    // Output Module States to SmartDashboard
    SwerveModuleState[] moduleStates = this.getModuleStates();

    // Update the odometry in the periodic block
    this.updatePoseEstimator();

    // Publish the pose to NetworkTables
    publisher.set(this.getPoseEstimator());

    // Publish the module states to NetworkTables
    publisherMeasured.set(moduleStates);

    // Publish the desired module states to NetworkTables
    publisherDesired.set(states);

    // Publish ROBOT RELATIVE chassisSpeed
    publisherChassisSpeeds.set(this.getChassisSpeeds());

    // Publish robot rotation2d
    publisherRotation2d.set(this.getPoseEstimator().getRotation());

    gyroUpdate();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stopMotors();
      }
    }

    if (!gyroIOinputs.connected && RobotConstants.robotMode != MODE.SIM) {
      gyroErrorAlert.set(true);
    }
  }

  /**
   * Seeds the gyro yaw with the yaw from the Limelight botpose, adjusted for
   * alliance color.
   * 
   * @return botpose Yaw from limelight
   */
  public void seedLimelightHeading() 
  {

    boolean validTargetLeft = LimelightHelpers.getTV(DriveConstants.limelightLeft);
    boolean validTargetRight = LimelightHelpers.getTV(DriveConstants.limelightRight);

    if (validTargetLeft || validTargetRight) 
    {

      if (DriverStation.getAlliance().get() == Alliance.Blue) {
        Double rz = LimelightHelpers
            .getBotPose_wpiBlue(validTargetLeft ? DriveConstants.limelightLeft : DriveConstants.limelightRight)[5];
          System.out.println(validTargetLeft);
        gyro.setYaw(rz);
      } else {
        Double rz = LimelightHelpers
            .getBotPose_wpiRed(validTargetLeft ? DriveConstants.limelightLeft : DriveConstants.limelightRight)[5];
        gyro.setYaw(rz);
         System.out.println(rz);
      }
    }
  }

  public void updatePoseEstimator() {
    m_poseEstimator.update(gyroRotation2d,
        this.getModulePositions()
    );
  }

  public Pose2d getPoseEstimator() {
    return m_poseEstimator.getEstimatedPosition();
  }

   /**
   * Add vision measurement to the pose estimator.
   */
  public void addPoseVisionNew() 
  {
    boolean doRejectUpdate = false;

    try {
      // List of cameras to process
      List<String> cameras = List.of(
          DriveConstants.limelightBack,
          DriveConstants.limelightRight,
          DriveConstants.limelightLeft);

      // Set robot orientation for all cameras
      double robotRotationDegrees = m_poseEstimator.getEstimatedPosition().getRotation().getDegrees();
      for (String camera : cameras) {
        LimelightHelpers.SetRobotOrientation(camera, robotRotationDegrees, 0, 0, 0, 0, 0);
      }

      // Get pose estimates for all cameras
      List<LimelightHelpers.PoseEstimate> poseEstimates = cameras.stream()
          .map(LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2)
          .filter(Objects::nonNull) // Filter out null estimates
          .collect(Collectors.toList());

      // Reject updates if no valid measurements are available
      if (poseEstimates.isEmpty() || doRejectUpdate) 
      {
        doRejectUpdate = true;
      }

      if (!doRejectUpdate) 
      {
        // Select the best measurement based on tag count, distance, and area
        LimelightHelpers.PoseEstimate bestMeasurement = poseEstimates.stream()
            .filter(p -> p.tagCount > 0 && p.avgTagDist < 3.5) // Valid measurements
            .max(Comparator.comparingDouble(p -> p.avgTagArea)) // Select the one with the largest tag area
            .orElse(null);

        if (bestMeasurement != null) {
          // Add the best measurement to the pose estimator
          m_poseEstimator.setVisionMeasurementStdDevs(visionPoseStdDevs);
          m_poseEstimator.addVisionMeasurement(bestMeasurement.pose, bestMeasurement.timestampSeconds);
        }
      }
    } catch (Exception e) {
      // Handle exception as needed
      DriverStation.reportError("Failed to get Limelight botpose", e.getStackTrace());
    }
  }


  /**
   * Get the position of each module
   * 
   * Drive distance in meters and angle in rad
   * 
   * @return SwerveModulePosition array
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  /**
   * Drive speeds m/s of each module
   * and angle in rad
   * 
   * @return states
   */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /**
   * 
   * @return true if the robot is on blue alliance
   */
  public boolean getBlueAlliance() {
    return blueAlliance;
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    frontLeftModule.resetDriveEncoders();
    frontRightModule.resetDriveEncoders();
    backLeftModule.resetDriveEncoders();
    backRightModule.resetDriveEncoders();
  }

  /**
   * Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
   * Path Planner uses
   * 
   * @param chassisSpeeds
   */
  public void drive(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics
        .toSwerveModuleStates(chassisSpeeds);

    this.setModuleStates(swerveModuleStates);
  }


  /**
   * 
   * @return ROBOT RELATIVE chassisSpeeds
   */
  public ChassisSpeeds getChassisSpeeds() {
    // Relative to robot
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   * This method does the same as getRelativeFieldChassisSpeeds() but is more explicit in its intent and calculations.
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
    public ChassisSpeeds getFieldVelocity() 
    {
      // Get the robot-relative chassis speeds (vx, vy, omega)
      ChassisSpeeds robotSpeeds = this.getChassisSpeeds();

      // Rotate the translational velocity (vx, vy) into the field frame using the
      // robot heading. Angular velocity (omega) is the same in both frames.
      var fieldTranslation = new Translation2d( robotSpeeds.vxMetersPerSecond,
                                                robotSpeeds.vyMetersPerSecond)
                                                .rotateBy(gyroRotation2d);

      return new ChassisSpeeds(fieldTranslation.getX(), fieldTranslation.getY(), robotSpeeds.omegaRadiansPerSecond);
    }

  /**
   * 
   * @return robot gyro rotation 2d
   */
  public Rotation2d getGyroRotation2d()
  {
    return gyroRotation2d;
  }

  /**
   * reset the pose Estimator to a new location
   * 
   * @param pose The pose to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(gyroRotation2d, 
        this.getModulePositions(),
        pose);
  }

  /**
   * Stop all modules
   */
  public void stopModules() {
    for (int i = 0; i < modules.length; i++) {
      modules[i].stopMotors();
    }
  }

  /**
   * disable all modules with no BRAKES
   */
  public void disableModules() {
    for (int i = 0; i < modules.length; i++) {
      modules[i].disableMotors();
    }
  }

  /**
   * Draw X with modules 
   */
  public void setX() {
    SwerveModuleState[] xStates = new SwerveModuleState[4];
    xStates[0] = new SwerveModuleState(0, Rotation2d.fromDegrees(45));// 45
    xStates[1] = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));// -45
    xStates[2] = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));// -45
    xStates[3] = new SwerveModuleState(0, Rotation2d.fromDegrees(45));// 45
    setModuleStates(xStates);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {

    // normalize the wheel speeds ;
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kTeleDriveMaxSpeedMetersPerSecond);

    // Output Module States to each one
    for (int i = 0; i < modules.length; i++) {
      modules[i].setDesiredState(desiredStates[i]);

    }
    // Store states for logging in
    states = desiredStates;
  }


  /**
   * Method drive with joystick
   * The use of these parameters as suppliers to dynamically provide speed and
   * field orientation values.
   * Suppliers are especially useful in cases where you need to calculate values
   * dynamically based on factors that can change over time.
   * The Supplier interface gives you a powerful mechanism to make your FRC robot
   * code more flexible and adaptable.
   * Suppliers are incredibly useful in command-based FRC programming because they
   * allow you to:
   * Decouple Logic: You can separate the logic for calculating drive parameters
   * (speeds, orientation)
   * from the actual drive command. This makes your code cleaner and more
   * maintainable.
   * Dynamic Values: You can easily update the speed and orientation values
   * on-the-fly based on real-time conditions,
   * such as sensor feedback or joystick input.
   * 
   * @param xSpdFunction          Speed of the robot in the x direction (forward).
   * @param ySpdFunction          Speed of the robot in the y direction
   *                              (sideways).
   * @param turningSpdFunction    Angular rate of the robot. rad/s
   * @param fieldOrientedFunction Boolean indicating if speeds are relative to the
   *                              field or to therobot.
   * 
   **/

  public void driveFieldOriented(Supplier<Double> xSpdFunction,
      Supplier<Double> ySpdFunction,
      Supplier<Double> turningSpdFunction,
      Supplier<Boolean> joystickButtonFunction) {

    // 1. Get real-time joystick inputs
    double xSpeed = Math.pow(xSpdFunction.get(), 3);
    double ySpeed = Math.pow(ySpdFunction.get(), 3);
    double turningSpeed = turningSpdFunction.get();
    // boolean fieldOriented = fieldOrientedFunction.get();
    boolean joystickButton = joystickButtonFunction.get();

    // 3. Make the driving smoother
    xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * speedMode.getSpeedValue();
    ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * speedMode.getSpeedValue();
    turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond
        * (joystickButton ? MathUtil.clamp(speedMode.getSpeedValue() + 0.2, 0, 0.9) : speedMode.getSpeedValue());

    // 4. Construct desired chassis speeds
    var swerveModuleStates = DriveConstants.kDriveKinematics
        .toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed,
            ySpeed,
            turningSpeed,
            gyroRotation2d));// Do this if fielOrientation is false

    // 6. Output each module states to wheels
    this.setModuleStates(swerveModuleStates);// */
  }

  public void gyroUpdate() {

    if(gyroIOinputs.connected) gyroRotation2d = gyroIOinputs.robotRotation2d;
    else
    {
    /*
     * SwerveModulePosition[] deltas = new SwerveModulePosition[4];
     * for (int i = 0; i < modules.length; i++) {
     * var current = modules[i].getPosition();
     * deltas[i] = new SwerveModulePosition(
     * current.distanceMeters - lastModulePositions[i].distanceMeters,
     * current.angle);
     * lastModulePositions[i] = current;
     * }
     * 
     * /*
     * Twist2d twist =
     * DriveConstants.kDriveKinematics.toTwist2d(getModulePositions());
     * gyroRotation2d = gyroRotation2d.plus(new Rotation2d(twist.dtheta));
     * 
     * // gyroRotation2d =
     */

    ChassisSpeeds speeds = getChassisSpeeds();
    gyroRotation2d = gyroRotation2d.plus(
        new Rotation2d(speeds.omegaRadiansPerSecond * 0.02));
    }
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    if(gyroIOinputs.connected) gyro.resetPosition();
    gyroRotation2d = getBlueAlliance() ? new Rotation2d(Math.PI) : Rotation2d.kZero;
  }


  /**
   * path command drive to the point
   * @param poseSupplier
   * @param maxSpeed
   * @param maxAceleration
   * @return
   */
  public Command pathfindToPose(Supplier<Pose2d> poseSupplier, double maxSpeed, double maxAceleration) {
    PathConstraints telePathConstraints = new PathConstraints(maxSpeed,
        maxAceleration,
        2 * Math.PI,
        3 * Math.PI);
    // return AutoBuilder.pathfindToPose(poseSupplier.get(), telePathConstraints);
    return new DeferredCommand(() -> AutoBuilder.pathfindToPose(poseSupplier.get(),
        telePathConstraints,
        0),
        Set.of(this)).beforeStarting(() -> PathPlannerPath.clearCache());
  }

  /**
   * Follows a path using PathPlanner.
   * This method uses the pathfindThenFollowPath method to load a path from a
   * file.
   * 
   * @param path
   * @return selected path command
   */
  public Command pathfindThenFollow(String path) {
    PathPlannerPath.clearCache();

    try {
      config = PathPlannerConstants.robotConfig;
      PathPlannerPath path2go = PathPlannerPath.fromPathFile(path);
      PathConstraints telePathConstraints = new PathConstraints(1,
          1,
          Math.PI,
          Math.PI);

      return AutoBuilder.pathfindThenFollowPath(path2go, telePathConstraints);
      // new DeferredCommand(()-> AutoBuilder.pathfindThenFollowPath(path2go,
      // telePathConstraints), Set.of(this));

    } catch (Exception e) {
      // Handle exception as needed
      DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
      return Commands.none();
    }
  }

   /**
   * Follows a path using PathPlanner.
   * This method uses the PathPlannerPath.fromPathFile() method to load a path
   * from a file.
   * 
   * @param pathName
   * @return selected path command
   */
  public Command followPath(String path)

  {
    // Clear any existing feedback overrides
    PathPlannerPath.clearCache();
    PPHolonomicDriveController.clearXYFeedbackOverride();
    PPHolonomicDriveController.clearRotationFeedbackOverride();
    try {

      config = RobotConfig.fromGUISettings();
      PathPlannerPath path2go = PathPlannerPath.fromPathFile(path);

      return AutoBuilder.followPath(path2go);

    } catch (Exception e) {
      // Handle exception as needed
      DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
      return Commands.none();
    }
  }

  /**
   * Ends the current path following operation.
   */
  public void endPath() {
    // PPHolonomicDriveController.overrideXYFeedback(() -> 0.0, () -> 0.0);
    // Calculate feedback from your custom PID controller

    // (() -> this.getPoseEstimator().getX(), ()-> this.getPoseEstimator().getY());
    PPHolonomicDriveController.overrideRotationFeedback(() -> this.getPoseEstimator().getRotation().getRadians());
    PPHolonomicDriveController.clearXYFeedbackOverride();
    PPHolonomicDriveController.clearRotationFeedbackOverride();

    PathPlannerPath.clearCache();
  }
}


// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package br.megazord7563.Robot;

import com.revrobotics.util.StatusLogger;

import br.megazord7563.Robot.Constants.RobotConstants;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private double autoStart;
  private boolean autoMessagePrinted;

  // private ControllerInputsAutoLogged controllerInputs = new ControllerInputsAutoLogged();

  double lowBatteryVoltage = 11.0;
  double BatteryVoltage;

  private final RobotContainer m_robotContainer;


  private final Alert batteryAlert =
      new Alert("Battery voltage is very low, please replace the battery.", AlertType.kWarning);
  private Alert joystickAlert =
      new Alert(
          "DriverJoystick on port 0 is not connected. Please connect the joystick and restart the robot.",
          Alert.AlertType.kInfo);

  private final Timer lowBatteryTimer = new Timer();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Reset alert timers
    lowBatteryTimer.restart();

    Logger.recordMetadata("RobotVersion", "Astro-2026");
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("RobotMode", RobotConstants.robotMode.toString());
    Logger.recordMetadata("BuildVersion", BuildConstants.VERSION);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitRevision", Integer.toString(BuildConstants.GIT_REVISION));

    String logPath = "/U/logs/";
    String logName;

    if (DriverStation.isFMSAttached()) {
      Logger.recordMetadata("Mode", "FMS");

      Logger.recordMetadata("Event", DriverStation.getEventName());
      Logger.recordMetadata("MatchType", DriverStation.getMatchType().toString());
      Logger.recordMetadata("MatchNumber", Integer.toString(DriverStation.getMatchNumber()));
      Logger.recordMetadata("Alliance", DriverStation.getAlliance().toString());

      logName =
          String.format(
              "%s_%d_%s.wpilog",
              DriverStation.getMatchType().toString(),
              DriverStation.getMatchNumber(),
              DriverStation.getAlliance().toString());

    } else {
      Logger.recordMetadata("Mode", "TEST");

      String timestamp = java.time.LocalDateTime.now().toString().replace(":", "-");

      Logger.recordMetadata("RunTime", timestamp);

      logName = "TEST_" + timestamp + ".wpilog";
    }

    if (new java.io.File("/U").exists()) {
      logPath = "/U/logs/";
    } else {
      logPath = "/home/lvuser/logs/";
    }

    switch (RobotConstants.robotMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter(logPath + logName));
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        Logger.addDataReceiver(new WPILOGWriter());
        break;
      default:
        break;
    }

    //Logger.registerURCL(URCL.startExternal());
    StatusLogger.disableAutoLogging(); // Disable REVLib's built-in logging

    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
    // be added.
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    BatteryVoltage = RobotController.getBatteryVoltage();
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Match time", DriverStation.getMatchTime());
    SmartDashboard.putNumber("Batery Voltage", BatteryVoltage);
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());

    // Print auto duration
    if (m_autonomousCommand != null) {
      if (!m_autonomousCommand.isScheduled() && !autoMessagePrinted) {
        if (DriverStation.isAutonomousEnabled()) {
          System.out.printf(
              "*** Auto finished in %.2f secs ***%n", Timer.getTimestamp() - autoStart);
        } else {
          System.out.printf(
              "*** Auto cancelled in %.2f secs ***%n", Timer.getTimestamp() - autoStart);
        }
        autoMessagePrinted = true;
      }
    }

    // XboxControllerIO.updateInputs(driverController, controllerInputs);

    Logger.recordOutput("Alerts/BatteryAlert", batteryAlert.get());
    Logger.recordOutput("Alerts/JoystickAlert", joystickAlert.get());
    Logger.recordOutput("Robot/BatteryVoltage", BatteryVoltage);
    Logger.recordOutput("Robot/MatchTime", DriverStation.getMatchTime());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    autoStart = Timer.getTimestamp();
    autoMessagePrinted = false;

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    lowBatteryTimer.reset();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    /* Battery Alert */
    if (BatteryVoltage > 0
        && BatteryVoltage <= lowBatteryVoltage
        && lowBatteryTimer.hasElapsed(3.0)) {
      batteryAlert.set(true);
    }

    if (BatteryVoltage > lowBatteryVoltage) {
      lowBatteryTimer.reset();
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}

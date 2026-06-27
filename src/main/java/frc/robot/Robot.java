// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import static edu.wpi.first.units.Units.Rotations;

/** This is a demo program showing how to use Mecanum control with the MecanumDrive class. */
public class Robot extends LoggedRobot {

  private static final int kFrontLeftChannel = 2;
  private static final int kRearLeftChannel = 1;
  private static final int kFrontRightChannel = 3;
  private static final int kRearRightChannel = 4;
  private static final int intakeChannel = 5;

  private static final int kJoystickChannel = 0;
  
  private final MecanumDrive db;
  private final XboxController gamepad;
  
  private final SparkMax frontLeft;
  private final SparkMax rearLeft;
  private final SparkMax frontRight;
  private final SparkMax rearRight;
  private final SparkMax intake;
  
  /** Called once at the beginning of the robot program. */
  public Robot() {
    Logger.addDataReceiver(new NT4Publisher());
    Logger.addDataReceiver(new WPILOGWriter());
    Logger.recordMetadata("ProjectName", "Electron");
    Logger.start();
    
    frontLeft = new SparkMax(kFrontLeftChannel, MotorType.kBrushed);
    rearLeft = new SparkMax(kRearLeftChannel, MotorType.kBrushed);
    frontRight = new SparkMax(kFrontRightChannel, MotorType.kBrushed);
    rearRight = new SparkMax(kRearRightChannel, MotorType.kBrushed);
    intake = new SparkMax(intakeChannel, MotorType.kBrushed);
    
    // Config
    SparkMaxConfig mainConfig = new SparkMaxConfig();
    
    mainConfig
      .smartCurrentLimit(60)
      .idleMode(IdleMode.kBrake)
      .encoder.countsPerRevolution(538);
    
    SparkMaxConfig invertedConfig = new SparkMaxConfig();
    
    invertedConfig
      .apply(mainConfig)
      .inverted(true);
    
    frontRight.configure(mainConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rearRight.configure(mainConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);

    // Invert the left side motors.
    frontLeft.configure(invertedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rearLeft.configure(invertedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    intake.configure(mainConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    db = new MecanumDrive(frontLeft::set, rearLeft::set, frontRight::set, rearRight::set);
    
    gamepad = new XboxController(kJoystickChannel);
  }
  
  @Override
  public void robotPeriodic() {
    Logger.recordOutput("FrontLeft", Rotations.of(frontLeft.getEncoder().getPosition()));
    Logger.recordOutput("RearLeft", Rotations.of(rearLeft.getEncoder().getPosition()));
    Logger.recordOutput("FrontRight", Rotations.of(frontRight.getEncoder().getPosition()));
    Logger.recordOutput("RearRight", Rotations.of(rearRight.getEncoder().getPosition()));
  }
  
  @Override
  public void teleopPeriodic() {
    // Use the joystick Y axis for forward movement, X axis for lateral
    // movement, and Z axis for rotation.
    db.driveCartesian(-gamepad.getLeftY(), -gamepad.getLeftX(), -gamepad.getRightX());

    boolean isLeftBumperPressed = gamepad.getLeftBumperButton();
    boolean isRightBumperPressed = gamepad.getRightBumperButton();

    if (isLeftBumperPressed == true){
      intake.set(0.5);
    } else if (isRightBumperPressed == true){
      intake.set(-0.5);
    } else {
      intake.set(0);
    }
  }

  @Override
  public void autonomousInit() {
    System.out.println("Auto START!!!");
  }

  @Override
  public void autonomousPeriodic() {

  }
  
}
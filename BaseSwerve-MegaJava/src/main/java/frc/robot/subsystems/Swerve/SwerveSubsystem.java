// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Pigeon2FeaturesConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
  /******************
   * Swerve Modules *
   ******************/

  /** FrontLeftModule **/
  private final SwerveModule frontLeft = new SwerveModule(
      DriveConstants.kFrontLeftDriveMotorPort,
      DriveConstants.kFrontLeftTurningMotorPort,
      DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
      DriveConstants.angleOffsetFLTurning,
      DriveConstants.kFrontLeftChassisAngularOffset);
  /** FrontRightModule **/
  private final SwerveModule frontRight = new SwerveModule(
      DriveConstants.kFrontRightDriveMotorPort,
      DriveConstants.kFrontRightTurningMotorPort,
      DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
      DriveConstants.angleOffsetFRTurning,
      DriveConstants.kFrontRightChassisAngularOffset);
  /** BackLeftModule **/
  private final SwerveModule backLeft = new SwerveModule(
      DriveConstants.kBackLeftDriveMotorPort,
      DriveConstants.kBackLeftTurningMotorPort,
      DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
      DriveConstants.angleOffsetBLTurning,
      DriveConstants.kBackLeftChassisAngularOffset);
  /** BackRightModule **/
  private final SwerveModule backRight = new SwerveModule(
      DriveConstants.kBackRightDriveMotorPort,
      DriveConstants.kBackRightTurningMotorPort,
      DriveConstants.kBackRightDriveAbsoluteEncoderPort,
      DriveConstants.angleOffsetBRTurning,
      DriveConstants.kBackRightChassisAngularOffset);

  private SwerveModule[] swerveModules = { frontLeft, frontRight, backLeft, backRight };

  private final SlewRateLimiter xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
  private final SlewRateLimiter turningLimiter= new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

  /** Create Pigeon */
  private final Pigeon2 pigeon = new Pigeon2(DriveConstants.kPigeonPort, "rio");

  public SwerveSubsystem() 
  {
    initializePigeon();
  }

  private void initializePigeon() 
  {
    // Factory Reset
    pigeon.clearStickyFaults();
    pigeon.getConfigurator().apply(new Pigeon2Configuration());

    // Create a new pigeon configuration
    Pigeon2Configuration pigeonConfig = new Pigeon2Configuration();

    pigeonConfig.withPigeon2Features(new Pigeon2FeaturesConfigs()
        .withDisableNoMotionCalibration(false)
        .withDisableTemperatureCompensation(false)
        .withEnableCompass(false)); 

     StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = pigeon.getConfigurator().apply(pigeonConfig);
      if (status.isOK())
        break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply gyro configs, error code: " + status.toString());
    }
  }

  @Override
  public void periodic() 
  {

  }

  public void setDesiredStates(SwerveModuleState[] desiredStates) 
  {
    for (int i = 0; i < swerveModules.length; i++) 
    {
      swerveModules[i].setModuleDisiredState(desiredStates[i]);
    }
  }

  public void drive(ChassisSpeeds chassisSpeeds)
  {
    SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    this.setDesiredStates(swerveModuleStates);
  }

  public void driveFieldOperated(Supplier<Double> xSpdFunction,
                                 Supplier<Double> ySpdFunction,
                                 Supplier<Double> turningSpdFunction)
  {
    double throttle = 0.1;

    double vx = xSpdFunction.get();
    double vy = ySpdFunction.get();
    double omega = turningSpdFunction.get();

    vx = xLimiter.calculate(vx) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * throttle;
    vy = yLimiter.calculate(vy) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * throttle;
    omega = turningLimiter.calculate(omega) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond *throttle;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            vx,
            vy,
            omega,
            pigeon.getRotation2d()
        )
    );

    this.setDesiredStates(swerveModuleStates);
  }

  public void setX()
  {
    SwerveModuleState[] desiredStates = new SwerveModuleState[4];

    desiredStates[0] = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
    desiredStates[1] = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
    desiredStates[2] = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
    desiredStates[3] = new SwerveModuleState(0, Rotation2d.fromDegrees(45));

    setDesiredStates(desiredStates);
  }

  public void resetGyro() 
  {
    pigeon.setYaw(0);
  }
}

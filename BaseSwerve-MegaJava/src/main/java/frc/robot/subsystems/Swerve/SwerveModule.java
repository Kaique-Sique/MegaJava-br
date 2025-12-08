// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Utils.Conversions;

public class SwerveModule extends SubsystemBase {
  private final TalonFX m_turningMotor;
  private final TalonFX m_drivingMotor;

  private final CANcoder m_absoluteEncoder;

  private final VelocityVoltage m_VelocityVoltage = new VelocityVoltage(0).withSlot(0);
  private final PositionVoltage mPositionVoltage = new PositionVoltage(0).withSlot(0);

  Rotation2d absoluteEncoderOffset;
  double chassisAngularOffset;

  SwerveModuleState desiredState;

  public SwerveModule(int drivingMotorID,
                      int turningMotorID,
                      int absoluteEncoderID,
                      Rotation2d absoluteEncoderOffset,
                      double chassisAngularOffset) 
                      {
    this.m_drivingMotor = new TalonFX(drivingMotorID);
    this.m_turningMotor = new TalonFX(turningMotorID);

    this.m_absoluteEncoder = new CANcoder(absoluteEncoderID);

    this.absoluteEncoderOffset = absoluteEncoderOffset;
    this.chassisAngularOffset = chassisAngularOffset;

    initilizeTurningMotor();
    initilizeDriveMotor();

    initializeCanCoder();

    resetToAbsolute();

    resetDriveEncoders();
  }

  public SwerveModulePosition getModulePosition()
  {
    SwerveModulePosition modulePosition = 
                new SwerveModulePosition(drivePositionMetters(), 
                                          new Rotation2d(this.getTurningPoseRad()-chassisAngularOffset));
    
    return modulePosition;
  }

  public SwerveModuleState getModuleState()
  {
    SwerveModuleState moduleState = 
                new SwerveModuleState(
                                      getDriveMPS(),
                                      new Rotation2d(
                                        this.getTurningPoseRad()-chassisAngularOffset
                                      )
                                    );

    return moduleState;
  }

  public void setModuleDisiredState(SwerveModuleState desiredState)
  {
    SwerveModuleState correctedState = new SwerveModuleState();
    correctedState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

    correctedState.optimize(new Rotation2d(this.getTurningPoseRad()));

    correctedState.cosineScale(new Rotation2d(this.getTurningPoseRad()));

    double driveRPS = Conversions.MPSToRPS(correctedState.speedMetersPerSecond,
                                            ModuleConstants.kWheelCircumferenceMeters,
                                            ModuleConstants.kDriveMotorGearRatio);

    //active Drive Motor
    m_drivingMotor.setControl(m_VelocityVoltage.withVelocity(driveRPS));
    //active Turn Motor
    m_turningMotor.setControl(mPositionVoltage.withPosition(correctedState.angle.getRotations()));


    this.desiredState = correctedState;
  }

  private void resetDriveEncoders() 
  {
    m_drivingMotor.setPosition(0);
  }

  private void resetToAbsolute() {
    double absoluteRot = getAbsuluteEncoder().getRotations();
    m_turningMotor.setPosition(absoluteRot);
  }

  private void initilizeDriveMotor() {
    // Factore Defaults
    m_drivingMotor.clearStickyFaults();
    m_drivingMotor.getConfigurator().apply(new TalonFXConfiguration());

    m_drivingMotor.setNeutralMode(NeutralModeValue.Brake);

    // Create a new configuration object
    TalonFXConfiguration drivingMotorConfig = new TalonFXConfiguration();

    // Setting current limits
    CurrentLimitsConfigs mCurrentLimitsConfigs = drivingMotorConfig.CurrentLimits;
    mCurrentLimitsConfigs.StatorCurrentLimit = ModuleConstants.driveCurrentThreshold;
    mCurrentLimitsConfigs.StatorCurrentLimitEnable = true;

    // Feedback encoder settings
    drivingMotorConfig.Feedback.SensorToMechanismRatio = 1 / ModuleConstants.kDriveMotorGearRatio;
    drivingMotorConfig.ClosedLoopGeneral.ContinuousWrap = true;

    // Neutral Mode Value
    drivingMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // setting pid consts
    drivingMotorConfig.Slot0.kS = ModuleConstants.driveKS;
    drivingMotorConfig.Slot0.kV = ModuleConstants.driveKV;
    drivingMotorConfig.Slot0.kP = ModuleConstants.kPdriving;
    drivingMotorConfig.Slot0.kI = ModuleConstants.kIdriving;
    drivingMotorConfig.Slot0.kD = ModuleConstants.kDdriving;

    StatusCode status = StatusCode.StatusCodeNotInitialized; // init status code
    /* Retry config apply up to 5 times, report if failure */
    for (int i = 0; i < 5; ++i) {
      status = m_drivingMotor.getConfigurator().apply(drivingMotorConfig);
      if (status.isOK())
        break; // if apply is ok break the loop
    }
    if (!status.isOK()) { // if not ok print the error
      System.out.println("Could not apply swerve module configs, error code: " + status.toString());
    }
  }

  private void initilizeTurningMotor() {
    // Factore Defaults
    m_turningMotor.clearStickyFaults();
    m_turningMotor.getConfigurator().apply(new TalonFXConfiguration());

    m_turningMotor.setNeutralMode(NeutralModeValue.Coast);

    // Create a new configuration object
    TalonFXConfiguration turningMotorConfig = new TalonFXConfiguration();

    CurrentLimitsConfigs mCurrentLimitsConfigs = turningMotorConfig.CurrentLimits;
    mCurrentLimitsConfigs.StatorCurrentLimit = ModuleConstants.kTurningMotorCurrentLimit;
    mCurrentLimitsConfigs.StatorCurrentLimitEnable = true;

    // Feedback encoders settings
    turningMotorConfig.Feedback.SensorToMechanismRatio = 1 / ModuleConstants.kTurningMotorGearRatio;
    turningMotorConfig.ClosedLoopGeneral.ContinuousWrap = true;

    turningMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // put pid values constants
    turningMotorConfig.Slot0.kP = ModuleConstants.kPTurning;
    turningMotorConfig.Slot0.kD = ModuleConstants.kDTurning;
    turningMotorConfig.Slot0.kI = ModuleConstants.kITurning;
    turningMotorConfig.Slot0.kS = ModuleConstants.kTurningFF;

    StatusCode status = StatusCode.StatusCodeNotInitialized; // init status code
    /* Retry config apply up to 5 times, report if failure */
    for (int i = 0; i < 5; ++i) {
      status = m_turningMotor.getConfigurator().apply(turningMotorConfig);
      if (status.isOK())
        break; // if apply is ok break the loop
    }
    if (!status.isOK()) { // if not ok print the error
      System.out.println("Could not apply swerve module configs, error code: " + status.toString());
    }
  }

  private void initializeCanCoder() {
    // Factory Default
    m_absoluteEncoder.clearStickyFaults();
    m_absoluteEncoder.getConfigurator().apply(new CANcoderConfiguration());

    // Create a new configuration object
    CANcoderConfiguration mCaNcoderConfiguration = new CANcoderConfiguration();

    mCaNcoderConfiguration.withMagnetSensor(
        new MagnetSensorConfigs()
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
            .withMagnetOffset(0));

    m_absoluteEncoder.getConfigurator().apply(mCaNcoderConfiguration);
  }

  @Override
  public void periodic() {
  }

  public Rotation2d getAbsuluteEncoder() {
    return Rotation2d.fromRotations(m_absoluteEncoder.getPosition().getValueAsDouble());
  }

  public double getTurningPoseRad()
  {
    double pose = m_turningMotor.getPosition().getValueAsDouble();
    return Units.rotationsToRadians(pose);
  }

  public double getDriveMPS()
  {
    double speed = m_drivingMotor.getRotorVelocity().getValueAsDouble();
    return Conversions.RPSToMPS(speed, ModuleConstants.kWheelCircumferenceMeters);
  }

  public double drivePositionMetters()
  {
    double pose = m_drivingMotor.getPosition().getValueAsDouble();
    return Conversions.rotationsToMeters(pose, ModuleConstants.kWheelCircumferenceMeters);
  }
}

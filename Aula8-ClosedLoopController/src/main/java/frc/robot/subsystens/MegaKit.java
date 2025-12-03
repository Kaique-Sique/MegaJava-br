// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystens;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.hal.simulation.DIODataJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.motorKitConstants;

public class MegaKit extends SubsystemBase {
  private SparkMax motorKit;
  private RelativeEncoder motorEncoder;
  private SparkClosedLoopController motorClosedLoopController;

  public MegaKit() {
    motorKit = new SparkMax(motorKitConstants.kMotorID, MotorType.kBrushless);

    motorEncoder = motorKit.getEncoder();
    motorClosedLoopController = motorKit.getClosedLoopController();

    initializeMotor();
  }

  private void initializeMotor() 
  {
    SparkMaxConfig globalConfig = new SparkMaxConfig();

    globalConfig
      .smartCurrentLimit(motorKitConstants.kCurrentLimits)
      .idleMode(IdleMode.kBrake);

    globalConfig.encoder
      .positionConversionFactor(motorKitConstants.kMotorGearBox)
      .velocityConversionFactor(motorKitConstants.kMotorGearBox);

    globalConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(0.2)
      .i(0.0)
      .d(0.0);

    motorKit.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() 
  {
    SmartDashboard.putNumber("MegaKit/Encoder", getEncoderValue());
  }

  public void runMotor(double out)
  {
    motorKit.set(out);
  }

  public void stopMotor()
  {
    motorKit.set(0);
  }
  
  public double getEncoderValue()
  {
    return motorEncoder.getPosition();
  }

  public void setPositionPID(double position)
  {
    motorClosedLoopController.setReference(position, ControlType.kPosition);
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystens;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.motorKitConstants;

public class MegaKit extends SubsystemBase {
  private SparkMax motorKit;
  private RelativeEncoder motorEncoder;

  public MegaKit() {
    motorKit = new SparkMax(motorKitConstants.kMotorID, MotorType.kBrushless);

    motorEncoder = motorKit.getEncoder();

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

      motorKit.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runMotor(double out)
  {
    motorKit.set(out);
  }

  public void stopMotor()
  {
    motorKit.set(0);
  }
  
}

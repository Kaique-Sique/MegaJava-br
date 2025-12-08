// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

  //Set dos motores do DriveTrain

  //Victor
  //MotorController frontLeft= new WPI_VictorSPX(1);
  //MotorController backLeft= new WPI_VictorSPX(2);
  //MotorController frontRight= new WPI_VictorSPX(3);
  //MotorController backRight= new WPI_VictorSPX(4);

  //CANSparkMAX
  CANSparkMax frontLeft = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax backLeft = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax frontRight = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax backRight = new CANSparkMax(4, MotorType.kBrushless);

  //Criante o objeto de controle
  private Joystick joy1 = new Joystick(0);

  //Criando o encoder
  private Encoder encoder=new Encoder(0,1,true,EncodingType.k4X);

  private DifferentialDrive drive;

  private final double kDriveTick2Meter = (1.0/128*6*Math.PI/12)/3.280;


  public Robot(){

  }

  @Override
  public void robotInit() {
    frontLeft.setInverted(true);
    backLeft.setInverted(true);

    MotorControllerGroup leftMotors = new MotorControllerGroup(frontLeft, backLeft);
    MotorControllerGroup rightMotors = new MotorControllerGroup(frontRight, backRight);

    leftMotors.setInverted(true);
    rightMotors.setInverted(false);

  }

  @Override
  public void autonomousInit() {

  }

  final double kP =0.5;
  final double kI = 5;
  final double kD = 0.05;
  final double iLimit =1;

  double setpoint =0;
  double errorSum =0;
  double lastTimestamp = 0;
  double lastError = 0;

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void robotPeriodic() {
  SmartDashboard.putNumber("Encoder Value", encoder.get()*kDriveTick2Meter);
  }

  @Override
  public void teleopInit() {
    encoder.reset();
    errorSum = 0;
    lastError = 0;
    lastTimestamp = Timer.getFPGATimestamp();
  }

  @Override
  public void teleopPeriodic() {

  //Le o comando do JoyStick
  if(joy1.getRawButton(1)){
    setpoint =4;
    
  }else if(joy1.getRawButton(2)){
    setpoint=0;
  }
    

  //Pegando a posição do Encoder e convertendo para pes
  double encoderPosition = encoder.get()* kDriveTick2Meter;
  //Calculando o erro
  double error = setpoint - encoderPosition;
  double dt = Timer.getFPGATimestamp() - lastTimestamp;

  if(Math.abs(error)<iLimit){
    errorSum += error * dt;
  }

  double errorRate = (error - lastError)/dt;

  errorSum += error *dt;
  double outputSpeed = error * kP * error + kI * errorSum + kD *errorRate;

  drive.tankDrive(outputSpeed, -outputSpeed);

  lastTimestamp =Timer.getFPGATimestamp();
  lastError = error;
  }
  //outptSpeed para os motores
  /* 
  frontLeft.set(outputSpeed);
  backLeft.set(outputSpeed);
  frontRight.set(-outputSpeed);
  backRight.set(-outputSpeed);
    */
  

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}

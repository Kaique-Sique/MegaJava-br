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
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */


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



  private final double kDriveTick2Feet = 1.0/128*6*Math.PI/12;

  //Criando o Encoder
  private Encoder encoder=new Encoder(0,1,true,EncodingType.k4X);


  @Override
  public void robotInit() {


  }

  @Override
  public void autonomousInit() {
    encoder.reset();
    errorSum =0;
    lastError =0;
    lastTimestamp = Timer.getFPGATimestamp();
  }

  final double kP =0.5;
  final double kI =0.5;
  final double kD = 0.1;
  final double iLimit =1;

  double setpoint =0;
  double errorSum =0;
  double lastTimestamp =0;
  double lastError =0;

  @Override
  public void autonomousPeriodic() {
    //Le o comando do JoyStick
    if(joy1.getRawButton(1)){
      setpoint =10;
      
    }else if(joy1.getRawButton(2)){
      setpoint=0;
    }

    //Pegando a posição do Encoder e convertendo para pes
    double encoderPosition = encoder.get()* kDriveTick2Feet;

    //Calculos
    double error = setpoint - encoderPosition;
    double dt = Timer.getFPGATimestamp() - lastTimestamp;

    //Colocando uma zona para ativar o Ki
    if(Math.abs(error)<iLimit){
      errorSum += error *dt;
    }

    double errorRate = (error -lastError)/dt;

    double outputSpeed = error * kP + kI * errorSum + kD * errorRate;

    //outptSpeed para os motores

    //Verificar se esta certo o set
    frontLeft.set(outputSpeed);
    backLeft.set(outputSpeed);
    frontRight.set(-outputSpeed);
    backRight.set(-outputSpeed);

    //Atualiza as variaveis
    lastTimestamp = Timer.getFPGATimestamp();
    lastError = error;
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Encoder Value", encoder.get()*kDriveTick2Feet);
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

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

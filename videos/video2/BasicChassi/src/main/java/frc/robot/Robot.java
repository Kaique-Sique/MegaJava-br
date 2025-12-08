
package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

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
  MotorController frontLeft= new WPI_VictorSPX(1);
  MotorController backLeft= new WPI_VictorSPX(2);
  MotorController frontRight= new WPI_VictorSPX(3);
  MotorController backRight= new WPI_VictorSPX(4);

  //CANSparkMAX
  //CANSparkMax frontLeft = new CANSparkMax(1, MotorType.kBrushless);
  //CANSparkMax backLeft = new CANSparkMax(2, MotorType.kBrushless);
  //CANSparkMax frontRight = new CANSparkMax(3, MotorType.kBrushless);
  //CANSparkMax backRight = new CANSparkMax(4, MotorType.kBrushless);

  //Criante o objeto de controle
  private Joystick joy1 = new Joystick(0);

  //Criando o encoder
  private Encoder encoder=new Encoder(0,1,true,EncodingType.k4X);

  private DifferentialDrive drive;

  private final double kDriveTick2Meter = 2.2455;//(1.0/128*6*Math.PI/12)/3.280;


  public Robot(){

  }

  @Override
  public void robotInit() {
    frontLeft.setInverted(true);
    backLeft.setInverted(true);

    MotorControllerGroup leftMotors = new MotorControllerGroup(frontLeft, backLeft);
    MotorControllerGroup rightMotors = new MotorControllerGroup(frontRight, backRight);

    drive =  new DifferentialDrive(leftMotors, rightMotors);

  }
  
  @Override
  public void robotPeriodic() {}

 
  @Override
  public void autonomousInit() {
    startTime = Timer.getFPGATimestamp();
  }


  @Override
  public void autonomousPeriodic() {

        double time = Timer.getFPGATimestamp();
      
        if(time - startTime < 3){      
        frontLeft.set(0.6);
        backLeft.set(0.6);
        frontRight.set(-0.6);
        backRight.set(-0.6);
        }
        else{
        frontLeft.set(0);
        backLeft.set(0);
        frontRight.set(0);
        backRight.set(0);
        }
    }

  @Override
  public void teleopInit() {}

 
  @Override
  public void teleopPeriodic() {
    double speed = xBoxController.getRawAxis(1);
    double turn = xBoxController.getRawAxis(2);

    double left = speed + turn;
    double right = speed - turn;
    
    frontLeft.set(left);
    backLeft.set(left);
    frontRight.set(-right);
    backRight.set(-right);

    drive.arcadeDrive(speed, turn);

  }
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

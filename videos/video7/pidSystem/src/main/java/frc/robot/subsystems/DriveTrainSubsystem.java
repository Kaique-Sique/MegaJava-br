package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
//import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;

public class DriveTrainSubsystem extends SubsystemBase{

    //Initialize Motors

    //Victor
    MotorController frontLeft;
    MotorController backLeft;
    MotorController frontRight;
    MotorController backRight;

    
    //CANSparkMax frontLeft;
    //CANSparkMax backLeft;
    //CANSparkMax frontRight;
    //CANSparkMax backRight;

    MotorControllerGroup leftMotors;
    MotorControllerGroup rightMotors;

    DifferentialDrive drive;
    double speed;

    public DriveTrainSubsystem(){

    //Assignment 
    
    //Motors

    //Victor
    frontLeft = new WPI_VictorSPX(OperatorConstants.frontLeft);
    backLeft = new WPI_VictorSPX(OperatorConstants.backLeft);
    
    frontRight = new WPI_VictorSPX(OperatorConstants.frontRight);
    backRight = new WPI_VictorSPX(OperatorConstants.backRight);

    //NEO
    
    //frontLeft = new CANSparkMax(1, MotorType.kBrushless);
    //backLeft = new CANSparkMax(2, MotorType.kBrushless);
    
    //frontRight = new CANSparkMax(3, MotorType.kBrushless);
    //backRight = new CANSparkMax(4, MotorType.kBrushless);

    //Inverting some motors so that gears don't run against each other

    frontLeft.setInverted(false);
    backLeft.setInverted(false);

    frontRight.setInverted(false);
    backRight.setInverted(false);

    
    //Only Spark
    //frontLeft.setIdleMode(IdleMode.kBrake); 
    //backLeft.setIdleMode(IdleMode.kBrake);
    //frontRight.setIdleMode(IdleMode.kBrake);
    //backRight.setIdleMode(IdleMode.kBrake);


    // Putting motors to follow each other

    leftMotors = new MotorControllerGroup(frontLeft, backLeft);
    rightMotors = new MotorControllerGroup(frontRight, backRight);

    drive = new DifferentialDrive(leftMotors, rightMotors);

    }

    public void driveWithJoystick(Joystick joystick, double speed) {

        drive.arcadeDrive(joystick.getRawAxis(OperatorConstants.xBoxLeftYaxis)*speed, 
                            joystick.getRawAxis(OperatorConstants.xBoxLeftXaxis)*speed);
        this.speed = speed;

    }

    public void driveForward(double speed){
        drive.tankDrive(speed, speed);
    }

    public void driveBack(double speed){
        drive.tankDrive(-speed, -speed);
    }

    public void driveLeft(double speed){
        drive.tankDrive(-speed, speed);
    }
    
    public void driveRight(double speed){
        drive.tankDrive(speed, -speed);
    }

    public void stop () {
        drive.stopMotor();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Drive Speed Value", speed);
    }
}
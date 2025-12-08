package frc.robot.subsystems.Intake;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;

public class ElevatorSubsystem extends SubsystemBase{

    //Initialize Motors

    //Victor
    MotorController leftMotor;
    MotorController rightMotor;

    Encoder elevatorEncoder;

    

    MotorControllerGroup elevatorMotors;

    PIDController pidElevator;
    double kP = 0.03;
    
    //CANSparkMax leftMotor;
    //CANSparkMax rightMotor;
    //RelativeEncoder elevatorEncoderSpark;
    //SparkMaxPIDController pidElevatorSpark;

    double speed;
    double output;
    double setpoint;

    public ElevatorSubsystem(){

    //Assignment 
    
    //Motors

    //Victor
    leftMotor = new WPI_VictorSPX(OperatorConstants.elevatorLeft);
    rightMotor =  new WPI_VictorSPX(OperatorConstants.elevatorRight);
    
    elevatorEncoder = new Encoder(OperatorConstants.elevatorChannelA, OperatorConstants.elevatorChannelB,
                             false, EncodingType.k4X);  

    //Spark
    
    //leftMotor = new CANSparkMax(OperatorConstants.elevatorLeft, MotorType.kBrushless);
    //rightMotor = new CANSparkMax(OperatorConstants.elevatorRight, MotorType.kBrushless);

    //elevatorEncoderSpark = leftMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

    //Inverting some motors so that gears don't run against each other

    leftMotor.setInverted(false);
    rightMotor.setInverted(false);



    // Putting motors to follow each other

    elevatorMotors = new MotorControllerGroup(leftMotor,rightMotor);

    pidElevator = new PIDController(kP, 0,0);

    //pidElevatorSpark = leftMotor.getPIDController();
    //pidElevatorSpark.setP(kP);
    //pidElevatorSpark.setFeedbackDevice(elevatorEncoderSpark);

    }

    public void elevatorWithJoystick(Joystick joystick, double speed) {

        elevatorMotors.set(joystick.getRawAxis(OperatorConstants.operatorElevatorAxis)*speed);
        this.speed = speed;

    }

    public void elevatorSetpoint(double elevatorSetpoint) {
        setpoint = elevatorSetpoint;
        output = pidElevator.calculate(elevatorEncoder.get(), elevatorSetpoint);
        elevatorMotors.set(output*speed);

        //pidElevatorSpark.setReference(elevatorSetpoint,CANSparkMax.ControlType.kVelocity);
    }

    public void stop () {
        elevatorMotors.stopMotor();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Elevator Speed", speed);
        SmartDashboard.putNumber("Elevator Setpoint", setpoint);
    }
}

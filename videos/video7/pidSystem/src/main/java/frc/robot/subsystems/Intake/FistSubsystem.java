package frc.robot.subsystems.Intake;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
//import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;

public class FistSubsystem extends SubsystemBase{

    //Initialize Motors

    //Victor
    MotorController fistMotor;

    PIDController pidFist;
    double kP = 0.03;
    //CANSparkMax fistMotor;

    double speed;

    Encoder fistEncoder;

    double setpoint;

    double output;

    public FistSubsystem(){

    //Assignment 
    
    //Motors

    //Victor
    fistMotor = new WPI_VictorSPX(OperatorConstants.fistMotor);

    //NEO
    
    //fistMotor = new CANSparkMax(OperatorConstants.fistMotor, MotorType.kBrushless);


    fistEncoder = new Encoder(OperatorConstants.fistChannelA, OperatorConstants.fistChannelB,
                             false, EncodingType.k4X); 

    //Inverting some motors so that gears don't run against each other

    fistMotor.setInverted(false);

    pidFist = new PIDController(kP, 0,0);

    }

    public void fistWithJoystick(Joystick joystick, double speed) {

        fistMotor.set(joystick.getRawAxis(OperatorConstants.operatorFistAxis)*speed);
        this.speed = speed;

    }

    public void fistSetpoint(double fistSetpoint) {
        setpoint = fistSetpoint;
        output = pidFist.calculate(fistEncoder.get(), fistSetpoint);
        fistMotor.set(output*speed);
    }

    public void stop () {
        fistMotor.stopMotor();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Fist Speed", speed);
        SmartDashboard.putNumber("Fist Setpoint", setpoint);
    }
}
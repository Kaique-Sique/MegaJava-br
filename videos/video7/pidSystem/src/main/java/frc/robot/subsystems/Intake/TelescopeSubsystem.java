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

public class TelescopeSubsystem extends SubsystemBase{

    //Initialize Motors

    //Victor
    MotorController telescopeMotor;

    //CANSparkMax fistMotor;

    Encoder telescopeEncoder;

    PIDController pidTelescope;
    double kP = 0.03;

    double speed;

    double setpoint;

    double output;

    public TelescopeSubsystem(){

    //Assignment 
    
    //Motors

    //Victor
    telescopeMotor = new WPI_VictorSPX(OperatorConstants.telescopeMotor);
    
    //NEO
    
    //telescopeMotor = new CANSparkMax(OperatorConstants.telescopeMotor, MotorType.kBrushless);

    telescopeEncoder = new Encoder(OperatorConstants.telescopeChannelA, OperatorConstants.telescopeChannelB,
                             false, EncodingType.k4X); 

    //Inverting some motors so that gears don't run against each other

    telescopeMotor.setInverted(false);

    pidTelescope = new PIDController(kP,0,0);

    }

    public void telescopeWithJoystick(Joystick joystick, double speed) {
        telescopeMotor.set(joystick.getRawAxis(OperatorConstants.operatorTelescopeAxis)*speed);
        this.speed = speed;
    }

    public void telescopeSetpoint(double telescopeSetpoint) {
        setpoint = telescopeSetpoint;
        output = pidTelescope.calculate(telescopeEncoder.get(), telescopeSetpoint);
        telescopeMotor.set(output*speed);
    }

    public void stop () {
        telescopeMotor.stopMotor();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Telescope Speed", speed);
        SmartDashboard.putNumber("Telescope Setpoint", setpoint);
    }
}
package org.usfirst.frc.team20.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

public class Navx {
	AHRS gyro = new AHRS(SerialPort.Port.kMXP);
	double turnAngle;
	DriveTrain drive;
	Constants constants;
	RobotDrive myRobot;
    	Joystick stick;
    	PIDController turnController;
    	double rotateToAngleRate;
    	PIDOutput output;
	
	//PID for NAVX
   	//Tuning:
   	//1. gradually increase P value until oscillation
 	//2. divide by 2
	//3. increase I value 
   	static final double kP = 0.05; 
    	static final double kI = 0.00;
    	static final double kD = 0.00;
    	static final double kF = 0.00;
	
	public Navx(DriveTrain d){
		drive = d;
		turnController = new PIDController(kP, kI, kD, kF, gyro, this);
       		turnController.setInputRange(-180.0f,  180.0f);
        	turnController.setOutputRange(-1.0, 1.0);
        	turnController.setAbsoluteTolerance(kToleranceDegrees);
        	turnController.setContinuous(true);
	}
	
	public void turnToAngle(double angle){
		angle = turnAngle;
		currentAngle = gyro.getYaw();
		if(turnAngle > 0){
			//TODO turn right
			drive.turnRight(speed);
			
		}
		if(turnAngle < 0){
			//TODO turn left
			drive.turnLeft(speed);
		}
		else{
			//do nothing
		}
	}
	public double getYawAngle(){
		double currentYawAngle = gyro.getYaw();
		return currentYawAngle;
	}
	
	public double getRollAngle(){
		double currentRollAngle = gyro.getRoll();
		return currentRollAngle;
	}
	
	public double getPitchAngle(){
		double currentPitchAngle = gyro.getPitch();
		return currentPitchAngle;
	}
	
	public void reset(){
		gyro.reset();
	}
}


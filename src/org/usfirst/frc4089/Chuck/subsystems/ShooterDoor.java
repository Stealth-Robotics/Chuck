package org.usfirst.frc4089.Chuck.subsystems;

import org.usfirst.frc4089.Chuck.RobotMap;

import edu.wpi.first.wpilibj.PWMConfigDataResult;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.command.Subsystem;

public class ShooterDoor extends Subsystem{

	private final Servo shooterDoor = RobotMap.shooterDoor;
	
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}
	public void openDoor(){
		shooterDoor.set(1.0);
	}
	public void closeDoor(){
		shooterDoor.set(0);
	}
}
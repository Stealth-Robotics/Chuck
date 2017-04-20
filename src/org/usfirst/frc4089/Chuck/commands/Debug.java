package org.usfirst.frc4089.Chuck.commands;

import org.usfirst.frc4089.Chuck.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class Debug extends Command{

	public Debug() {

	    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
	        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING

	        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
	        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
	        requires(Robot.drive);

	    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
	    }

	    // Called just before this Command runs the first time
	    protected void initialize() {
	    	setTimeout(0.5);
	    }

	    // Called repeatedly when this Command is scheduled to run
	    protected void execute() {
	    	//Robot.drive.debugR1();
	    	Robot.drive.resetEncoders();
	    }

	    // Make this return true when this Command no longer needs to run execute()
	    protected boolean isFinished() {
	        return isTimedOut();
	    }

	    // Called once after isFinished returns true
	    protected void end() {
	    }

	    // Called when another command which requires one or more of the same
	    // subsystems is scheduled to run
	    protected void interrupted() {
	    	end();
	    }

}
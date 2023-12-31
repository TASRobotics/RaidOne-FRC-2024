// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package raidone.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import raidone.robot.utils.AutoChooser;

public class Robot extends TimedRobot {

	private Command autonomousCommand;

	private RobotContainer robotContainer;

	private static AutoChooser chooser;
	Field2d field  = new Field2d();

	@Override
	public void robotInit() {
		robotContainer = new RobotContainer();
		chooser = new AutoChooser();
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void disabledInit() {}

	@Override
	public void disabledPeriodic() {}

	@Override
	public void disabledExit() {}

  	@Override
  	public void autonomousInit() {
    	autonomousCommand = robotContainer.getAutonomousCommand();

    	if (autonomousCommand != null) {
      		autonomousCommand.schedule();
		}
	}

  	@Override
  	public void autonomousPeriodic() {}

	@Override
	public void autonomousExit() {}

	@Override
	public void teleopInit() {
    	if (autonomousCommand != null) {
    		autonomousCommand.cancel();
    	}
	}

	@Override
	public void teleopPeriodic() {}

	@Override
	public void teleopExit() {}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {}

	@Override
	public void testExit() {}	

	/**
	 * Gets the AutoChooser object
	 * 
	 * @return AutoChooser
	 */
	public static AutoChooser getChooser() {
		return chooser;
	}

}

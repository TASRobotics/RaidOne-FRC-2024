// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package raidone.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import monologue.Monologue;

public class Robot extends LoggedRobot {

	private Command autonomousCommand;

	private RobotContainer robotContainer;

	Field2d field = new Field2d();

	@Override
	public void robotInit() {
		Logger.recordMetadata("R1 Season", "Amogus");

		if (isReal()) {
		    // Logger.addDataReceiver(new WPILOGWriter("/U/Logs")); // Log to a USB stick ("/U/logs")
		    Logger.addDataReceiver(new NT4Publisher());
		    new PowerDistribution(1, ModuleType.kRev);
		} else {
		    setUseTiming(false);
		    // String logPath = LogFileUtil.findReplayLog();
		    // Logger.setReplaySource(new WPILOGReader(logPath));
		    // Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
		}
		
		Logger.start();

		robotContainer = new RobotContainer();
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		Monologue.update();
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

	@Override
	public void simulationInit() {
		SmartDashboard.putData("Field", field);
	}

	@Override
	public void simulationPeriodic() {
		field.setRobotPose(robotContainer.getSwerve().getPose());
	}

}

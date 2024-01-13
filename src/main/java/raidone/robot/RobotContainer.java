// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package raidone.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import monologue.Logged;
import monologue.Monologue;
import monologue.Monologue.LogNT;
import raidone.robot.Constants.TeleopConstants;
import raidone.robot.auto.Autos;
import raidone.robot.subsystems.Swerve;

public class RobotContainer implements Logged {

	private final Swerve swerve = new Swerve();
	private final XboxController master = new XboxController(0);

	@LogNT
	private final Autos autos = new Autos(swerve);

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {		
		Monologue.setupLogging(this, "/Robot");

    	configureBindings();

		swerve.setDefaultCommand(
			new RunCommand(() -> swerve.drive(
				-MathUtil.applyDeadband(master.getLeftY(), TeleopConstants.kDriveDeadband),
				-MathUtil.applyDeadband(master.getLeftX(), TeleopConstants.kDriveDeadband),
				-MathUtil.applyDeadband(master.getRightX(), TeleopConstants.kDriveDeadband),
				false),
				swerve
			)
		);
	}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be created via the
	 * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
	 * predicate, or via the named factories in {@link
	 * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
	 * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
	 * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
	 * joysticks}.
	 */
	private void configureBindings() {}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
    	return autos.get();
	}

	/**
	 * Gets the swerve subsystem
	 * 
	 * @return Swerve object
	 */
	public Swerve getSwerve() {
		return swerve;
	}
}

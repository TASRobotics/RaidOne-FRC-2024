// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package raidone.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import monologue.Logged;
import monologue.Monologue;
import monologue.Monologue.LogNT;
import raidone.robot.Constants.TeleopConstants;
import raidone.robot.auto.Autos;
import raidone.robot.commands.OrdinalTurn;
import raidone.robot.subsystems.Swerve;

public class RobotContainer implements Logged {

	private final Swerve swerve = new Swerve();
	private final XboxController driver = new XboxController(0);

     /* Drive Controls */
     private final int translationAxis = XboxController.Axis.kLeftY.value;
     private final int strafeAxis = XboxController.Axis.kLeftX.value;
     private final int rotationAxis = XboxController.Axis.kRightX.value; // For controller
     // private final int rotationAxis = Joystick.kDefaultTwistChannel; // For joystick

     /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final POVButton ordinalTurnUp = new POVButton(driver, 0);
    private final POVButton ordinalTurnDown = new POVButton(driver, 180);
    private final POVButton ordinalTurnLeft = new POVButton(driver, 270);
    private final POVButton ordinalTurnRight = new POVButton(driver, 90);

	@LogNT
	private final Autos autos = new Autos(swerve);

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {		
		Monologue.setupLogging(this, "/Robot");

    	configureBindings();

		swerve.setDefaultCommand(
			new RunCommand(() -> swerve.drive(
				-MathUtil.applyDeadband(translationAxis, TeleopConstants.DRIVE_DEADBAND),
				-MathUtil.applyDeadband(strafeAxis, TeleopConstants.DRIVE_DEADBAND),
				MathUtil.applyDeadband(rotationAxis, TeleopConstants.DRIVE_DEADBAND),
				true),
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
	private void configureBindings() {
        zeroGyro.onTrue(new InstantCommand(() -> swerve.zeroHeading()));
        ordinalTurnUp.onTrue(new OrdinalTurn(0, swerve));
        ordinalTurnDown.onTrue(new OrdinalTurn(180, swerve));
        ordinalTurnLeft.onTrue(new OrdinalTurn(270, swerve));
        ordinalTurnRight.onTrue(new OrdinalTurn(90, swerve));
    }

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

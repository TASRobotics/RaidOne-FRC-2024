package raidone.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import raidone.robot.commands.*;
import raidone.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final XboxController driver = new XboxController(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value; // For controller
    // private final int rotationAxis = Joystick.kDefaultTwistChannel; // For
    // joystick

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    // private final JoystickButton robotCentric = new JoystickButton(driver,
    // XboxController.Button.kLeftBumper.value);
    private final JoystickButton zeroPose = new JoystickButton(driver, XboxController.Button.kX.value);

    /* Ordinal Turn Buttons */
    private final POVButton ordinalTurnUp = new POVButton(driver, 0);
    private final POVButton ordinalTurnDown = new POVButton(driver, 180);
    private final POVButton ordinalTurnLeft = new POVButton(driver, 270);
    private final POVButton ordinalTurnRight = new POVButton(driver, 90);

    // IF BLUE: 45, if red, 315
    private final Trigger turnToSource = new Trigger(() -> getLeftTrigger());
    private final Trigger turnToAmp = new Trigger(() -> getRightTrigger());

    /* Subsystems */
    private final Swerve swerve = new Swerve();

    public boolean getRightTrigger() {
        return driver.getRightTriggerAxis() > 0.7;
    }

    public boolean getLeftTrigger() {
        return driver.getLeftTriggerAxis() > 0.7;
    }

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        swerve.setDefaultCommand(
                new TeleopSwerve(
                        swerve,
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis),
                        () -> -driver.getRawAxis(rotationAxis) * 0.5,
                        () -> false));

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> swerve.zeroHeading()));
        zeroPose.onTrue(
                new InstantCommand(() -> swerve.setPose(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)))));
                
        turnToSource.onTrue(
                new OrdinalTurn(45, swerve));
        turnToAmp.onTrue(
                new OrdinalTurn(270, swerve));

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
        return null;
    }

    public Swerve getSwerve() {
        return swerve;
    }
}

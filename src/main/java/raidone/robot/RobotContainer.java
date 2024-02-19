package raidone.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import monologue.Annotations.Log;
import raidone.robot.Auto.Autos;
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
    private final Joystick driver = new Joystick(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value; // For controller
    // private final int rotationAxis = Joystick.kDefaultTwistChannel; // For joystick

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton zeroPose = new JoystickButton(driver, XboxController.Button.kX.value);

    /* Subsystems */
    private final Swerve swerve = new Swerve();

    private Autos autos = new Autos(swerve);


    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        swerve.setDefaultCommand(
                new TeleopSwerve(
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis),
                        () -> driver.getRawAxis(rotationAxis) * 0.5,
                        () -> robotCentric.getAsBoolean(),
                        swerve));

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
        zeroPose.onTrue(new InstantCommand(() -> swerve.setPose(new Pose2d(new Translation2d(0,0), new Rotation2d(0)))));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // return autos.get();
        return AutoBuilder.buildAuto("testAuto");

        // SwerveModuleState[] states = new SwerveModuleState[4];
        // states[0] = new SwerveModuleState(2.0, Rotation2d.fromDegrees(0));
        // states[1] = new SwerveModuleState(2.0, Rotation2d.fromDegrees(0));
        // states[2] = new SwerveModuleState(2.0, Rotation2d.fromDegrees(0));
        // states[3] = new SwerveModuleState(2.0, Rotation2d.fromDegrees(0));

        // return Commands.run( () ->
        //     swerve.setModuleStates(states)
        // );

        
    }
    
    public Swerve getSwerve() {
        return swerve;
    }

}

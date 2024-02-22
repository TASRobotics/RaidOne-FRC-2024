package raidone.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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
    // private final int rotationAxis = Joystick.kDefaultTwistChannel; // For
    // joystick

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    // private final JoystickButton setArm = new JoystickButton(driver,
    // XboxController.Button.kStart.value);
    private final JoystickButton stow = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton home = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton amp = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton intake = new JoystickButton(driver, XboxController.Button.kX.value);

    /* Subsystems */
    private final Swerve swerve = new Swerve();
    private final Wrist wrist = new Wrist();
    private final Arm arm = new Arm();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        swerve.setDefaultCommand(
                new TeleopSwerve(
                        swerve,
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis),
                        () -> driver.getRawAxis(rotationAxis) * 0.5,
                        () -> robotCentric.getAsBoolean()));

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
        // zeroPose.onTrue(new InstantCommand(() -> swerve.setPose(new Pose2d(new
        // Translation2d(0,0), new Rotation2d(0)))));
        // setArm.toggleOnTrue(new ParallelCommandGroup(new AutoArm(arm), new
        // AutoWrist(wrist)));
        // stow.onTrue(new ArmGo(arm, Constants.Arm.INTAKEPOS).andThen(new
        // ArmHome(arm)));
        amp.onTrue(new ParallelCommandGroup(new ArmGo(arm, Constants.Arm.SCORINGPOS),
                new WristGo(wrist, Constants.Wrist.SCORINGPOS)));
        intake.onTrue(new ParallelCommandGroup(
                new SequentialCommandGroup(new ArmGo(arm, Constants.Arm.INTAKEPOS), new ArmHome(arm)),
                new WristGo(wrist, Constants.Wrist.INTAKEPOS)));
        home.onTrue(new ParallelCommandGroup(new ArmHome(arm), new WristHome(wrist)));
        stow.onTrue(new SequentialCommandGroup(new ArmGo(arm, Constants.Arm.INTAKEPOS), new ArmHome(arm)));
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

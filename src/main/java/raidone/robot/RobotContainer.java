package raidone.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import raidone.robot.commands.*;
// import static raidone.robot.Constants.*;
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
    // Controllers
    private final XboxController driver = new XboxController(0);
    private final Joystick driver2 = new Joystick(1);

    // Driver joystick axes
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    // Button board bindings
    private final int pinkButton = XboxController.Button.kY.value;
    private final int greenButtonL = XboxController.Button.kB.value;
    private final int greenButtonR = XboxController.Button.kA.value;
    private final int yellowButtonL = XboxController.Button.kX.value;
    private final int yellowButtonR = XboxController.Button.kStart.value;

    // Reset for field oriented
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);

    // Intake buttons
    private final JoystickButton intakeIn = new JoystickButton(driver2, greenButtonL);
    private final JoystickButton intakeOut = new JoystickButton(driver2, greenButtonR);

    // Arm & wrist position buttons
    private final JoystickButton stow = new JoystickButton(driver2, yellowButtonR);
    private final JoystickButton home = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton amp = new JoystickButton(driver2, pinkButton);
    private final JoystickButton intakePos = new JoystickButton(driver2, yellowButtonL);

    // Ordinal turn buttons
    private final POVButton ordinalTurnUp = new POVButton(driver, 0);
    private final POVButton ordinalTurnDown = new POVButton(driver, 180);
    private final POVButton ordinalTurnLeft = new POVButton(driver, 270);
    private final POVButton ordinalTurnRight = new POVButton(driver, 90);
    private final Trigger turnToSource = new Trigger(() -> getTrigger(true));
    private final Trigger turnToAmp = new Trigger(() -> getTrigger(false));

    // Declare subsystems
    private final Swerve swerve = new Swerve();
    private final Wrist wrist = new Wrist();
    private final Intake intake = new Intake();

    // Get the triggers
    public boolean getTrigger(boolean isRight) {
        if (isRight)
            return driver.getRightTriggerAxis() > 0.5;
        else
            return driver.getLeftTriggerAxis() > 0.5;
    }

    public RobotContainer() {
        swerve.setDefaultCommand(
                new TeleopSwerve(
                        swerve,
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis),
                        () -> -driver.getRawAxis(rotationAxis) * 0.5,
                        () -> false));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        zeroGyro.onTrue(new InstantCommand(() -> swerve.zeroHeading()));

        intakeIn.toggleOnTrue(new IntakeIn(intake, Constants.Intake.PERCENT).andThen(new IntakeRetract(intake)));
        intakeOut.onTrue(new IntakeOut(intake, Constants.Intake.PERCENT).withTimeout(1));

        stow.onTrue(new SequentialCommandGroup(
                new ParallelCommandGroup(new ArmGo(Constants.Arm.INTAKEPOS), new WristGo(wrist, 0)).withTimeout(1),
                new ParallelCommandGroup(new ArmHome(), new WristHome(wrist))));
        home.onTrue(new ParallelCommandGroup(new ArmHome(), new WristHome(wrist)));
        amp.onTrue(new ParallelCommandGroup(
                new ArmGo(Constants.Arm.SCORINGPOS),
                new WristGo(wrist, Constants.Wrist.SCORINGPOS)));
        intakePos.onTrue(new ParallelCommandGroup(
                new SequentialCommandGroup(new ArmGo(Constants.Arm.INTAKEPOS), new ArmHome()),
                new WristGo(wrist, Constants.Wrist.INTAKEPOS)));

        ordinalTurnUp.onTrue(new OrdinalTurn(0, swerve));
        ordinalTurnDown.onTrue(new OrdinalTurn(180, swerve));
        ordinalTurnLeft.onTrue(new OrdinalTurn(270, swerve));
        ordinalTurnRight.onTrue(new OrdinalTurn(90, swerve));
        turnToSource.onTrue(
                new OrdinalTurn(135, swerve)); // blue = 135; red = 225
        turnToAmp.onTrue(
                new OrdinalTurn(270, swerve)); // blue = 270; red = 90
    }

    public Command getAutonomousCommand() {
        return null;
    }
}

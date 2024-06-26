package raidone.robot;

import static raidone.robot.commands.TrapezoidGenerator.armProfile;
import static raidone.robot.commands.TrapezoidGenerator.wristProfile;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import raidone.robot.Constants.*;
import raidone.robot.commands.*;

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
    public static final XboxController driver = new XboxController(0);
    // public static final Joystick driver2 = new Joystick(1);
    public static final XboxController driver2 = new XboxController(1);

    // Driver joystick axes
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    // Button board bindings
    // private final int pinkButton = XboxController.Button.kY.value;
    // private final int greenButtonL = XboxController.Button.kB.value;
    // private final int greenButtonR = XboxController.Button.kA.value;
    // private final int yellowButtonL = XboxController.Button.kX.value;
    // private final int yellowButtonR = XboxController.Button.kStart.value;

    // 2nd controller bindings
    // start = home
    // y = amp
    // a = ground
    // LB = score out
    // RB = intake

    private final int pinkButton = XboxController.Button.kY.value;
    private final int greenButtonL = XboxController.Button.kRightBumper.value;
    private final int greenButtonR = XboxController.Button.kLeftBumper.value;
    private final int yellowButtonL = XboxController.Button.kA.value;
    private final int yellowButtonR = XboxController.Button.kStart.value;

    // Reset for field oriented
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);

    // Intake buttons
    private final JoystickButton intakeIn = new JoystickButton(driver2, greenButtonL);
    private final JoystickButton intakeOut = new JoystickButton(driver2, greenButtonR);
    private final JoystickButton intakeEject = new JoystickButton(driver2, XboxController.Button.kB.value);

    // Arm & wrist position buttons
    private final JoystickButton stow = new JoystickButton(driver2, yellowButtonR);
    // private final JoystickButton home = new JoystickButton(driver,
    // XboxController.Button.kRightBumper.value);
    private final JoystickButton amp = new JoystickButton(driver2, pinkButton);
    private final JoystickButton intakePos = new JoystickButton(driver2, yellowButtonL);

    // Ordinal turn buttons
    // private final POVButton ordinalTurnUp = new POVButton(driver, 0);
    // private final POVButton ordinalTurnDown = new POVButton(driver, 180);
    // private final POVButton ordinalTurnLeft = new POVButton(driver, 270);
    // private final POVButton ordinalTurnRight = new POVButton(driver, 90);
    // private final Trigger turnToSource = new Trigger(() -> getTrigger(true));
    // private final Trigger turnToAmp = new Trigger(() -> getTrigger(false));

    private final JoystickButton climbUp = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton climbHome = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    // Autochooser
    private final SendableChooser<Command> chooser;

    // Subsystem references
    private final raidone.robot.subsystems.Swerve swerve = raidone.robot.subsystems.Swerve.system();
    private final raidone.robot.subsystems.Arm arm = raidone.robot.subsystems.Arm.system();

    // Get the triggers
    public boolean getTrigger(boolean isRight) {
        if (isRight)
            return driver.getRightTriggerAxis() > 0.5;
        else

            return driver.getLeftTriggerAxis() > 0.5;
    }

    public RobotContainer() {

        // Commands for auto
        NamedCommands.registerCommand("ArmIntake", new SequentialCommandGroup(
                armProfile(Arm.CONSTRAINTPOS, false),
                wristProfile(Wrist.INTAKEPOS, false),
                new ArmHome()));

        NamedCommands.registerCommand("IntakeNote",
                new IntakeIn(Constants.Intake.PERCENT).withTimeout(3.5));

        NamedCommands.registerCommand("Amp", new ParallelCommandGroup(
                armProfile(Arm.SCORINGPOS, false),
                wristProfile(Wrist.SCORINGPOS, false)));

        NamedCommands.registerCommand("AmpScore", new IntakeOut(Constants.Intake.PERCENT).withTimeout(1));

        NamedCommands.registerCommand("Home", new SequentialCommandGroup(
                armProfile(Arm.CONSTRAINTPOS, false),
                new WristHome(),
                new ArmHome()));

        NamedCommands.registerCommand("IntakeEject", new IntakeEject().withTimeout(0.5));

        NamedCommands.registerCommand("TurnTo0", new OrdinalTurn(0));
        NamedCommands.registerCommand("TurnTo90", new OrdinalTurn(90));

        chooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto", chooser);

        swerve.setDefaultCommand(
                new TeleopSwerve(
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis),
                        () -> -driver.getRawAxis(rotationAxis),
                        () -> false));

        arm.setDefaultCommand(new ResetArmEncoder());

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        zeroGyro.onTrue(new InstantCommand(() -> swerve.zeroHeading()));

        intakeIn.toggleOnTrue(new IntakeIn(Intake.PERCENT).andThen(new IntakeRetract())
                .andThen(new ParallelCommandGroup(
                        new InstantCommand(() -> driver.setRumble(GenericHID.RumbleType.kRightRumble, 1)),
                        new WaitCommand(0.987)))
                .andThen(new InstantCommand(() -> driver.setRumble(GenericHID.RumbleType.kRightRumble, 0))));

        intakeOut.onTrue(new IntakeOut(Intake.PERCENT).withTimeout(1));

        intakeEject.toggleOnTrue(new IntakeEject().withTimeout(0.5));

        stow.onTrue(new SequentialCommandGroup(
                armProfile(Arm.CONSTRAINTPOS, false),
                new WristHome(),
                new ArmHome()));

        amp.onTrue(new ParallelCommandGroup(
                armProfile(Arm.SCORINGPOS, false),
                wristProfile(Wrist.SCORINGPOS, false)));

        intakePos.onTrue(new SequentialCommandGroup(
                armProfile(Arm.CONSTRAINTPOS, false).withTimeout(0.75),
                wristProfile(Wrist.INTAKEPOS, false),
                new ArmHome(),
                new IntakeIn(Constants.Intake.PERCENT),
                new IntakeRetract(),
                armProfile(Arm.CONSTRAINTPOS, false),
                new WristHome(),
                new ArmHome()));
        // new ParallelCommandGroup(
        // new InstantCommand(() -> driver.setRumble(GenericHID.RumbleType.kRightRumble,
        // 1)),
        // new WaitCommand(0.987))
        // .andThen(new InstantCommand(() ->
        // driver.setRumble(GenericHID.RumbleType.kRightRumble, 0)))));

        // ordinalTurnUp.onTrue(new OrdinalTurn(0));
        // ordinalTurnDown.onTrue(new OrdinalTurn(180));
        // ordinalTurnLeft.onTrue(new OrdinalTurn(270));
        // ordinalTurnRight.onTrue(new OrdinalTurn(90));
        // turnToSource.onTrue(
        // new OrdinalTurn(225)); // blue = 135; red = 225
        // turnToAmp.onTrue(
        // new OrdinalTurn(90)); // blue = 270; red = 90

        climbHome.toggleOnTrue(new ParallelCommandGroup(
                new ClimbHome(Constants.Climb.DOWN_SPEED_PCT),
                new ClimbFollowHome(Constants.Climb.DOWN_SPEED_PCT)));
        climbUp.toggleOnTrue(new ParallelCommandGroup(
                new ClimbUp(Constants.Climb.UP_SPEED_PCT),
                new ClimbFollowUp(Constants.Climb.UP_SPEED_PCT)));
    }

    public Command getAutonomousCommand() {
        return chooser.getSelected();
    }
}
package raidone.robot;

import static raidone.robot.Constants.Intake.percent;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import raidone.robot.Auto.Autos;
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
    private final Joystick driver2 = new Joystick(1);

    // Driver joystick axes
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    // Brake/Coastbuttons
    private final JoystickButton brakeButton = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton coastButton = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    // Button board bindings
    private final int pinkButton = XboxController.Button.kY.value;
    private final int greenButtonL = XboxController.Button.kB.value;
    private final int greenButtonR = XboxController.Button.kA.value;
    private final int yellowButtonL = XboxController.Button.kX.value;
    private final int yellowButtonR = XboxController.Button.kStart.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);

    private final JoystickButton intakeIn = new JoystickButton(driver2, greenButtonL);
    private final JoystickButton intakeOut = new JoystickButton(driver2, greenButtonR);

    // Arm & wrist position buttons
    private final JoystickButton stow = new JoystickButton(driver2, yellowButtonR);
    // private final JoystickButton home = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton amp = new JoystickButton(driver2, pinkButton);
    private final JoystickButton intakePos = new JoystickButton(driver2, yellowButtonL);

    // Ordinal turn buttons
    private final POVButton ordinalTurnUp = new POVButton(driver, 0);
    private final POVButton ordinalTurnDown = new POVButton(driver, 180);
    private final POVButton ordinalTurnLeft = new POVButton(driver, 270);
    private final POVButton ordinalTurnRight = new POVButton(driver, 90);
    private final Trigger turnToSource = new Trigger(() -> getTrigger(true));
    private final Trigger turnToAmp = new Trigger(() -> getTrigger(false));

    private final SendableChooser<Command> autoChooser;

    public boolean getTrigger(boolean isRight) {
        if (isRight)
            return driver.getRightTriggerAxis() > 0.5;
        else
            return driver.getLeftTriggerAxis() > 0.5;
    }

    /* Subsystems */
    private final Swerve swerve = new Swerve();
    private final Wrist wrist = new Wrist();
    private final Arm arm = new Arm();
    private final Intake intake = new Intake();

    public static boolean noteStatus = false;

    // private Autos autos = new Autos(swerve);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Commands for auto
        NamedCommands.registerCommand("Intake", new ParallelCommandGroup(
                new ArmGo(arm, Constants.Arm.INTAKEPOS),
                new WristGo(wrist, Constants.Wrist.INTAKEPOS).withTimeout(1))
                .andThen(new Intake_In(intake, Constants.Intake.percent).andThen(new Intake_Retract(intake))));

        NamedCommands.registerCommand("Amp", new ParallelCommandGroup(
                new ArmGo(arm, Constants.Arm.SCORINGPOS).withTimeout(1.25),
                new WristGo(wrist, Constants.Wrist.SCORINGPOS).withTimeout(1)));
        // .andThen(new Intake_Out(intake, Constants.Intake.percent).withTimeout(1)));

        NamedCommands.registerCommand("AmpScore", new Intake_Out(intake, Constants.Intake.percent).withTimeout(1));

        // NamedCommands.registerCommand("AmpScore", new Intake_Out(intake,
        // Constants.Intake.percent).withTimeout(1));

        NamedCommands.registerCommand("Home", new ParallelCommandGroup(
                new ArmHome(arm), new WristHome(wrist)));

        NamedCommands.registerCommand("TurnTo0", new OrdinalTurn(0, swerve));
        NamedCommands.registerCommand("TurnTo90", new OrdinalTurn(90, swerve));

        swerve.setDefaultCommand(
                new TeleopSwerve(
                        () -> -driver.getRawAxis(translationAxis) * 0.75,
                        () -> -driver.getRawAxis(strafeAxis) * 0.75,
                        () -> -driver.getRawAxis(rotationAxis),
                        () -> false,
                        swerve));

        // Configure the button bindings
        configureButtonBindings();

        autoChooser = AutoBuilder.buildAutoChooser();
        // do it like this to manually set the angle when you start with an auto that
        // starts with a non-zero angle
        // autoChooser.addOption("preloadauto", new SequentialCommandGroup(
        // new InstantCommand(() -> swerve.getImu().setYaw(270)), new
        // PathPlannerAuto("DS1")));
        SmartDashboard.putData(autoChooser);

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
        zeroGyro.onTrue(new InstantCommand(() -> swerve.zeroHeading()));

        intakeIn.toggleOnTrue(new Intake_In(intake, Constants.Intake.percent).andThen(new Intake_Retract(intake)));
        intakeOut.onTrue(new Intake_Out(intake, Constants.Intake.percent).withTimeout(1));

        stow.onTrue(new SequentialCommandGroup(
                new ParallelCommandGroup(new ArmGo(arm, Constants.Arm.INTAKEPOS), new WristGo(wrist, 0)).withTimeout(1),
                new ParallelCommandGroup(new ArmHome(arm), new WristHome(wrist))));
        // home.onTrue(new ParallelCommandGroup(new ArmHome(arm), new WristHome(wrist)));
        amp.onTrue(new ParallelCommandGroup(
                new ArmGo(arm, Constants.Arm.SCORINGPOS),
                new WristGo(wrist, Constants.Wrist.SCORINGPOS)));
        intakePos.onTrue(new ParallelCommandGroup(
                new SequentialCommandGroup(new ArmGo(arm, Constants.Arm.INTAKEPOS), new ArmHome(arm)),
                new WristGo(wrist, Constants.Wrist.INTAKEPOS)));

        ordinalTurnUp.onTrue(new OrdinalTurn(0, swerve));
        ordinalTurnDown.onTrue(new OrdinalTurn(180, swerve));
        ordinalTurnLeft.onTrue(new OrdinalTurn(270, swerve));
        ordinalTurnRight.onTrue(new OrdinalTurn(90, swerve));
        turnToSource.onTrue(
                new OrdinalTurn(135, swerve)); // blue = 135; red = 225
        turnToAmp.onTrue(
                new OrdinalTurn(270, swerve)); // blue = 270; red = 90

        brakeButton.onTrue(new InstantCommand(() -> swerve.setBrake()));
        coastButton.onTrue(new InstantCommand(() -> swerve.setCoast()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // return autoChooser.getSelected();
        // return new PathPlannerAuto("Rebuild");
        return new PathPlannerAuto("TuneAuto");

    }

    public Swerve getSwerve() {
        return swerve;
    }

}

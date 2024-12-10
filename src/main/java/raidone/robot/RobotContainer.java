package raidone.robot;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.CANifier;
//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Commands;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import raidone.robot.commands.*;
import raidone.robot.subsystems.*;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import raidone.robot.TunerConstants;
import raidone.robot.subsystems.CommandSwerveDrivetrain;

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
    // private static RobotContainer robotContainer = new RobotContainer();
    /* Subsystems */
    private final static CANifier limitCanifier = new CANifier(0);
    private final Wrist wrist = Wrist.system();
    private final Arm arm = Arm.system();
    private final Intake intake = Intake.system();
    private final Lights lights = Lights.system();
    private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
    private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final CommandXboxController joystick = new CommandXboxController(1); // My joystick
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                                     // driving in open loop
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // private void configureBindings() {
    // drivetrain.setDefaultCommand( // Drivetrain will execute this command
    // periodically
    // drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() *
    // MaxSpeed) // Drive forward with
    // // negative Y (forward)
    // .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X
    // (left)
    // .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive
    // counterclockwise with negative X (left)
    // ));

    // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // joystick.b().whileTrue(drivetrain
    // .applyRequest(() -> point.withModuleDirection(new
    // Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // // reset the field-centric heading on left bumper press
    // joystick.leftBumper().onTrue(drivetrain.runOnce(() ->
    // drivetrain.seedFieldRelative()));

    // if (Utils.isSimulation()) {
    // drivetrain.seedFieldRelative(new Pose2d(new Translation2d(),
    // Rotation2d.fromDegrees(90)));
    // }
    // drivetrain.registerTelemetry(logger::telemeterize);
    // }

    /* Drive Controls */
    // private final int translationAxis = XboxController.Axis.kLeftY.value;
    // private final int strafeAxis = XboxController.Axis.kLeftX.value;
    // private final int rotationAxis = XboxController.Axis.kRightX.value; // For
    // controller
    // private final int rotationAxis = Joystick.kDefaultTwistChannel; // For
    // joystick

    /* Driver Buttons */
    // private final JoystickButton zeroGyro = new JoystickButton(driver,
    // XboxController.Button.kY.value);
    // private final JoystickButton robotCentric = new JoystickButton(driver,
    // XboxController.Button.kLeftBumper.value);
    // private final JoystickButton zeroPose = new JoystickButton(driver,
    // XboxController.Button.kX.value);
    // private final JoystickButton setArm = new JoystickButton(driver,
    // XboxController.Button.kStart.value);
    // private final JoystickButton home = new JoystickButton(driver,
    // XboxController.Button.kRightBumper.value);
    private final Trigger intakePos = joystick.a();
    private final Trigger scoringPos = joystick.b();
    private final Trigger bothHome = joystick.back();
    // private final JoystickButton intakePos = new JoystickButton(driver, XboxController.Button.kA.value);
    // private final JoystickButton scoringPos = new JoystickButton(driver, XboxController.Button.kB.value);
    // private final JoystickButton armMotionProfile = new JoystickButton(driver,
    // XboxController.Button.kY.value);
    // private final JoystickButton armhome = new JoystickButton(driver,
    // XboxController.Button.kX.value);
    // private final JoystickButton bothMotionMagic = new JoystickButton(driver,
    // XboxController.Button.kRightBumper.value);
    // private final JoystickButton armgoreverse = new JoystickButton(driver,
    // XboxController.Button.kLeftBumper.value);
    // private final JoystickButton bothHome = new JoystickButton(driver, XboxController.Button.kBack.value);
    // private final JoystickButton runIntake = new JoystickButton(driver,
    // XboxController.Button.kRightStick.value);
    // private final BooleanSupplier leftTrigger = () ->
    // driver.getRawAxis(XboxController.Axis.kLeftTrigger.value) > 0.2;
    private final Trigger rightTrigger = joystick.axisGreaterThan(Axis.kRightTrigger.value, 0.2);
    private final Trigger leftTrigger = joystick.axisGreaterThan(Axis.kLeftTrigger.value, 0.2);

    // private SendableChooser<Command> autoChooser;

    CommandSequences sequences = new CommandSequences(this.arm, this.wrist, this.intake);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        // swerve.setDefaultCommand(
        // new TeleopSwerve(
        // () -> -driver.getRawAxis(translationAxis),
        // () -> -driver.getRawAxis(strafeAxis),
        // () -> driver.getRawAxis(rotationAxis) * 0.5,
        // () -> robotCentric.getAsBoolean()));

        // Configure the button bindings
        configureBindings();
        configureButtonBindings();
        arm.setDefaultCommand(new ArmGo(0));
        wrist.setDefaultCommand(new WristGo(0));
        intake.setDefaultCommand(new IntakeIn(0));

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    
    private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }
    private void configureButtonBindings() {
        /* Driver Buttons */
        // zeroGyro.onTrue(new InstantCommand(() -> swerve.zeroHeading()));
        // zeroPose.onTrue(new InstantCommand(() -> swerve.setPose(new Pose2d(new
        // Translation2d(0,0), new Rotation2d(0)))));
        // Command wristHomeSequence = sequences.wristHomeSequence();
        // Command armHomeSequence = sequences.armHomeSequence();
        // Command bothHomeSequence = sequences.bothHomeSequence();
        // Command scoreAndHome = sequences.scoreSequence();

        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                                   // negative Y
                                                                                                   // (forward)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                    // negative X (left)
                ));

        joystick.x().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.y().whileTrue(drivetrain
                .applyRequest(
                        () -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }
        drivetrain.registerTelemetry(logger::telemeterize);

        intakePos.onTrue(sequences.bothMotionProfile(Constants.Arm.intakePosition, Constants.Wrist.INTAKEPOS.position));
        scoringPos.onTrue(
                sequences.bothMotionProfile(Constants.Arm.scoringPosition, Constants.Wrist.SCORINGPOS.position));

        // wristgoreverse.whileTrue(new WristGo(-0.1));
        // wristhome.onTrue(new
        // WristHome().andThen(Commands.waitSeconds(0.5)).andThen(new WristHome()));
        // wristhome.onTrue(sequences.wristHomeSequence().withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming));
        // armMotionProfile.onTrue(new
        // ArmMotionProfile(Constants.Arm.scoringPosition).withTimeout(1.5)); // leo
        // added
        // armhome.onTrue(sequences.armHomeSequence().withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming));
        // ; // leo added

        // bothMotionMagic.onTrue(sequences.bothMotionProfile(Constants.Arm.scoringPosition,
        // Constants.Wrist.SCORINGPOS.position));
        // armgoreverse.whileTrue(new ArmGo(-0.1)); // leo added/commented out
        // temporarily
        bothHome.onTrue(sequences.bothHomeSequence());
        // runIntake.whileTrue(new IntakeIn(Constants.Intake.intakePercent));
        // Trigger leftTriggerBoolean = new Trigger(leftTrigger);
        // leftTriggerBoolean.whileTrue(new IntakeIn(1.0));
        rightTrigger.whileTrue(sequences.intakeInSequence());
        leftTrigger.whileTrue(new IntakeOut(Constants.Intake.scorePercent));
        // setArm.toggleOnTrue(new SequentialCommandGroup(new AutoArm(arm), new
        // AutoWrist(wrist)));
        // home.onTrue(new ArmHome(arm, wrist));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }

    // public Swerve getSwerve() {
    // return swerve;
    // }

    public static CANifier getCANifier() {
        return limitCanifier;
    }

}

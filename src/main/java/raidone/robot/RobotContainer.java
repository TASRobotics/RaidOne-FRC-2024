package raidone.robot;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.CANifier;
//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
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
    //private static RobotContainer robotContainer = new RobotContainer();
    /* Subsystems */
    private final static CANifier limitCanifier = new CANifier(0);
    private final Wrist wrist = Wrist.system();
    private final Arm arm = Arm.system();
    private final Intake intake = Intake.system();

    /* Drive Controls */
    //private final int translationAxis = XboxController.Axis.kLeftY.value;
    //private final int strafeAxis = XboxController.Axis.kLeftX.value;
    //private final int rotationAxis = XboxController.Axis.kRightX.value; // For controller
    // private final int rotationAxis = Joystick.kDefaultTwistChannel; // For joystick

    /* Driver Buttons */
    //private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    //private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    //private final JoystickButton zeroPose = new JoystickButton(driver, XboxController.Button.kX.value);
    //private final JoystickButton setArm = new JoystickButton(driver, XboxController.Button.kStart.value);
    //private final JoystickButton home = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton wristhome = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton wristgo = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton wristgoreverse = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton armhome = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton armgo = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton armgoreverse = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton bothHome = new JoystickButton(driver, XboxController.Button.kLeftStick.value);
    private final JoystickButton runIntake = new JoystickButton(driver, XboxController.Button.kRightStick.value);
    //private final Joystick test = new Joystick(1);
    private final BooleanSupplier leftTrigger = () -> driver.getRawAxis(XboxController.Axis.kLeftTrigger.value) > 0.2;
    private final BooleanSupplier rightTrigger = () -> driver.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.2;

    //private SendableChooser<Command> autoChooser;



    CommandSequences sequences = new CommandSequences(this.arm, this.wrist, this.intake);

   
    public enum RobotState {
         IDLE,
         HOMED_NO_NOTE,
         HOMED_HAS_NOTE,
         INTAKE_NO_NOTE,
         INTAKE_HAS_NOTE,
         SCORING_NO_NOTE,
         SCORING_HAS_NOTE
     }
     private static RobotState robotState = RobotState.IDLE; 
    

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        
        // swerve.setDefaultCommand(
        //         new TeleopSwerve(
        //                 () -> -driver.getRawAxis(translationAxis),
        //                 () -> -driver.getRawAxis(strafeAxis),
        //                 () -> driver.getRawAxis(rotationAxis) * 0.5,
        //                 () -> robotCentric.getAsBoolean()));
        
        // Configure the button bindings
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
    private void configureButtonBindings() {
        /* Driver Buttons */
        //zeroGyro.onTrue(new InstantCommand(() -> swerve.zeroHeading()));
       // zeroPose.onTrue(new InstantCommand(() -> swerve.setPose(new Pose2d(new Translation2d(0,0), new Rotation2d(0)))));
        Command wristHomeSequence = sequences.wristHomeSequence();
        Command armHomeSequence = sequences.armHomeSequence();
        Command bothHomeSequence = sequences.bothHomeSequence();
        Command scoreAndHome = sequences.scoreSequence();

        wristgo.whileTrue(new WristGo(0.1));
        wristgoreverse.whileTrue(new WristGo(-0.1));
        //wristhome.onTrue(new WristHome().andThen(Commands.waitSeconds(0.5)).andThen(new WristHome()));
        wristhome.onTrue(wristHomeSequence);
        armgo.whileTrue(new ArmGo(0.1));    
        armhome.onTrue(armHomeSequence);    
        //armhome.onTrue(new ArmHome().andThen(Commands.waitSeconds(0.1)).andThen(new ArmHome()));
        armgoreverse.whileTrue(new ArmGo(-0.1));
        bothHome.onTrue(bothHomeSequence);
        runIntake.whileTrue(new IntakeIn(Constants.Intake.intakePercent));
        Trigger leftTriggerBoolean = new Trigger(leftTrigger);
        leftTriggerBoolean.whileTrue(new IntakeIn(1.0));
        Trigger rightTriggerBoolean = new Trigger(rightTrigger);
        rightTriggerBoolean.onTrue(scoreAndHome);
        //setArm.toggleOnTrue(new SequentialCommandGroup(new AutoArm(arm), new AutoWrist(wrist)));
        //home.onTrue(new ArmHome(arm, wrist));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }
    
    //public Swerve getSwerve() {
        //return swerve;
    //}

    public static CANifier getCANifier() {
        return limitCanifier;
    }

    public static void setRobotState(RobotState rs){
        robotState = rs;
    }

    public static RobotState getRobotState(){
        return robotState;
    }
    
    

}

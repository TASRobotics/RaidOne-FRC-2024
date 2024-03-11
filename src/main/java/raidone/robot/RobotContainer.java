package raidone.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import raidone.robot.subsystems.Lighting;
import raidone.robot.commands.*;
import raidone.robot.Constants.*;

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
    
    private final JoystickButton intakePosing = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton intakePos = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton intakeRoll = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton scoringPosing = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton scoringPos = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton homePos = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton notePresent = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    Lighting LightControl = Lighting.system();

    // Subsystem references
    // Get the triggers
    public boolean getTrigger(boolean isRight) {
        if (isRight)
            return driver.getRightTriggerAxis() > 0.5;
        else

            return driver.getLeftTriggerAxis() > 0.5;
    }

    public RobotContainer() {
        

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        intakePosing.onTrue(new LEDAnim(LED.INTAKE_POSING));
        intakePos.onTrue(new LEDColor(LED.INTAKE_POS));
        scoringPosing.onTrue(new LEDAnim(LED.SCORING_POSING));
        scoringPos.onTrue(new LEDColor(LED.SCORING_POS));
        homePos.onTrue(new LEDAnim(LED.HOME_POS));
        intakeRoll.onTrue(new LEDAnim(LED.ROLLER_ON));
        notePresent.onTrue(new LEDAnim(LED.NOTE_PRESENT));
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("Rebuild");
    }
}

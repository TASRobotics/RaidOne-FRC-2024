package raidone.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import raidone.robot.Constants;
import raidone.robot.subsystems.Arm;
import raidone.robot.subsystems.Intake;
import raidone.robot.subsystems.Wrist;

public class CommandSequences {
    private Arm arm;
    private Wrist wrist;
    private Intake intake;

    
    
    public CommandSequences(Arm arm, Wrist wrist) {
        this.arm = arm;
        this.wrist = wrist;
       
      }
  
     public Command armHomeSequence() {
        return Commands.sequence(
            //new WristCoast(true),
            new ArmHome(),
            Commands.waitSeconds(0.25),
            new ArmHome()
            //new WristCoast(false)
        );
    }

    public Command wristHomeSequence() {
        return Commands.sequence(
            new WristHome(),
            Commands.waitSeconds(0.5),
            new WristHome()
        );
    }

    public Command bothHomeSequence(){
        // return Commands.parallel(
        //     armHomeSequence(),
        //     wristHomeSequence()
        // );
        return Commands.sequence(
            new WristCoast(true),
            Commands.parallel( armHomeSequence(),
                 wristHomeSequence()),
            Commands.waitSeconds(0.1),
            new ArmHome(),
            Commands.waitSeconds(0.1),
            new WristHome(),
            new WristCoast(false)
        );
    }
    public Command scoreSequence(){
        // return Commands.parallel(
        //     armHomeSequence(),
        //     wristHomeSequence()
        // );
        return Commands.sequence(
            new IntakeOut(Constants.Intake.scorePercent).withTimeout(1.0),
            Commands.waitSeconds(0.5),
            new IntakeOut(0).withTimeout(0.01),
            bothHomeSequence()
        );
    }
}

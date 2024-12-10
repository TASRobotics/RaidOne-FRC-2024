package raidone.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import raidone.robot.Constants;
import raidone.robot.subsystems.Arm;
import raidone.robot.subsystems.Intake;
import raidone.robot.subsystems.Wrist;

public class CommandSequences {
    //private Arm arm;
    //private Wrist wrist;
    //private Intake intake;

    
    
    public CommandSequences(Arm arm, Wrist wrist, Intake intake) {
        //this.arm = arm;
        //this.wrist = wrist;
        //this.intake = intake;
      }
  
     public Command armHomeSequence() {
        
        return Commands.sequence(
            
        
            //leo added/changed
            //new ArmHome(Constants.Arm.intakePosition),//double check 
            new ArmHome(),
             Commands.waitSeconds(0.25),
             new ArmHome()
             //new ArmHome(Constants.Arm.intakePosition)
  
        );
    }

    public Command wristHomeSequence() {
        return Commands.sequence(
            // new ArmMotionProfile(Constants.Arm.constrainPosition)).withTimeout(0.5),
            new WristHome(Constants.Wrist.HOMEPOS.position),
            Commands.waitSeconds(0.5),
            new WristHome(Constants.Wrist.HOMEPOS.position)
        
        );
    }


    public Command intakePos(){
        return Commands.sequence(new SequentialCommandGroup(
                new ArmMotionProfile(Constants.Arm.constrainPosition)).withTimeout(0.5),
                
                new WristMotionMagic(Constants.Wrist.CONSTRAINTPOS.position).withTimeout(0.5),
                
               
                armHomeSequence(),
                new WristMotionMagic(Constants.Wrist.INTAKEPOS.position).withTimeout(0.5)
        );
        
    }

    
    public Command intakeInSequence(){
        return Commands.sequence(new SequentialCommandGroup(
                new IntakeIn(Constants.Intake.intakePercent),
                new IntakeOut(0.2).withTimeout(0.1)
        )
    );
        
    }


    public Command bothHomeSequence(){
        // return Commands.parallel(
        //     armHomeSequence(),
        //     wristHomeSequence()
        // );
        // return Commands.sequence(
        //     new WristCoast(true),
        //     Commands.parallel( armHomeSequence(),
        //          wristHomeSequence()),
        //     Commands.waitSeconds(0.1),
            
        //     new WristCoast(false)
        // )
        return Commands.sequence(
            new WristCoast(true),
            armHomeSequence(),
            wristHomeSequence(),
            Commands.waitSeconds(0.1),
            armHomeSequence(),
            Commands.waitSeconds(0.1),
            new WristCoast(false)
        );
    }

    public Command bothMotionProfile(double armsetpoint, double wristsetpoint){
        return Commands.sequence(
            new WristCoast(true),
            Commands.parallel( new ArmMotionProfile(armsetpoint).withTimeout(1.0),
                new WristMotionMagic(wristsetpoint).withTimeout(1.0)),
            Commands.waitSeconds(0.1),
            
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

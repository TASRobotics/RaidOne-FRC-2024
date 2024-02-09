package raidone.robot.Auto;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import raidone.robot.commands.DrivePath;
import raidone.robot.subsystems.Swerve;

public class Autos {

    private final Swerve swerve;

    private final SendableChooser<Command> chooser;

    public Autos(Swerve swerve) {
        this.swerve = swerve;

        chooser = new SendableChooser<Command>();
        configureAutos();
        
        SmartDashboard.putData("Auton", chooser);
    }

    private void configureAutos() {
        chooser.setDefaultOption("No auto", null);
        chooser.addOption("Test auto", testAuto());
    }

    public Command get() {
        return chooser.getSelected();
    }

    public Command testAuto() {
        return Commands.sequence(
            new DrivePath(this.swerve, PathPlannerPath.fromPathFile("TestPath"))
                .andThen(new WaitCommand(0.5))
                .andThen(swerve.setX())
        );
    }
    
}

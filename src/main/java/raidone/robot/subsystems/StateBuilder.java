package raidone.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class StateBuilder extends SubsystemBase{
    private static StateBuilder stateBuilder = new StateBuilder();
    private final Wrist wrist = Wrist.system();
    private final Arm arm = Arm.system();
    private final Intake intake = Intake.system();
    private final Lights lights = Lights.system();
    public StateBuilder(){

        System.out.println("StateBuilder init");
        
    }

   
 
    @Override
    public void periodic(){
        
     
    }
   public static StateBuilder system(){
        return stateBuilder;
    }

}

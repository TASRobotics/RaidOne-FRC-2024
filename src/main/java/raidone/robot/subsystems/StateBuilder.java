package raidone.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import raidone.robot.subsystems.Intake.IntakeStateEnum;
import raidone.robot.subsystems.Lights.AnimationTypes;


public class StateBuilder extends SubsystemBase{
    private static StateBuilder stateBuilder = new StateBuilder();
    private final Wrist wrist = Wrist.system();
    private final Arm arm = Arm.system();
    private final Intake intake = Intake.system();
    private final Lights lights = Lights.system();

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

    public StateBuilder(){

        System.out.println("StateBuilder init");
        
    }

   
 
    @Override
    public void periodic(){
        getIntakeInfo();
        //getArmInfo();
        //getWristInfo();
     
    }

    private void getIntakeInfo(){
         IntakeStateEnum intakeState = intake.getState();
        
        if(intakeState == IntakeStateEnum.IDLE_NO_NOTE || intakeState == IntakeStateEnum.RUNNING_NO_NOTE){

            setRobotState(StateBuilder.RobotState.HOMED_NO_NOTE);
        } else if (intakeState == IntakeStateEnum.IDLE_HAS_NOTE || intakeState == IntakeStateEnum.RUNNING_HAS_NOTE){
          
            StateBuilder.setRobotState(StateBuilder.RobotState.HOMED_HAS_NOTE);
            //RobotContainer.setRobotState(RobotContainer.RobotState.HOMED_HAS_NOTE);
        }
    }


   public static StateBuilder system(){
        return stateBuilder;
    }

    public static void setRobotState(RobotState rs){
        robotState = rs;
    }

    public static RobotState getRobotState(){
        return robotState;
    }
    

}

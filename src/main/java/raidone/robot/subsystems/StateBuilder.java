package raidone.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import raidone.robot.subsystems.Arm.ArmStateEnum;
import raidone.robot.subsystems.Intake.IntakeStateEnum;
import raidone.robot.subsystems.Lights.AnimationTypes;
import raidone.robot.subsystems.Wrist.WristStateEnum;
import raidone.robot.subsystems.Wrist.WristStateEnum;


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
    private static RobotState prevRobotState = RobotState.IDLE;

    public enum ArmWristState {
        AT_HOME_POS,
        AT_INTAKE_POS,
        AT_SCORE_POS
    }
    private static ArmWristState armWristState = ArmWristState.HOMED;



    public StateBuilder(){

        System.out.println("StateBuilder init");
        
    }

   
 
    @Override
    public void periodic(){
        IntakeStateEnum intakeState = intake.getState();
        ArmStateEnum armState = arm.getState();
        WristStateEnum wristState = wrist.getState();

        if(armState == ArmStateEnum.AT_HOME_POS && wristState == WristStateEnum.AT_HOME_POS){
            armWristState = ArmWristState.AT_HOME_POS;
        } else if (armState == ArmStateEnum.AT_INTAKE_POS && wristState == WristStateEnum.AT_INTAKE_POS){
            armWristState = ArmWristState.AT_INTAKE_POS;
        } else if (armState == ArmStateEnum.AT_SCORE_POS && wristState == WristStateEnum.AT_SCORE_POS){
            armWristState = ArmWristState.AT_SCORE_POS;
        }
        

        

         if(intakeState == IntakeStateEnum.IDLE_NO_NOTE || intakeState == IntakeStateEnum.RUNNING_NO_NOTE){

            setRobotState(StateBuilder.RobotState.HOMED_NO_NOTE);
        } else if (intakeState == IntakeStateEnum.IDLE_HAS_NOTE || intakeState == IntakeStateEnum.RUNNING_HAS_NOTE){
          
            setRobotState(StateBuilder.RobotState.HOMED_HAS_NOTE);
            
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
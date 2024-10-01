package raidone.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import raidone.robot.subsystems.Arm.ArmStateEnum;
import raidone.robot.subsystems.Intake.IntakeStateEnum;
import raidone.robot.subsystems.Lights.AnimationTypes;
import raidone.robot.subsystems.Wrist.WristStateEnum;



public class StateBuilder extends SubsystemBase{
    private static StateBuilder stateBuilder = new StateBuilder();
    private final Wrist wrist = Wrist.system();
    private final Arm arm = Arm.system();
    private final Intake intake = Intake.system();
    //private final Lights lights = Lights.system();

    public enum RobotState {
        IDLE,
        HOMED_NO_NOTE,
        HOMED_HAS_NOTE,
        INTAKE_NO_NOTE,
        INTAKE_HAS_NOTE,
        SCORING_NO_NOTE,
        SCORING_HAS_NOTE,
        ARM_MOVING
    }
    private static RobotState robotState = RobotState.IDLE; 
    private static RobotState prevRobotState = RobotState.IDLE;

    private static boolean changed = false;

    public enum ArmWristState {
        AT_HOME_POS,
        AT_INTAKE_POS,
        AT_SCORE_POS,
        MOVING
    }
    private static ArmWristState armWristState = ArmWristState.AT_HOME_POS;



    public StateBuilder(){

        System.out.println("StateBuilder init");
        
    }

   
 
    @Override
    public void periodic(){
        IntakeStateEnum intakeState = intake.getState();
        ArmStateEnum armState = arm.getState();
        WristStateEnum wristState = wrist.getState();

        prevRobotState = RobotState.valueOf(robotState.name());

        if(armState == ArmStateEnum.AT_HOME_POS && wristState == WristStateEnum.AT_HOME_POS){
            armWristState = ArmWristState.AT_HOME_POS;
        } else if (armState == ArmStateEnum.AT_HOME_POS && wristState == WristStateEnum.AT_INTAKE_POS){
            armWristState = ArmWristState.AT_INTAKE_POS;
        } else if (armState == ArmStateEnum.AT_SCORE_POS && wristState == WristStateEnum.AT_SCORE_POS){
            armWristState = ArmWristState.AT_SCORE_POS;
        } else if (armState == ArmStateEnum.MOVING || wristState == WristStateEnum.MOVING){
            armWristState = ArmWristState.MOVING;
        }

        if(armWristState == ArmWristState.AT_HOME_POS && intakeState == IntakeStateEnum.IDLE_HAS_NOTE){
            robotState = RobotState.HOMED_HAS_NOTE;
        } else if (armWristState == ArmWristState.AT_HOME_POS && intakeState == IntakeStateEnum.IDLE_NO_NOTE){
            robotState = RobotState.HOMED_NO_NOTE;
         } else if (armWristState == ArmWristState.AT_SCORE_POS && intakeState == IntakeStateEnum.IDLE_HAS_NOTE){
            robotState = RobotState.SCORING_HAS_NOTE;
         } else if (armWristState == ArmWristState.AT_SCORE_POS && intakeState == IntakeStateEnum.IDLE_NO_NOTE){
            robotState = RobotState.SCORING_NO_NOTE;
         } else if (armWristState == ArmWristState.AT_INTAKE_POS && intakeState == IntakeStateEnum.RUNNING_NO_NOTE){
            robotState = RobotState.SCORING_HAS_NOTE;
         } else if (armWristState == ArmWristState.AT_INTAKE_POS && intakeState == IntakeStateEnum.RUNNING_HAS_NOTE){
            robotState = RobotState.SCORING_NO_NOTE;
         } else if (armWristState == ArmWristState.MOVING){
            robotState = RobotState.ARM_MOVING;
         }
        
        if(robotState != prevRobotState){
            changed = true;
        } else {
            changed = false;
        }

        

        //  if(intakeState == IntakeStateEnum.IDLE_NO_NOTE || intakeState == IntakeStateEnum.RUNNING_NO_NOTE){

        //     setRobotState(StateBuilder.RobotState.HOMED_NO_NOTE);
        // } else if (intakeState == IntakeStateEnum.IDLE_HAS_NOTE || intakeState == IntakeStateEnum.RUNNING_HAS_NOTE){
          
        //     setRobotState(StateBuilder.RobotState.HOMED_HAS_NOTE);
            
        // }
   
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
    
    public static boolean getChanged(){
        return changed;
    }

}
package frc.robot.auto.actions;


//import frc.robot.subsystems.Suspension;
//import frc.robot.subsystems.Suspension.WantedState;


/**
 * Turns the robot to a specified heading
 * 
 * @see Action
 */
/*
public class DriveSuspensionForward implements Action {

  
    private Suspension mSuspension = Suspension.getInstance();
    private double setPoint=0;
    

    public DriveSuspensionForward(double set) {
        setPoint=set;
    }

    @Override
    public boolean isFinished() {
        if(mSuspension.atWheelPosition()){
            return true;
        }else return false;
    }

    @Override
    public void update() {
        // Nothing done here, controller updates in mEnabedLooper in robot
    }

    @Override
    public void done() {
        mSuspension.setWantedState(WantedState.IDLE);
    }

    @Override
    public void start() {
        mSuspension.setWheelPosition(setPoint);
       
    }
}*/

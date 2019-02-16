package frc.robot.auto.actions;


//import frc.robot.subsystems.Suspension;
//import frc.robot.subsystems.Suspension.WantedState;


/**
 * Turns the robot to a specified heading
 * 
 * @see Action
 */
/*
public class ClimbUp implements Action {

  
    private Suspension mSuspension = Suspension.getInstance();
    private double backLiftHeight=0;
    private double frontLiftHeight=0;

    public ClimbUp(double back, double front) {
        backLiftHeight=back;
        frontLiftHeight=front;
    }

    @Override
    public boolean isFinished() {
        if(mSuspension.atBackPosition()&&mSuspension.atFrontPosition()){
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
        mSuspension.setBackLiftPosition(backLiftHeight);
        mSuspension.setFrontLiftPosition(frontLiftHeight);
    }
}
*/
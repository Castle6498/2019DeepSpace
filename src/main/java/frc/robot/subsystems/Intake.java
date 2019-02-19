package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.loops.Loop;
import frc.robot.loops.Looper;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Talon;

/**
 * The suspension subsystem consists of dynamo motors that are meant to send one ball at a time into the shooter.
 * There are ir sensors placed before and after the suspension to sense blockage or emptiness of the hopper.
 * The main things this subsystem has to are feed fuel and unjam
 * 
 *
 */
public class Intake extends Subsystem {
    
   

    private static Intake sInstance = null;

    public static Intake getInstance() {
        if (sInstance == null) {
            sInstance = new Intake();
        }
        return sInstance;
    }

    private Talon mTalon; 
    private DigitalInput mPhotoeye;

    public Intake() {
        
        //Talon Initialization 
        mTalon = new Talon(Constants.kIntakeTalonChannel);
        mTalon.setInverted(true);
        
       //Photoeye Initialization
       mPhotoeye=new DigitalInput(Constants.kIntakeSensorPort);
    }

    public enum SystemState {
        IDLE,
        PICKINGUP,
        SHOOTING
    }

    private SystemState mSystemState = SystemState.IDLE;
    private SystemState mWantedState = SystemState.IDLE;

    private double mCurrentStateStartTime;
    private boolean mStateChanged;

    private Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            stop();
            synchronized (Intake.this) {
                mSystemState = SystemState.IDLE;
                mStateChanged = true;
                mCurrentStateStartTime = timestamp;
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Intake.this) {
                SystemState newState;
                switch (mSystemState) {
                case IDLE:
                    newState = handleIdle();
                    break;
                case PICKINGUP:
                    newState = handlePickingUp(timestamp);
                    break;
                case SHOOTING:
                    newState = handleShooting(timestamp);
                    break;                
                default:
                    newState = SystemState.IDLE;
                }
                if (newState != mSystemState) {
                    System.out.println("Intake state " + mSystemState + " to " + newState);
                    mSystemState = newState;
                    mCurrentStateStartTime = timestamp;
                    mStateChanged = true;
                } else {
                    mStateChanged = false;
                }
            }

            ballUpdate(timestamp);
        }

        @Override
        public void onStop(double timestamp) {
            stop();
        }
    };


    private SystemState defaultIdleTest(){
        if(mSystemState == mWantedState){
            mWantedState=SystemState.IDLE;
            return SystemState.IDLE; 
        }
        else return mWantedState;
    }

    private SystemState handleIdle() {
        if(mStateChanged){
            stopMotor();
        }
        return defaultIdleTest();
    }

    private SystemState handlePickingUp(double now){
        if(mStateChanged){
            mTalon.set(Constants.kIntakePickUpSpeed);
        }

        if(hasBall()){
            stopMotor();
            return defaultIdleTest();
        }

        return mWantedState;
    }




    private double shootStartTime=0;
    private SystemState handleShooting(double now){
        if(mStateChanged){
            mTalon.set(Constants.kIntakeShootSpeed);
            shootStartTime=0;
        }

        if(hasBall()&&shootStartTime==0){
            shootStartTime=now;
        }

        if(now-shootStartTime>=Constants.kIntakeShootPause){
            stopMotor();
            return defaultIdleTest();
        }


        return mWantedState;
    }


    public boolean seesBall(){
        return mPhotoeye.get();
    }


    private boolean mHasBall=false;
    public boolean hasBall(){
        return mHasBall;
    }

    private double ballStartSeenTime=0;
    private double ballSeenTime=0;

    private void ballUpdate(double time){
        boolean seen = seesBall();
        if(!seen){
            ballSeenTime=0;
            ballStartSeenTime=0;
            mHasBall=false;
        }else{
            if(ballStartSeenTime==0)ballStartSeenTime=time;
            ballSeenTime=time;
        } 

        if(ballSeenTime-ballStartSeenTime>=Constants.kIntakeBallRequiredTime){
            mHasBall=true;
        }
    }

   
    
 

    //Boring Stuff

        private void stopMotor(){
            mTalon.stopMotor();
        }


        public synchronized void setWantedState(SystemState state) {
            mWantedState = state;
        }

        @Override
        public void outputToSmartDashboard() {
            // SmartDashboard.putNumber("suspension_speed", mMasterTalon.get() / Constants.kIntakeSensorGearReduction);
        }

        @Override
        public void stop() {
            setWantedState(SystemState.IDLE);
        }

        @Override
        public void zeroSensors() {
        }


        @Override
        public void registerEnabledLoops(Looper in) {
            in.register(mLoop);
        }

        public boolean checkSystem() {
            System.out.println("Testing Intake.-----------------------------------");
            boolean failure=false;       
            return !failure;
        }

}
package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.loops.Loop;
import frc.robot.loops.Looper;
import frc.lib.util.drivers.Talon.CANTalonFactory;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * The suspension subsystem consists of dynamo motors that are meant to send one ball at a time into the shooter.
 * There are ir sensors placed before and after the suspension to sense blockage or emptiness of the hopper.
 * The main things this subsystem has to are feed fuel and unjam
 * 
 *
 */
public class Lift extends Subsystem {
    
   

    private static Lift sInstance = null;

    public static Lift getInstance() {
        if (sInstance == null) {
            sInstance = new Lift();
        }
        return sInstance;
    }

    private TalonSRX mTalon;
   
    public Lift() {
        
        //Talon Initialization 
        mTalon = CANTalonFactory.createTalon(Constants.kLiftTalonID, 
        false, NeutralMode.Brake, FeedbackDevice.QuadEncoder, 0, false);

        mTalon = CANTalonFactory.setupHardLimits(mTalon, LimitSwitchSource.Deactivated,
        LimitSwitchNormal.Disabled, false, LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen,true);
        
        mTalon = CANTalonFactory.setupSoftLimits(mTalon, true, (int) Math.round(Constants.kLiftSoftLimit*Constants.kLiftTicksPerInch),
        false, 0);
        
        mTalon = CANTalonFactory.tuneLoops(mTalon, 0, Constants.kLiftTalonP,
        Constants.kLiftTalonI, Constants.kLiftTalonD, Constants.kLiftTalonF);
        
       
    }

    public enum ControlState {
        IDLE,
        HOMING,
        CLOSEDLOOP,
    }

    private ControlState mControlState = ControlState.IDLE;
    private ControlState mWantedState = ControlState.IDLE;

    private double mCurrentStateStartTime;
    private boolean mStateChanged;

    private Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            stop();
            synchronized (Lift.this) {
                mControlState = ControlState.IDLE;
                mStateChanged = true;
                mCurrentStateStartTime = timestamp;
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Lift.this) {
                ControlState newState;
                switch (mControlState) {
                case IDLE:
                    newState = handleIdle();
                    break;
                case HOMING:
                    newState = handleHoming();
                    break;  
                case CLOSEDLOOP:
                    newState = handleClosedLoop();     
                default:
                    newState = ControlState.IDLE;
                }
                if (newState != mControlState) {
                    System.out.println("Lift state " + mControlState + " to " + newState);
                    mControlState = newState;
                    mCurrentStateStartTime = timestamp;
                    mStateChanged = true;
                } else {
                    mStateChanged = false;
                }
            }
        }

        @Override
        public void onStop(double timestamp) {
            stop();
        }
    };


    private ControlState defaultIdleTest(){
        if(mControlState == mWantedState) return ControlState.IDLE;
        else return mWantedState;
    }

    private ControlState handleIdle() {
        if(mStateChanged){
            stopMotor();
        }
        
       return defaultIdleTest();
    }

    private boolean hasHomed = false;

    private ControlState handleHoming(){
        if(mStateChanged){
            
            hasHomed=false;
            mTalon.set(ControlMode.PercentOutput,-.2);
            mTalon.setSelectedSensorPosition(-1);
        }

        if(!hasHomed&&mTalon.getSelectedSensorPosition()==0){
            hasHomed=true;
            System.out.println("home done");
            mSetPosition(0);
        }


        if(hasHomed){
            if(atPosition()){
            return defaultIdleTest();
            }else{
                return mWantedState;
            }
        }else{
            return ControlState.HOMING;
        }
    }

    private double mWantedPosition = 0;
    private double mTravelingPosition = 0;

    private ControlState handleClosedLoop(){
            if(mTravelingPosition!=mWantedPosition){//To keep from spamming talon
                mSetPosition(mWantedPosition);
                mTravelingPosition = mWantedPosition;
            }   
       return mWantedState;
    }
    

    //CLOSED LOOP CONTROL
    
    public synchronized void setClosedLoop(double set){
        if(mWantedState!=mControlState){
            setWantedState(ControlState.CLOSEDLOOP);
        }
        mWantedPosition=set;
    }
    public synchronized void jog(double displacement){
        setClosedLoop(displacement+mWantedPosition);
    }
     
   private synchronized void mSetPosition(double set){
       if(hasHomed){
       mTalon.set(ControlMode.Position, set*Constants.kLiftTicksPerInch);
       }else{
           System.out.println("Lift tried position without homing");
       }
   }


   public boolean atPosition(){
       //TODO: test getclosedlooptarget
      if(Math.abs(mTalon.getClosedLoopTarget()-mTalon.getSelectedSensorPosition())<=Constants.kLiftTolerance){
          return true;
      }else{
          return false;
      }
       
   }
  
   private synchronized void stopMotor(){
        mTalon.set(ControlMode.Disabled,0);
    }

    public synchronized void setWantedState(ControlState state) {
        mWantedState = state;
    }

    @Override
    public void outputToSmartDashboard() {
        // SmartDashboard.putNumber("suspension_speed", mMasterTalon.get() / Constants.kLiftSensorGearReduction);
    }

    @Override
    public void stop() {
        setWantedState(ControlState.IDLE);
    }

    @Override
    public void zeroSensors() {
    }


    @Override
    public void registerEnabledLoops(Looper in) {
        in.register(mLoop);
    }

    public boolean checkSystem() {
        System.out.println("Testing Lift.-----------------------------------");
        boolean failure=false;       
        return !failure;
    }

}
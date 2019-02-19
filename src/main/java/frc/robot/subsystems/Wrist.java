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
public class Wrist extends Subsystem {
    
   

    private static Wrist sInstance = null;

    public static Wrist getInstance() {
        if (sInstance == null) {
            sInstance = new Wrist();
        }
        return sInstance;
    }

    private TalonSRX mTalon, mTalonChild;
   
    public Wrist() {
        
        //Talon Initialization 
        mTalon = CANTalonFactory.createTalon(Constants.kWristTalonID, 
        false, NeutralMode.Brake, FeedbackDevice.Analog, 0, true);


        mTalon = CANTalonFactory.setupHardLimits(mTalon,  LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen,true,
        LimitSwitchSource.Deactivated,LimitSwitchNormal.Disabled, false);
        
        mTalon = CANTalonFactory.setupSoftLimits(mTalon, false, 0,true, -(int) Math.round(Constants.kWristSoftLimit*Constants.kWristTicksPerDeg));
        


        mTalon = CANTalonFactory.tuneLoops(mTalon, 0, Constants.kWristTalonP,
        Constants.kWristTalonI, Constants.kWristTalonD, Constants.kWristTalonF);

        setMaxOuput(Constants.kWristMaxOutput);

        
        mTalonChild = new TalonSRX(Constants.kWristChildTalonID);
        mTalonChild.setNeutralMode(NeutralMode.Brake);
        mTalonChild.setInverted(true);
        mTalonChild.follow(mTalon);
        
        
       
    }

    void setMaxOuput(double output){
        mTalon.configClosedLoopPeakOutput(0, output);
    }

    public enum ControlState {
        IDLE,
        HOMING
    }

    private ControlState mControlState = ControlState.IDLE;
    private ControlState mWantedState = ControlState.IDLE;

    private double mCurrentStateStartTime;
    private boolean mStateChanged;

    private Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            stop();
            synchronized (Wrist.this) {
                mControlState = ControlState.IDLE;
                mStateChanged = true;
                mCurrentStateStartTime = timestamp;
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Wrist.this) {
                ControlState newState;
                switch (mControlState) {
                    case IDLE:
                        newState = handleIdle();
                        break;
                    case HOMING:
                        newState = handleHoming();
                        break;   
                    default:
                        newState = ControlState.IDLE;
                    }
                if (newState != mControlState) {
                    System.out.println("Wrist state " + mControlState + " to " + newState);
                    mControlState = newState;
                    mCurrentStateStartTime = timestamp;
                    mStateChanged = true;
                } else {
                    mStateChanged = false;
                }
                positionUpdater();
            }
        }

        @Override
        public void onStop(double timestamp) {
            stop();
        }
    };

    private ControlState defaultIdleTest(){
        if(mControlState == mWantedState){
            mWantedState=ControlState.IDLE;
            return ControlState.IDLE; 
        }
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
            System.out.println("wrist home set to false");
            hasHomed=false;
            mTalon.set(ControlMode.PercentOutput,.4);
            mTalon.setSelectedSensorPosition(5000);
        }

        if(!hasHomed&&mTalon.getSelectedSensorPosition()==0){
            hasHomed=true;
            System.out.println("home done");
            setPosition(0);
        }


        if(hasHomed){
            setPosition(0);
            if(atPosition()){
            return defaultIdleTest();
            }else{
                return mWantedState;
            }
        }else{
            return ControlState.HOMING;
        }
    }


//CLOSED LOOP CONTROL
private double mWantedPosition = -.01;
private double mTravelingPosition = 0;

public synchronized void setPosition(double pos){
    if(pos<=(-Constants.kWristSoftLimit))pos=-Constants.kWristSoftLimit;
    else if(pos>0)pos=0;
    mWantedPosition=pos;
    
   // System.out.println("Set wanted pos to "+pos);
}
private boolean jog;
public void jog(double amount){
    setPosition(mWantedPosition+=amount);
    jog=true;
}


public boolean atPosition(){
  if(Math.abs(mWantedPosition-getPosition())<=Constants.kWristTolerance){
      return true;
  }else{
      return false;
  }
   
}

public double getPosition(){
   return mTalon.getSelectedSensorPosition()/Constants.kWristTicksPerDeg;
}

private void positionUpdater(){
       
if(hasHomed&&mWantedPosition!=mTravelingPosition){

    mTravelingPosition=mWantedPosition;
    /*if(!jog)*/ System.out.println("Wrist to "+mTravelingPosition+ " Position now: "+getPosition());
    mTalon.set(ControlMode.Position, mTravelingPosition*Constants.kWristTicksPerDeg);
    jog=false;
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
        System.out.println("Testing Wrist.-----------------------------------");
        boolean failure=false;       
        return !failure;
    }

}
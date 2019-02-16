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
/*
public class Suspension extends Subsystem {
    
   

    private static Suspension sInstance = null;

    public static Suspension getInstance() {
        if (sInstance == null) {
            sInstance = new Suspension();
        }
        return sInstance;
    }

    private TalonSRX mFrontLiftTalon, mBackLiftTalon,mWheelTalon ;
   
    public Suspension() {
        
      //Front Lift Initialization 
      mFrontLiftTalon = CANTalonFactory.createTalon(Constants.kSuspensionFrontLiftTalonID, 
      false, NeutralMode.Brake, FeedbackDevice.QuadEncoder, 0, false);

      mFrontLiftTalon = CANTalonFactory.setupHardLimits(mFrontLiftTalon, LimitSwitchSource.Deactivated,
      LimitSwitchNormal.Disabled, false, LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen,true);
      
      mFrontLiftTalon = CANTalonFactory.setupSoftLimits(mFrontLiftTalon, true, Constants.kSuspensionLiftSoftLimit,
      false, 0);
      
      mFrontLiftTalon = CANTalonFactory.tuneLoops(mFrontLiftTalon, 0, Constants.kSuspensionFrontLiftTalonP,
      Constants.kSuspensionFrontLiftTalonI, Constants.kSuspensionFrontLiftTalonD, Constants.kSuspensionFrontLiftTalonF);
      
      //Back Lift Initialization 
      mBackLiftTalon = CANTalonFactory.createTalon(Constants.kSuspensionBackLiftTalonID, 
      false, NeutralMode.Brake, FeedbackDevice.QuadEncoder, 0, false);

      mBackLiftTalon = CANTalonFactory.setupHardLimits(mBackLiftTalon, LimitSwitchSource.Deactivated,
      LimitSwitchNormal.Disabled, false, LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen,true);
      
      mBackLiftTalon = CANTalonFactory.setupSoftLimits(mBackLiftTalon, true, Constants.kSuspensionLiftSoftLimit,
      false, 0);
      
      mBackLiftTalon = CANTalonFactory.tuneLoops(mBackLiftTalon, 0, Constants.kSuspensionBackLiftTalonP,
      Constants.kSuspensionBackLiftTalonI, Constants.kSuspensionBackLiftTalonD, Constants.kSuspensionBackLiftTalonF);            
      
      //Wheel Initialization 
      mWheelTalon = CANTalonFactory.createTalon(Constants.kSuspensionWheelTalonID, 
      false, NeutralMode.Brake, FeedbackDevice.QuadEncoder, 0, false);

      mWheelTalon = CANTalonFactory.setupHardLimits(mWheelTalon, LimitSwitchSource.Deactivated,
      LimitSwitchNormal.Disabled, false, LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled,false);
      
      mWheelTalon = CANTalonFactory.setupSoftLimits(mWheelTalon, false, 0, false, 0);
      
      mWheelTalon = CANTalonFactory.tuneLoops(mWheelTalon, 0, Constants.kSuspensionWheelTalonP,
      Constants.kSuspensionWheelTalonI, Constants.kSuspensionWheelTalonD, Constants.kSuspensionWheelTalonF);            
      
       

    }

    public enum ControlState {
        IDLE,
        HOMING,
        MOVINGTOPOINT,
        OPENLOOP
    }

    private ControlState mControlState = ControlState.IDLE;
    private ControlState mWantedState = ControlState.IDLE;

    private double mCurrentStateStartTime;
    private boolean mStateChanged;

    private Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            stop();
            synchronized (Suspension.this) {
                mControlState = ControlState.IDLE;
                mStateChanged = true;
                mCurrentStateStartTime = timestamp;
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Suspension.this) {
                ControlState newState;
                switch (mControlState) {
                case IDLE:
                    newState = handleIdle();
                    break;
                case HOMING:
                    newState = handleHoming();
                    break;       
                case MOVINGTOPOINT:
                    newState = handleMovingToPoint();
                    break;
                case OPENLOOP:
                    newState = handleOpenLoop();
                    break;
                default:
                    newState = ControlState.IDLE;
                }
                if (newState != mControlState) {
                    System.out.println("Suspension state " + mControlState + " to " + newState);
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

    //Handlers
        private ControlState defaultIdleTest(){
            if(mControlState == mWantedState) return ControlState.IDLE;
            else return mWantedState;
        }

        
        private ControlState handleIdle() {
            if(mStateChanged){
                stopMotors();
            }
            
            return defaultIdleTest();
        }


        private double mOpenLoopFrontSetpoint=0;
        private double mOpenLoopBackSetpoint=0;
        private double mOpenLoopWheelSetpoint=0;
        private ControlState handleOpenLoop(){
            //Don't know about homing here?
            //Could just rely that it was alread down to begin with
            mBackLiftTalon.set(ControlMode.PercentOutput,mOpenLoopBackSetpoint);
            mFrontLiftTalon.set(ControlMode.PercentOutput,mOpenLoopFrontSetpoint);
            mWheelTalon.set(ControlMode.PercentOutput,mOpenLoopWheelSetpoint);
            //To make sure that it turns off if changing states
            if(mWantedState!=ControlState.OPENLOOP) stopMotors();

            return mWantedState;
        }

        private boolean hasHomed = false;

        private ControlState handleHoming(){
        if(mStateChanged){
            //mTalon.set(ControlMode.PercentOutput,-.2);
        } 

        //TODO: Homing procedure
        
        //Don't let it move to go to point if it has not homed yet
        if(hasHomed){
            return defaultStateTransfer();
            }else{
                return ControlState.HOMING;
            }
        }

        private double mWantedFrontPosition = 0;
        private double mTravelingFrontPosition = 0;

        private double mWantedBackPosition = 0;
        private double mTravelingBackPosition = 0;

        private double mWantedWheelPosition = 0;
        private double mTravelingWheelPosition = 0;

        private ControlState handleMovingToPoint(){
            if(hasHomed){
                if(mStateChanged){
                    mSetFrontLiftPosition(mWantedFrontPosition);
                    mSetBackLiftPosition(mWantedBackPosition);
                    mSetWheelPosition(mWantedWheelPosition);
                    mTravelingFrontPosition = mWantedFrontPosition;
                    mTravelingBackPosition = mWantedBackPosition;
                    mTravelingWheelPosition = mWantedWheelPosition;
                }

                if(mTravelingFrontPosition!=mWantedFrontPosition){
                    mSetFrontLiftPosition(mWantedFrontPosition);
                    mTravelingFrontPosition = mWantedFrontPosition;
                }
                if(mTravelingBackPosition!=mWantedBackPosition){
                    mSetBackLiftPosition(mWantedBackPosition);
                    mTravelingBackPosition = mWantedBackPosition;
                }
                if(mTravelingWheelPosition!=mWantedWheelPosition){
                    mSetWheelPosition(mWantedWheelPosition);
                    mTravelingWheelPosition = mWantedWheelPosition;
                }
                
                if(atFrontPosition()&&atBackPosition()&&atWheelPosition()){
                    return defaultStateTransfer();
                }else{
                    return ControlState.IDLE;
                }

            }else{
                System.out.println("Tried to move without homing!");
                return ControlState.IDLE;
            }

        
        }


    
    //Open Loop Control
        public synchronized void setOpenLoop(double frontLift, double backLift, double wheel){
            mWantedState=WantedState.OPENLOOP;
            mOpenLoopBackSetpoint=backLift;
            mOpenLoopFrontSetpoint=frontLift;
            mOpenLoopWheelSetpoint=wheel;
        } 
        public synchronized void setBackLiftOpenLoop(double backLift){
            setOpenLoop(mOpenLoopFrontSetpoint,backLift, mOpenLoopWheelSetpoint);
        }
        public synchronized void setFrontLiftOpenLoop(double frontLift){
            setOpenLoop(frontLift,mOpenLoopBackSetpoint, mOpenLoopWheelSetpoint);
        }
        public synchronized void setWheelOpenLoop(double wheel){
            setOpenLoop(mOpenLoopFrontSetpoint,mOpenLoopBackSetpoint, wheel);
        }
        public synchronized void setLiftOpenLoop(double frontLift, double backLift){
            setOpenLoop(frontLift,backLift, mOpenLoopWheelSetpoint);
        }


    //Position Control

        //Private Individual set point 
            private synchronized void mSetBackLiftPosition(double set){
                if(hasHomed){
                mBackLiftTalon.set(ControlMode.Position, set*Constants.kSuspensionLiftTicksPerInch);
                }else{
                    System.out.println("Suspension back tried to move without homing");
                }
            }
            private synchronized void mSetFrontLiftPosition(double set){
                    if(hasHomed){
                    mBackLiftTalon.set(ControlMode.Position, set*Constants.kSuspensionLiftTicksPerInch);
                    }else{
                        System.out.println("Suspension front tried to move without homing");
                    }
                }
            private synchronized void mSetWheelPosition(double set){
                
                mBackLiftTalon.set(ControlMode.Position, set*Constants.kSuspensionWheelTicksPerInch);
            
            }

        //Public set points
            public synchronized void setBackLiftPosition(double set){
                
            }
            public synchronized void setFrontLiftPosition(double set){
                    
            }
            public synchronized void setWheelPosition(double set){
                
            }


        

        public synchronized boolean atFrontPosition(){
            //TODO: test getclosedlooptarget
            if(Math.abs(mFrontLiftTalon.getClosedLoopTarget()-mFrontLiftTalon.getSelectedSensorPosition())<=Constants.kSuspensionLiftTolerance){
                return true;
            }else{
                return false;
            }
            
        }
        public synchronized boolean atBackPosition(){
            //TODO: test getclosedlooptarget
            if(Math.abs(mBackLiftTalon.getClosedLoopTarget()-mBackLiftTalon.getSelectedSensorPosition())<=Constants.kSuspensionLiftTolerance){
                return true;
            }else{
                return false;
            }
            
        }
        public synchronized boolean atWheelPosition(){
            //TODO: test getclosedlooptarget
            if(Math.abs(mWheelTalon.getClosedLoopTarget()-mWheelTalon.getSelectedSensorPosition())<=Constants.kSuspensionWheelTolerance){
                return true;
            }else{
                return false;
            }
            
        }
    
    //Boring overrides
        private synchronized void stopMotors(){
            mBackLiftTalon.set(ControlMode.Disabled,0);
            mFrontLiftTalon.set(ControlMode.Disabled,0);
            mWheelTalon.set(ControlMode.Disabled,0);
        }
        public synchronized void setWantedState(WantedState state) {
            mWantedState = state;
        }

        @Override
        public void outputToSmartDashboard() {
            // SmartDashboard.putNumber("suspension_speed", mMasterTalon.get() / Constants.kSuspensionSensorGearReduction);
        }

        @Override
        public void stop() {
            setWantedState(WantedState.IDLE);
        }

        @Override
        public void zeroSensors() {
        }


        @Override
        public void registerEnabledLoops(Looper in) {
            in.register(mLoop);
        }

        public boolean checkSystem() {
            System.out.println("Testing Suspension.-----------------------------------");
            boolean failure=false;       
            return !failure;
        }

}*/
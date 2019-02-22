package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import frc.lib.util.drivers.Talon.CANTalonFactory;
import frc.robot.CameraVision;
import frc.robot.Constants;
import frc.robot.CameraVision.CameraMode;
import frc.robot.CameraVision.LightMode;
import frc.robot.loops.Loop;
import frc.robot.loops.Looper;

/**
 * The plateCenter subsystem consists of dynamo motors that are meant to send
 * one ball at a time into the shooter. There are ir sensors placed before and
 * after the plateCenter to sense blockage or emptiness of the hopper. The main
 * things this subsystem has to are feed fuel and unjam
 * 
 *
 */
public class PlateCenter extends Subsystem {
    
   


    private static PlateCenter sInstance = null;

    public static PlateCenter getInstance() {
        if (sInstance == null) {
            sInstance = new PlateCenter();
        }
        return sInstance;
    }

    private TalonSRX mBeltTalon;
    private final Solenoid mSuckSolenoid, mHardStopYeeYeeSolenoid;
    private final DoubleSolenoid mDeploySolenoid;
  
    DigitalInput mLidar;

    Compressor compressor;
 

    
    public PlateCenter() {
       

        //Configure Talon
            mBeltTalon = CANTalonFactory.createTalon(Constants.kPlateCenterTalonID,
            false, NeutralMode.Brake, FeedbackDevice.QuadEncoder, 0, false);

            mBeltTalon = CANTalonFactory.setupHardLimits(mBeltTalon, LimitSwitchSource.Deactivated,
            LimitSwitchNormal.Disabled, false, LimitSwitchSource.FeedbackConnector,
            LimitSwitchNormal.NormallyOpen, true);

            mBeltTalon = CANTalonFactory.setupSoftLimits(mBeltTalon, true, (int) Math.round(Constants.kPlateCenterTalonSoftLimit*Constants.kPlateCenterTicksPerInch),
            false, 0);

            mBeltTalon = CANTalonFactory.tuneLoops(mBeltTalon, 0, Constants.kPlateCenterTalonP,
            Constants.kPlateCenterTalonI, Constants.kPlateCenterTalonD, Constants.kPlateCenterTalonF);
  
        //LIDAR
            mLidar = new DigitalInput(Constants.kPlateCenterLidar);
           
        //Pneumatics        
            mSuckSolenoid = new Solenoid(Constants.kPlateCenterSuckSolenoidPort);
            mDeploySolenoid = new DoubleSolenoid(Constants.kPlateCenterDeploySolenoidPort[0],Constants.kPlateCenterDeploySolenoidPort[1]);
            mHardStopYeeYeeSolenoid = new Solenoid(Constants.kPlateCenterHardStopYeeYeeSolenoidPort);

            compressor = new Compressor();

        System.out.println("Plate initialized");
    }

    public enum SystemState {
        IDLE, 
        CENTERING, 
        AUTOALIGNING, 
        DEPLOYINGPLATE,
        HOMING
    }

    private SystemState mSystemState = SystemState.IDLE;
    private SystemState mWantedState = SystemState.IDLE;

    private double mCurrentStateStartTime;
    private boolean mStateChanged;

    private Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            stop();
            compressor.start();
            synchronized (PlateCenter.this) {
                System.out.println("Plate onStart");
                mSystemState = SystemState.IDLE;
                mStateChanged = true;
                mCurrentStateStartTime = timestamp;
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (PlateCenter.this) {
                SystemState newState;
                switch (mSystemState) {
                case IDLE:
                    newState = handleIdle();
                    break;
                case CENTERING:
                    newState = handleCentering();
                    break;                
                case AUTOALIGNING:
                    newState = handleAutoAligning();
                    break;  
                case DEPLOYINGPLATE:
                    newState = handleDeployingPlate(timestamp, mCurrentStateStartTime);
                    break;  
                case HOMING:
                    newState = handleHoming();
                    break;
                default:
                    newState = SystemState.IDLE;
                }
                if (newState != mSystemState) {
                    System.out.println("PlateCenter state " + mSystemState + " to " + newState);
                    mSystemState = newState;
                    mCurrentStateStartTime = timestamp;
                    mStateChanged = true;
                } else {
                    mStateChanged = false;
                }
                positionUpdater();
            }
          //  System.out.println("Plate Loop");
        }

        @Override
        public void onStop(double timestamp) {
            stop();
        }
    };  //LOOP SET ENDS HERE


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
            resetPistons();
        }
       
        return defaultIdleTest();
    }
    
    private boolean hasHomed = false;
    private SystemState handleHoming(){
        if(mStateChanged){
            System.out.println("plate Home Plate false");
            hasHomed=false;
            mBeltTalon.set(ControlMode.PercentOutput,-.7);
            mBeltTalon.setSelectedSensorPosition(-500);
        }

        if(!hasHomed&&mBeltTalon.getSelectedSensorPosition()==0){
            hasHomed=true;
            System.out.println("plate home done, set to middle");
            mTravelingSetPosition=0;
            setPosition(Constants.kPlateCenterTalonSoftLimit/2);
        }
       // System.out.println("Current pos: "+getPosition()+ " "+mWantedSetPosition);

        if(hasHomed){
            if(atPosition()){
            return defaultIdleTest();
            }else{
                //System.out.println("Wanted state: "+mWantedState);
                return mWantedState;
            }
        }else{
            return SystemState.HOMING;
        }
    }

    double inchesToCenter;
    boolean plateCentered=false;

    enum CenteringState {FARLIMIT, SENSE, DONE};
    CenteringState centeringState = CenteringState.FARLIMIT;
    //DigitalInput senseZero = new DigitalInput(0);
    TalonSRX exTal = new TalonSRX(8);

    private SystemState handleCentering() {
        if(mStateChanged){
            System.out.println("Centering");
            setPosition(0);
            centeringState = CenteringState.FARLIMIT;
            plateCentered=false;
        }    
       
        switch(centeringState){
            case FARLIMIT:
            if(atPosition()){
                mBeltTalon.set(ControlMode.PercentOutput,Constants.kPlateCenterCenteringSpeed);
                centeringState=CenteringState.SENSE;
            }
            case SENSE:
                if(!getLidar()){
                    stopMotor();
                    inchesToCenter = getPosition() - Constants.kPlateCenterTalonSoftLimit/2; 
                    System.out.println("centered succesfully");
                    centeringState = CenteringState.DONE;
                    plateCentered=true;
                }else if(getPosition()>=Constants.kPlateCenterTalonSoftLimit) {
                    System.out.println("center failed");
                    centeringState = CenteringState.DONE;
                    plateCentered=false;
                }
            
        }

        if(mWantedState!=SystemState.CENTERING){
            stopMotor();
            centeringState=CenteringState.DONE;
        }

        if(centeringState==CenteringState.DONE)return defaultIdleTest(); 
        else return mWantedState;
    }
    
    private SystemState handleDeployingPlate(double now, double startStartedAt) {
        
        if (mStateChanged) {
           stopMotor();
           System.out.println("Deploying Plate");    
            //suck(true);
        }

        double elapsedTime = now - startStartedAt;
        System.out.println(elapsedTime);
        
        if (elapsedTime > Constants.kPlateCenterDeployPauses[0]&&
        elapsedTime<= Constants.kPlateCenterDeployPauses[1]) {
            //suck(true);
            push(true);
            plateCentered=false;
            //System.out.println("Suck and Push 1");
        } else if(elapsedTime > Constants.kPlateCenterDeployPauses[1]&&
        elapsedTime<= Constants.kPlateCenterDeployPauses[2]) {
            suck(false);
            push(true);
           // System.out.println("unsuck 2");
        }else if(elapsedTime > Constants.kPlateCenterDeployPauses[2]&&
        elapsedTime<= Constants.kPlateCenterDeployPauses[3]) {
            //System.out.println("unpush 3");
            suck(false);
            push(false);
        }else if(elapsedTime>=Constants.kPlateCenterDeployPauses[3]){
           return defaultIdleTest(); 
        }
        
        return SystemState.DEPLOYINGPLATE;
       
    }

    private SystemState handleAutoAligning(){
        boolean ready=false;

        if(plateCentered){
        
            if(mStateChanged){
                CameraVision.setCameraMode(CameraMode.eVision);
                CameraVision.setPipeline(0);
                CameraVision.setLedMode(LightMode.eOn);
            }

            //Find the offset of target from camera with 0 at middle
            double target = Constants.kLimeLightDistancetoTarget*Math.tan(Math.toRadians(CameraVision.getTx()));

            //Find set point for motor with 0 on the left 
            target = Constants.kPlateCenterTalonSoftLimit/2 - 
                    Constants.kLimeLightDistanceFromCenter - target +inchesToCenter;

            setPosition(target);
        
            ready = atPosition();

            SystemState newState=mWantedState;
            
            if(ready&&Constants.kLimeLightAutoDeploy) newState = SystemState.DEPLOYINGPLATE;

            if(newState != SystemState.AUTOALIGNING){
                CameraVision.setCameraMode(CameraMode.eDriver);
                CameraVision.setPipeline(1);
                CameraVision.setLedMode(LightMode.eOff);
            }

            return newState;
        
        } else return defaultIdleTest();

    }
    
   
    //POSITION CONTROL
        private double mWantedSetPosition=.1;
        private double mTravelingSetPosition=0;

        public synchronized void setPosition(double pos){
            if(pos>=Constants.kPlateCenterTalonSoftLimit)pos=Constants.kPlateCenterTalonSoftLimit;
            else if(pos<0)pos=0;
            mWantedSetPosition=pos;
            
           // System.out.println("Set wanted pos to "+pos);
        }

        public synchronized void jog(double amount){
            setPosition(mWantedSetPosition+=amount);
        }

        public double getPosition(){
            return mBeltTalon.getSelectedSensorPosition()/Constants.kPlateCenterTicksPerInch;
        }

        public boolean atPosition(){
           if(Math.abs(mWantedSetPosition-getPosition())<=Constants.kPlateCenterTalonTolerance){
               return true;
           }else{
               return false;
           }
        }

        private void positionUpdater(){
           
            if(hasHomed&&mWantedSetPosition!=mTravelingSetPosition){

                mTravelingSetPosition=mWantedSetPosition;
               // System.out.println("set position: "+mTravelingSetPosition);
                mBeltTalon.set(ControlMode.Position, mTravelingSetPosition*Constants.kPlateCenterTicksPerInch);
            }
        }

    //Sensors
        public boolean getLidar(){
            return mLidar.get();
        }

    //Pneumatic Controls
        private void suck(boolean s){
           mSuckSolenoid.set(s);
           System.out.println("Suck solenoid: "+s);
        }
        private void push(boolean p){
            if(p) mDeploySolenoid.set(DoubleSolenoid.Value.kForward);
            else mDeploySolenoid.set(DoubleSolenoid.Value.kReverse);
          System.out.println("Push solenoid: "+p);
        }
        public void hardStop(boolean h){
          mHardStopYeeYeeSolenoid.set(h);
        // System.out.println("HardStop solenoid: "+h);
        }
        private void resetPistons(){
            hardStop(false);
            suck(false);
            push(false);
        }

    //Boring Stuff

        private void stopMotor(){
            mBeltTalon.set(ControlMode.Disabled,0);
        }

        public synchronized void setWantedState(SystemState state) {
            mWantedState = state;
        }
 
        public boolean checkSystem() {
            System.out.println("Testing FEEDER.-----------------------------------");
            boolean failure=false;       
            return !failure;
        }

        @Override
        public void outputToSmartDashboard() {

        }

        @Override
        public void stop() {
            compressor.stop();
            setWantedState(SystemState.IDLE);
        }

        @Override
        public void zeroSensors() {

        }

        @Override
        public void registerEnabledLoops(Looper in) {
            in.register(mLoop);
        }

}
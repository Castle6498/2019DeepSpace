package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.Ultrasonic;
import frc.lib.util.drivers.Talon.CANTalonFactory;
import frc.robot.CameraVision;
import frc.robot.Constants;
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
    DigitalInput mLidarOne;
    DigitalInput mLidarTwo;
    DigitalInput mLidarThree;
    double distanceFromRightBound;
    double distanceFromCenter;
    double distanceFromObject;
   CameraVision Limelight = new CameraVision();

    private static PlateCenter sInstance = null;

    public static PlateCenter getInstance() {
        if (sInstance == null) {
            sInstance = new PlateCenter();
        }
        return sInstance;
    }

    private TalonSRX mBeltTalon;
   // private final Solenoid mSuckSolenoid, mDeploySolenoid, mHardStopYeeYeeSolenoid;
   // private final Ultrasonic mTriggerOutsideLeft, mTriggerOutsideRight, mTriggerInsideLeft, mTriggerInsideRight;
//TODO use dio instead of ultrasonic for lazzzers
    public enum SensorSide {
        OUTSIDELEFT,
        OUTSIDERIGHT,//TODO eliminate one of the outsides
        INSIDELEFT,
        INSIDERIGHT
    }

    
    public PlateCenter() {
       

        //Configure Talon
        mBeltTalon = CANTalonFactory.createTalon(Constants.kPlateCenterTalonID,
        false, NeutralMode.Brake, FeedbackDevice.QuadEncoder, 0, false);

        mBeltTalon = CANTalonFactory.setupHardLimits(mBeltTalon, LimitSwitchSource.Deactivated,
        LimitSwitchNormal.Disabled, false, LimitSwitchSource.FeedbackConnector,
        LimitSwitchNormal.NormallyOpen, true);

        mBeltTalon = CANTalonFactory.setupSoftLimits(mBeltTalon, true, Constants.kPlateCenterTalonSoftLimit,
        false, 0);

        mBeltTalon = CANTalonFactory.tuneLoops(mBeltTalon, 0, Constants.kPlateCenterTalonP,
        Constants.kPlateCenterTalonI, Constants.kPlateCenterTalonD, Constants.kPlateCenterTalonF);
  
        //LIDAR
        mLidarOne = new DigitalInput(Constants.kPlateCenterLeftLidar);
        mLidarTwo = new DigitalInput(Constants.kPlateCenterCenterLidar);
        mLidarThree = new DigitalInput(Constants.kPlateCenterRightLidar);
      /*  mSuckSolenoid = new Solenoid(Constants.kPlateCenterSuckSolenoidPort);
        mDeploySolenoid = new Solenoid(Constants.kPlateCenterDeploySolenoidPort);
        mHardStopYeeYeeSolenoid = new Solenoid(Constants.kPlateCenterHardStopYeeYeeSolenoidPort);

        mTriggerOutsideLeft = new Ultrasonic(Constants.kPlateCenterOustideLeftSensorPin[0], Constants.kPlateCenterOustideLeftSensorPin[1]);
        mTriggerOutsideRight = new Ultrasonic(Constants.kPlateCenterOutsideRightSensorPin[0], Constants.kPlateCenterOutsideRightSensorPin[1]);
        mTriggerInsideLeft = new Ultrasonic(Constants.kPlateCenterInsideLeftSensorPin[0], Constants.kPlateCenterInsideLeftSensorPin[1]);
        mTriggerInsideRight = new Ultrasonic(Constants.kPlateCenterInsideRightSensorPin[0], Constants.kPlateCenterInsideRightSensorPin[1]);

*/
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
        }

        @Override
        public void onStop(double timestamp) {
            stop();
        }
    };  //LOOP SET ENDS HERE

    private SystemState handleIdle() {
        if(mStateChanged){
            stopMotor();
            resetPistons();
        }
       
        return mWantedState;
    }
    
    private boolean hasHomed = false;
    private SystemState handleHoming(){
        if(mStateChanged){
            
            hasHomed=false;
            mBeltTalon.set(ControlMode.PercentOutput,-.2);
            mBeltTalon.setSelectedSensorPosition(-1);
        }

        if(!hasHomed&&mBeltTalon.getSelectedSensorPosition()==0){
            hasHomed=true;
            System.out.println("home done");
            setPosition(2);
        }


        if(hasHomed){
        
        return mWantedState;
        }else{
            return SystemState.HOMING;
        }
    }
    public enum DiscState{
        CENTER, MOVELEFT, MOVERIGHT
      }
      DiscState discOrient;
      boolean leftRange;
      boolean rightRange;
      boolean center;
      DigitalInput senseZero = new DigitalInput(0);
      DigitalInput senseOne = new DigitalInput(1);
      DigitalInput senseTwo = new DigitalInput(2);
      TalonSRX exTal = new TalonSRX(8);

    private SystemState handleCentering() {
        if(mStateChanged){
            System.out.println("Centering");
        }
     
        //TODO: Plate Center Code Here
        //plate centering
        leftRange = senseZero.get();
        center = senseOne.get();
        rightRange = senseTwo.get();
        if(center){
            discOrient = DiscState.CENTER;
            System.out.println("CENTER");
        }
        else  if(leftRange){ // move left
            discOrient = DiscState.MOVELEFT;
            System.out.println("Move LEFT");
        }
        else  if(rightRange){ // move right
            discOrient = DiscState.MOVERIGHT;
            System.out.println("Move RIGHT");
        }

       return mWantedState;
    }
    
    private SystemState handleDeployingPlate(double now, double startStartedAt) {
        
        if (mStateChanged) {
           stopMotor();
           System.out.println("Deploying Plate");    
            suck(true);
        }

        double elapsedTime = now - startStartedAt;
        if (elapsedTime > Constants.kPlateCenterDeployPauses[0]&&
        elapsedTime<= Constants.kPlateCenterDeployPauses[1]) {
            suck(true);
            push(true);
        } else if(elapsedTime > Constants.kPlateCenterDeployPauses[1]&&
        elapsedTime<= Constants.kPlateCenterDeployPauses[2]) {
            suck(false);
            push(true);
        }else if(elapsedTime > Constants.kPlateCenterDeployPauses[2]&&
        elapsedTime<= Constants.kPlateCenterDeployPauses[3]) {
            suck(false);
            push(false);
        }else{
           return mWantedState; //TODO: Calibrate times in constants
        }
        
        return SystemState.DEPLOYINGPLATE;
       
    }

    private SystemState handleAutoAligning(){
        //TODO: KADEN GET YOUR CRAP
        //Need some sort of command to get the inches
        //from your vision class
        
            
            if(mLidarTwo.get() == true | mLidarOne.get() == true | mLidarThree.get() == true){
                distanceFromCenter= Math.tan(Limelight.x)*24;
                distanceFromRightBound= -distanceFromCenter + (Constants.kSuspensionLiftSoftLimit/2)/Constants.kPlateCenterTicksPerInch;
                
                if(Limelight.x == 0){
                stopMotor();
    
                }
                if(Limelight.x != 0){
    
               setPosition(distanceFromRightBound);
                }
            }
            leftRange = senseZero.get();
            center = senseOne.get();
            rightRange = senseTwo.get();          
        
            if(leftRange) {
              discOrient = DiscState.MOVELEFT;
              exTal.set(ControlMode.PercentOutput, .5);
            }
            else if (rightRange) {
              discOrient = DiscState.MOVERIGHT;
              exTal.set(ControlMode.PercentOutput, -.5);
            }
            else if (center) {
              discOrient = DiscState.CENTER;
              exTal.set(ControlMode.PercentOutput, 0);
            }
    
        setPosition(0);

        return mWantedState;
    }
    
   
    //POSITION CONTROL
        private final double slideMiddlePoint = Constants.kPlateCenterTalonSoftLimit/Constants.kPlateCenterTicksPerInch/2;
        private double mWantedSetPosition=.1;
        private double mTravelingSetPosition=0;

        public synchronized void setPosition(double pos){
            mWantedSetPosition=pos;
            System.out.println("Set wanted pos to "+pos);
        }

        public void jog(double amount){
            setPosition(mWantedSetPosition+=amount);
        }

        public double getPosition(){
            return mBeltTalon.getSelectedSensorPosition()/Constants.kPlateCenterTalonSoftLimit;
        }

        private void positionUpdater(){
           
            if(hasHomed&&mWantedSetPosition!=mTravelingSetPosition){

                mTravelingSetPosition=mWantedSetPosition;
                System.out.println("set position: "+mTravelingSetPosition);
                mBeltTalon.set(ControlMode.Position, mTravelingSetPosition*Constants.kPlateCenterTicksPerInch);
            }
        }

    //Pneumatic Controls
        private void suck(boolean s){
           // mSuckSolenoid.set(s);
        }
        private void push(boolean p){
           // mDeploySolenoid.set(p);
        }
        private void hardStop(boolean h){
          //  mHardStopYeeYeeSolenoid.set(h);
        }
        private void resetPistons(){
            hardStop(false);
            suck(false);
            push(false);
        }
    
    //Sensor Controls
        public boolean getAnySensor(){
            for(SensorSide sensorSide : SensorSide.values()){
                if(getSensor(sensorSide)) return true;                
            }
            return false;
        }

        public boolean getSensor(SensorSide side){
            switch(side){
              /*  case OUTSIDELEFT: //TODO update for DIO
                    return getSpecificSensor(mTriggerOutsideLeft);
                case OUTSIDERIGHT:
                    return getSpecificSensor(mTriggerOutsideRight);
                case INSIDELEFT:
                    return getSpecificSensor(mTriggerInsideLeft);
                case INSIDERIGHT:
                    return getSpecificSensor(mTriggerInsideRight);*/
                default: 
                    return false;
            }
        }

        public boolean getSpecificSensor(Ultrasonic sensor){
            double distance = sensor.getRangeInches();
            if(distance>=Constants.kPlateCenterSensorThreshold[0]&&
            distance<=Constants.kPlateCenterSensorThreshold[1]){
            return true;
            }else return false;
        }

    //Boring Stuff

        private void stopMotor(){
            mBeltTalon.set(ControlMode.Disabled,0);
        }

        public synchronized void setWantedState(SystemState state) {
            mWantedState = state;
        }

        @Override
        public void outputToSmartDashboard() {
            // SmartDashboard.putNumber("plateCenter_speed", mMasterTalon.get() / Constants.kPlateCenterSensorGearReduction);
        }

        @Override
        public void stop() {
            setWantedState(SystemState.IDLE);
            stopMotor();
            resetPistons();
        }

        @Override
        public void zeroSensors() {
        
        }

        @Override
        public void registerEnabledLoops(Looper in) {
            in.register(mLoop);
        }

        public boolean checkSystem() {
            System.out.println("Testing FEEDER.-----------------------------------");
            boolean failure=false;       
            return !failure;
        }

}
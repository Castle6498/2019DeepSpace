package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;

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
    
    DigitalInput mLidarOne, mLidarTwo;
    double distanceFromRightBound;
    double distanceFromCenter;
    double distanceFromObject;
   CameraVision mLimeLight;


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
        mLidarTwo = new DigitalInput(Constants.kPlateCenterRightLidar);
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

    boolean leftRange, rightRange;
    double inchesToCenter;
    //DigitalInput senseZero = new DigitalInput(0);
    DigitalInput senseOne = new DigitalInput(1); // second (right)
    DigitalInput senseTwo = new DigitalInput(2); //first (left)
    TalonSRX exTal = new TalonSRX(8);

    private SystemState handleCentering() {
        if(mStateChanged){
            System.out.println("Centering");
        }    
        //TODO: Plate Center Code Here
        //plate centering

        setPosition(0); //You have to wait until it gets to the actual position use
        //Like use atPosition() to know if it is moved, then move forward to percent output
        leftRange = senseTwo.get();      
        rightRange = senseOne.get();
        if(leftRange && rightRange){
            stopMotor();
            inchesToCenter = getPosition() - slideMiddlePoint; //center point
        }
        else {
            exTal.set(ControlMode.PercentOutput, .5);
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
        
            
        if(mStateChanged){
            distanceFromCenter= Math.tan(mLimeLight.getX())*24;
            distanceFromRightBound= -distanceFromCenter + (Constants.kSuspensionLiftSoftLimit/2)/Constants.kPlateCenterTicksPerInch;
            
            if(mLimeLight.getX() == 0){ //Kaden its basically never going to be 0, just a tiny bit off
            stopMotor();

            }
            if(mLimeLight.getX() != 0){

           setPosition(distanceFromRightBound);
            }
        }
        
        //Why would you have this line ? :
        //setPosition(0);

        /*You are going to have to sense when it is at the correct position
        *use the atPosition() method to see when it is at the current setPosition setpoint
        *This is where you need to leave space for the light code (just leave a TODO statement like in 260)
        */

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

        public boolean atPosition(){
           if(Math.abs(mBeltTalon.getClosedLoopTarget()-mBeltTalon.getSelectedSensorPosition())<=Constants.kPlateCenterTalonTolerance){
               return true;
           }else{
               return false;
           }
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

        }

        @Override
        public void zeroSensors() {

        }

        @Override
        public void registerEnabledLoops(Looper enabledLooper) {

        }

}
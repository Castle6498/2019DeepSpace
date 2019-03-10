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
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.lib.util.drivers.Talon.CANTalonFactory;
import frc.robot.CameraVision;
import frc.robot.Constants;
import frc.robot.ControlBoard;
import frc.robot.ControlBoardInterface;
import frc.robot.CameraVision.CameraMode;
import frc.robot.CameraVision.LightMode;
import frc.robot.ControlBoardInterface.Controller;
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
    
   
    private ControlBoardInterface mControlBoard = ControlBoard.getInstance();

    private static PlateCenter sInstance = null;

    public static PlateCenter getInstance() {
        if (sInstance == null) {
            sInstance = new PlateCenter();
        }
        return sInstance;
    }

    private TalonSRX mBeltTalon;
    private final Solenoid mSuckSolenoid, mHardStopYeeYeeSolenoid;
    private final DoubleSolenoid mDeploySolenoid, mVaccuumReleaseSolenoid;
  
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
            mVaccuumReleaseSolenoid = new DoubleSolenoid(Constants.kPlateVaccuumReleaseSolenoidPort[0],Constants.kPlateVaccuumReleaseSolenoidPort[1]);


            compressor = new Compressor();

        System.out.println("Plate initialized");
    }

    private void setLimitClear(boolean e){
        mBeltTalon.configClearPositionOnLimitR(e,0);    
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
                    newState = handleAutoAligning(timestamp, mCurrentStateStartTime);
                    break;  
                case DEPLOYINGPLATE:
                    newState = handleDeployingPlate(timestamp, mCurrentStateStartTime);
                    break;  
                case HOMING:
                    newState = handleHoming(timestamp, mCurrentStateStartTime);
                    break;
                default:
                    newState = SystemState.IDLE;
                }
                if (newState != mSystemState) {
                    System.out.println("PlateCenter state " + mSystemState + " to " + newState + " "+timestamp);
                    mSystemState = newState;
                    mCurrentStateStartTime = timestamp;
                    mStateChanged = true;
                } else {
                    mStateChanged = false;
                }
                if(hasHomed)positionUpdater();
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
            //resetPistons();
            
        }
       
        return defaultIdleTest();
    }
    
    private boolean hasHomed = false;

    

    private SystemState handleHoming(double now, double startedAt){
        if(mStateChanged){
            hasHomed=false;
            setLimitClear(true);
            mBeltTalon.set(ControlMode.PercentOutput,-.7);
            mBeltTalon.setSelectedSensorPosition(-500);
        }

        if((now-startedAt)>2.5) {
            System.out.println("plate reset triggered");
            stopMotor();
            return SystemState.IDLE;
        }
        


        if(!hasHomed&&mBeltTalon.getSelectedSensorPosition()==0){
            hasHomed=true;
            mTravelingSetPosition=.1;
            System.out.println("plate has Homed");
            setPosition(Constants.kPlateCenterTalonSoftLimit/2);
        }
       // System.out.println("Current pos: "+getPosition()+ " "+mWantedSetPosition);

       SystemState newState;

       if(hasHomed){
           if(atPosition()){
           newState= defaultIdleTest();
           }else{
               newState= mWantedState;
           }
       }else{
           newState= SystemState.HOMING;
       }

       if(newState!=SystemState.HOMING)setLimitClear(false);
       return newState;
   }

   double inchesToCenter;
   boolean plateCentered=false;

   enum CenteringState {FARLIMIT, SENSE, DONE};
   CenteringState centeringState = CenteringState.FARLIMIT;

   private SystemState handleCentering() {
       if(mStateChanged){
           System.out.println("Centering");
           setPosition(0);
           centeringState = CenteringState.FARLIMIT;
           plateCentered=false;
           mTravelingSetPosition=.1;
           hardStop(false);
           vacRelease(true);
           suck(true);
       }    
       
      System.out.println("Lidar: "+getLidar()+ " State: "+centeringState);

       switch(centeringState){
           case FARLIMIT:
           if(atPosition()){
               mBeltTalon.set(ControlMode.PercentOutput,Constants.kPlateCenterCenteringSpeed);
               centeringState=CenteringState.SENSE;
           }
           break;
           case SENSE:
               if(getLidar()){
                   stopMotor();
                   inchesToCenter = getPosition() - Constants.kPlateCenterTalonSoftLimit/2; 
                   System.out.println("centered succesfully, inches to center: "+inchesToCenter);
                   centeringState = CenteringState.DONE;
                   plateCentered=true;
               }else if(getPosition()>=Constants.kPlateCenterTalonSoftLimit&&centeringState!=CenteringState.FARLIMIT) {
                   System.out.println("center failed");
                   centeringState = CenteringState.DONE;
                   plateCentered=false;
               }
           break;
       }

       if(mWantedState!=SystemState.CENTERING){
           stopMotor();
           centeringState=CenteringState.DONE;
       }

       if(centeringState==CenteringState.DONE){
           hardStop(true);
           return defaultIdleTest(); 
       }
       else return mWantedState;
   }
    
   enum PlateDeployState {VacOn, Suck, Push, vacOff, Retract, FinalReset, Done}

   private PlateDeployState deployState = PlateDeployState.VacOn;
   private double lastStateStart=0;
   private double elapsedStateTime=0;

    private SystemState handleDeployingPlate(double now, double startStartedAt) {
        
        if (mStateChanged) {
           stopMotor();
           System.out.println("Deploying Plate");  
            deployState= PlateDeployState.VacOn;
            lastStateStart=now;
           
        }


         elapsedStateTime = now - lastStateStart;
        
       
        PlateDeployState newState=deployState;
        switch(deployState){
            case VacOn:
                vacRelease(true);
                push(false);
                if(deployUpdate(Constants.kPlateCenterDeployPauses[0])) 
                    newState = PlateDeployState.Suck;
            break;
            case Suck: 
                vacRelease(true);
                suck(true);
                push(false);
                if(deployUpdate(Constants.kPlateCenterDeployPauses[1])) 
                    newState = PlateDeployState.Push;
            break;
            case Push: 
                vacRelease(true);
                suck(true);
                push(true);
                if(deployUpdate(Constants.kPlateCenterDeployPauses[2])) 
                    newState = PlateDeployState.vacOff;
            break;
            case vacOff: 
                vacRelease(false);
                suck(false);
                push(true);
                if(deployUpdate(Constants.kPlateCenterDeployPauses[3])) 
                    newState = PlateDeployState.Retract;
            break;
            case Retract:
                vacRelease(false);
                suck(false);
                push(false);
                if(deployUpdate(Constants.kPlateCenterDeployPauses[4])) 
                    newState = PlateDeployState.FinalReset;
            break;
            case FinalReset:
                vacRelease(false);
                suck(false);
                push(false);
                hardStop(false);
                setPosition(Constants.kPlateCenterTalonSoftLimit/2);
                if(deployUpdate(Constants.kPlateCenterDeployPauses[5])) 
                    newState = PlateDeployState.Done;
            break;
        }


        if(newState!=deployState){
            deployState=newState;
            lastStateStart=now;
            System.out.println("Plate Deploy State: "+deployState);
        }

        if(deployState==PlateDeployState.Done){
            return defaultIdleTest();
        }
        
        return SystemState.DEPLOYINGPLATE;
       
    }

    private boolean deployUpdate(double time){
        if(time<=elapsedStateTime) return true;
        else return false;
    }

    private SystemState handleAutoAligning(double now, double startStartedAt){
        boolean ready=false;

        if(plateCentered){
        
            if(mStateChanged){
               // CameraVision.setCameraMode(CameraMode.eVision);
                CameraVision.setPipeline(0);
               CameraVision.setLedMode(LightMode.eOn);

               
            }


            //d = (h2-h1) / tan(a1+a2)
            double distance = (Constants.kLimeCameraHeight-Constants.kLimeTargetHeight)/
                Math.tan(Math.toRadians(Constants.kLimeCameraAngleFromHorizontal+CameraVision.getTy()));
           
            double rumble = 1-(distance/Constants.kLimeTriggerDistance);
            mControlBoard.setRumble(rumble);
           
            
            //Find the offset of target from camera with 0 at middle
            double target = Constants.kLimeLightDistancetoTarget*Math.tan(Math.toRadians(CameraVision.getTx()));

            //Find set point for motor with 0 on the left 
            target = Constants.kPlateCenterTalonSoftLimit/2 - 
                    Constants.kLimeLightDistanceFromCenter - target +inchesToCenter;

            

            
                    ready = atPosition();
            

            if(!CameraVision.isTarget()||now-startStartedAt<=Constants.kLimeLightTargetPause)ready=false;
            else setPosition(target);



            System.out.println("Distantce: "+distance+" Target set point: "+target+ " ready: "+ready+" inches to center: "+inchesToCenter+" position: "+getPosition());


            SystemState newState=mWantedState;

            if(ready&&Constants.kLimeLightAutoDeploy) newState = SystemState.DEPLOYINGPLATE;
           


            if(newState != SystemState.AUTOALIGNING){
                CameraVision.setLedMode(LightMode.eOff);
               CameraVision.setPipeline(1);
                mControlBoard.rumbleOff();
               // CameraVision.setCameraMode(CameraMode.eDriver);
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
        
        private boolean jog=false;
        public synchronized void jog(double amount){
            setPosition(mWantedSetPosition+=amount);
            jog=true;
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
                if(!jog)System.out.println("Wrist to "+mTravelingSetPosition);
                jog=false;
                mBeltTalon.set(ControlMode.Position, mTravelingSetPosition*Constants.kPlateCenterTicksPerInch);
            }
        }

    //Sensors
        public boolean getLidar(){
            return mLidar.get();
        }

    //Pneumatic Controls
        boolean pistonPrints = false;

        private void suck(boolean s){
           mSuckSolenoid.set(!s);
          if(pistonPrints) System.out.println("Suck solenoid: "+s);
        }
        private void push(boolean p){
            if(p) mDeploySolenoid.set(DoubleSolenoid.Value.kForward);
            else mDeploySolenoid.set(DoubleSolenoid.Value.kReverse);
            if(pistonPrints)System.out.println("Push solenoid: "+p);
        }
        public void hardStop(boolean h){
          mHardStopYeeYeeSolenoid.set(h);
          if(pistonPrints)System.out.println("HardStop solenoid: "+h);
        }
        public void vacRelease(boolean h){
         if(h) mVaccuumReleaseSolenoid.set(DoubleSolenoid.Value.kForward);
           else mVaccuumReleaseSolenoid.set(DoubleSolenoid.Value.kOff);
           if(pistonPrints)System.out.println("Vac release solenoid: "+h);
          }


        private void resetPistons(){
            hardStop(false);
            suck(false);
            push(false);
            vacRelease(false);
        }

    //Boring Stuff

        private void stopMotor(){
            mBeltTalon.set(ControlMode.PercentOutput,0);
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
            resetPistons();
        }

        @Override
        public void zeroSensors() {

        }

        @Override
        public void registerEnabledLoops(Looper in) {
            in.register(mLoop);
        }

}
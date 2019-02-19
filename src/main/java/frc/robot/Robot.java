package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.auto.AutoModeExecuter;
import frc.robot.loops.Looper;
import frc.robot.state_machines.BallControlHelper;
import frc.robot.state_machines.BallControlHelper.SystemState;
//import frc.robot.state_machines.Superstructure;
import frc.robot.subsystems.*;
import frc.lib.util.*;
import frc.lib.util.math.RigidTransform2d;

import java.util.Arrays;
import java.util.Map;
//kaden was here

/**
 * The main robot class, which instantiates all robot parts and helper classes and initializes all loops. Some classes
 * are already instantiated upon robot startup; for those classes, the robot gets the instance as opposed to creating a
 * new object
 * 
 * After initializing all robot parts, the code sets up the autonomous and teleoperated cycles and also code that runs
 * periodically inside both routines.
 * 
 * This is the nexus/converging point of the robot code and the best place to start exploring.
 * 
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the IterativeRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the manifest file in the resource directory.
 */
public class Robot extends TimedRobot {

    // Get subsystem instances
    private Drive mDrive = Drive.getInstance();
    private PlateCenter mPlate = PlateCenter.getInstance();
    private BallControlHelper mBall = BallControlHelper.getInstance();
    //private Superstructure mSuperstructure = Superstructure.getInstance();
    
    private AutoModeExecuter mAutoModeExecuter = null;

    // Create subsystem manager
   /* private final SubsystemManager mSubsystemManager = new SubsystemManager( 
            Arrays.asList(mDrive, Intake.getInstance(),Wrist.getInstance(),
           mPlate,Wrist.getInstance(),mBall));*/

    private final SubsystemManager mSubsystemManager = new SubsystemManager(Arrays.asList(mDrive,mPlate,mBall,
    Intake.getInstance(),Lift.getInstance(),Wrist.getInstance()));

    // Initialize other helper objects
    private ControlBoardInterface mControlBoard = ControlBoard.getInstance();

    private Looper mEnabledLooper = new Looper();

   private CameraVision cameraVision=new CameraVision();

  
    public Robot() {
        CrashTracker.logRobotConstruction();
    }

    public void zeroAllSensors() {
        mSubsystemManager.zeroSensors();
       mDrive.zeroSensors();
    }

    /**
     * This function is run when the robot is first started up and should be used for any initialization code.
     */
    @Override
    public void robotInit() {
        try {
            CrashTracker.logRobotInit();

            mSubsystemManager.registerEnabledLoops(mEnabledLooper);

            //Here it is:
          //  AutoModeSelector.initAutoModeSelector();

           

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
        zeroAllSensors();
    }

    /**
     * Initializes the robot for the beginning of autonomous mode (set drivebase, intake and superstructure to correct
     * states). Then gets the correct auto mode from the AutoModeSelector
     * 
     * @see AutoModeSelector.java
     */
    @Override
    public void autonomousInit() {
       /* try {
            CrashTracker.logAutoInit();

            System.out.println("Auto start timestamp: " + Timer.getFPGATimestamp());

            if (mAutoModeExecuter != null) {
                mAutoModeExecuter.stop();
            }

            zeroAllSensors();
            mSuperstructure.setWantedState(Superstructure.WantedState.IDLE);
           
            mAutoModeExecuter = null;

           
            
            mEnabledLooper.start();
            //mSuperstructure.reloadConstants();
            mAutoModeExecuter = new AutoModeExecuter();
            mAutoModeExecuter.setAutoMode(AutoModeSelector.getSelectedAutoMode());
            mAutoModeExecuter.start();

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }*/
    }

    /**
     * This function is called periodically during autonomous
     */
    @Override
    public void autonomousPeriodic() {
       // allPeriodic();
    }

    /**
     * Initializes the robot for the beginning of teleop
     */
    @Override
    public void teleopInit() {
        try {
            CrashTracker.logTeleopInit();

            // Start loopers
            mEnabledLooper.start();
            mDrive.setOpenLoop(DriveSignal.NEUTRAL);
            mBall.setWantedState(BallControlHelper.SystemState.HOME);
            mPlate.setWantedState(PlateCenter.SystemState.HOMING);
        

           zeroAllSensors();
           
            
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    /**
     * This function is called periodically during operator control.
     * 
     * The code uses state machines to ensure that no matter what buttons the driver presses, the robot behaves in a
     * safe and consistent manner.
     * 
     * Based on driver input, the code sets a desired state for each subsystem. Each subsystem will constantly compare
     * its desired and actual states and act to bring the two closer.
     */
    @Override
    public void teleopPeriodic() {
        try {
            //double timestamp = Timer.getFPGATimestamp();
           

        mDrive.setOpenLoop(mControlBoard.getDriveSignal());
          
       if(mControlBoard.getHatchPanelAlignment()) mPlate.setWantedState(PlateCenter.SystemState.AUTOALIGNING);
        else if(mControlBoard.getHatchPanelCentering()) mPlate.setWantedState(PlateCenter.SystemState.CENTERING);
        else if(mControlBoard.getHatchPanelDeploy()) mPlate.setWantedState(PlateCenter.SystemState.DEPLOYINGPLATE);
        if(mControlBoard.getPlateHome()) mPlate.setWantedState(PlateCenter.SystemState.HOMING);
       
            
       mPlate.jog(mControlBoard.getHatchPanelJog());

        if(mControlBoard.getBallPickUp()) mBall.pickUp(BallControlHelper.PickUpHeight.FLOOR);
        else if(mControlBoard.getBallShootPosition())mBall.shootPosition(BallControlHelper.ShootHeight.CARGO_SHIP);
        else if(mControlBoard.getBallShoot()) mBall.setWantedState(BallControlHelper.SystemState.SHOOT);
         if(mControlBoard.getBallHome()) mBall.setWantedState(BallControlHelper.SystemState.HOME);
        
        mBall.jogLift(mControlBoard.getLiftJog());    
     
        mBall.jogWrist(mControlBoard.getWristJog());         
            
        
        mDrive.lowGear(mControlBoard.getLowGear());
        
       // System.out.println(cameraVision.getX());
          //System.out.println("Left: "+mPlate.getLeftLidar()+" Right: "+mPlate.getRightLidar());

           allPeriodic();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledInit() {
        try {
            CrashTracker.logDisabledInit();

            if (mAutoModeExecuter != null) {
                mAutoModeExecuter.stop();
            }
            mAutoModeExecuter = null;

            mEnabledLooper.stop();

            // Call stop on all our Subsystems.
            mSubsystemManager.stop();

           mDrive.setOpenLoop(DriveSignal.NEUTRAL);
           
           

            // If are tuning, dump map so far.
          /*  if (Constants.kIsShooterTuning) {
                for (Map.Entry<InterpolatingDouble, InterpolatingDouble> entry : mTuningFlywheelMap.entrySet()) {
                    System.out.println("{" +
                            entry.getKey().value + ", " + entry.getValue().value + "},");
                }
            }*/
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledPeriodic() {
      /*  final double kVoltageThreshold = 0.15;
        if (mCheckLightButton.getAverageVoltage() < kVoltageThreshold) {
            mLED.setLEDOn();
        } else {
            mLED.setLEDOff();
        }*/

        zeroAllSensors();
        allPeriodic();
    }

    @Override
    public void testInit() {
       
    }

    @Override
    public void testPeriodic() {
    }

    /**
     * Helper function that is called in all periodic functions
     */
    public void allPeriodic() {
        
        mSubsystemManager.outputToSmartDashboard();
        mSubsystemManager.writeToLog();
        mEnabledLooper.outputToSmartDashboard();
       
        
    }
}

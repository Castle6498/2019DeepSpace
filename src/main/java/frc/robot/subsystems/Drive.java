package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;


import frc.robot.Constants;
import frc.robot.loops.Loop;
import frc.robot.loops.Looper;
import frc.lib.util.DriveSignal;
import frc.lib.util.drivers.Talon.CANTalonFactory;

/**
 * 
 * 
 *
 */
public class Drive extends Subsystem {

    private static Drive mInstance = new Drive();

    
    //private static final int kHighGearVelocityControlSlot = 1;

    public static Drive getInstance() {
        return mInstance;
    }

    // The robot drivetrain's various states.
    public enum DriveControlState {
        OPEN_LOOP, // open loop voltage control
        POSITION //position control
    }


    // Control states
    private DriveControlState mDriveControlState;

    // Hardware
   // private final CANTalon mLeftMaster, mRightMaster, mLeftSlave, mRightSlave;
   // private final Solenoid mShifter;
    
    private TalonSRX mLeftMaster, mRightMaster;
    private VictorSPX mLeftSlave, mRightSlave;
   
    
    //private final NavX mNavXBoard;

    private Drive() {
        
         //Talon Initialization 
            mLeftMaster = CANTalonFactory.createTalon(Constants.kDriveLeftTalonID, 
            false, NeutralMode.Brake, FeedbackDevice.QuadEncoder, 0, false);
    
            mLeftMaster = CANTalonFactory.setupHardLimits(mLeftMaster, LimitSwitchSource.Deactivated,
            LimitSwitchNormal.Disabled, false, LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled,false);
            
            mLeftMaster = CANTalonFactory.setupSoftLimits(mLeftMaster, false, 0, false, 0);
            
            mLeftMaster = CANTalonFactory.tuneLoops(mLeftMaster, 0, Constants.kDriveLeftTalonP,
            Constants.kDriveLeftTalonI, Constants.kDriveLeftTalonD, Constants.kDriveLeftTalonF);

            mRightMaster = CANTalonFactory.createTalon(Constants.kDriveRightTalonID, 
            false, NeutralMode.Brake, FeedbackDevice.QuadEncoder, 0, false);
    
            mRightMaster = CANTalonFactory.setupHardLimits(mRightMaster, LimitSwitchSource.Deactivated,
            LimitSwitchNormal.Disabled, false, LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled,false);
            
            mRightMaster = CANTalonFactory.setupSoftLimits(mRightMaster, false, 0, false, 0);
            
            mRightMaster = CANTalonFactory.tuneLoops(mRightMaster, 0, Constants.kDriveRightTalonP,
            Constants.kDriveRightTalonI, Constants.kDriveRightTalonD, Constants.kDriveRightTalonF);
        
        
        
        //Victor Initialization
            mLeftSlave = new VictorSPX(Constants.kDriveLeftVictorID);
            mLeftSlave.setInverted(false);
            mLeftSlave.follow(mLeftMaster);
            
            mRightSlave = new VictorSPX(Constants.kDriveRightVictorID);
            mLeftSlave.setInverted(false);
            mRightSlave.follow(mRightMaster);
        
       
        setOpenLoop(DriveSignal.NEUTRAL);

        
        //mNavXBoard = new NavX(SPI.Port.kMXP);

      
    }

    private final Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            synchronized (Drive.this) {
                setOpenLoop(DriveSignal.NEUTRAL);                
                //mNavXBoard.reset();
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Drive.this) {
                switch (mDriveControlState) {
                case OPEN_LOOP:
                    return;      
                case POSITION:
                    return;                          
                default:
                    System.out.println("Unexpected drive control state: " + mDriveControlState);
                    break;
                }
            }
        }

        @Override
        public void onStop(double timestamp) {
            stop();
            
        }
    };

    //allows the initialization of the drive loop
    @Override
    public void registerEnabledLoops(Looper in) {
        in.register(mLoop);
    }

    /**
     * Update open loop control
     */
    public synchronized void setOpenLoop(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.OPEN_LOOP) {
            mDriveControlState = DriveControlState.OPEN_LOOP;  
            System.out.println("Drive in open loop"); 
        }
        setBrakeMode(signal.getBrakeMode());
        // Right side is reversed, but reverseOutput doesn't invert PercentVBus.
        // So set negative on the right master.
        mRightMaster.set(ControlMode.PercentOutput,signal.getRight());
        mLeftMaster.set(ControlMode.PercentOutput, signal.getLeft());
    }

    /**
     * Adjust position setpoint (if already in position mode)
     * 
     * @param left_position_inches
     * @param right_position_inches
     */
    public synchronized void setPositionSetpoint(double left_position_inches, double right_position_inches) {
        if(mDriveControlState != DriveControlState.POSITION){
            mDriveControlState = DriveControlState.POSITION;
            System.out.println("Drive in position control");
            setBrakeMode(true);
        }

        mLeftMaster.set(ControlMode.Position,inchesToTicks(left_position_inches));
        mRightMaster.set(ControlMode.Position,inchesToTicks(right_position_inches));
   }

   public boolean atSetpoint(){
       if(Math.abs(mLeftMaster.getSelectedSensorPosition()-mLeftMaster.getClosedLoopTarget())>Constants.kDriveTolerance){
           return true;
       }else return false;
   }

    private boolean mIsBrakeMode=false;
  
    public boolean isBrakeMode() {
        return mIsBrakeMode;
    }

    public synchronized void setBrakeMode(boolean on) {
        if (mIsBrakeMode != on) {
            mIsBrakeMode = on;
            if(on){
            mRightMaster.setNeutralMode(NeutralMode.Brake);
            mRightSlave.setNeutralMode(NeutralMode.Brake);
            mLeftMaster.setNeutralMode(NeutralMode.Brake);
            mLeftSlave.setNeutralMode(NeutralMode.Brake);
            }else{
            mRightMaster.setNeutralMode(NeutralMode.Coast);
            mRightSlave.setNeutralMode(NeutralMode.Coast);
            mLeftMaster.setNeutralMode(NeutralMode.Coast);
            mLeftSlave.setNeutralMode(NeutralMode.Coast);
            }
        }
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    @Override
    public void outputToSmartDashboard() {

    }

    public synchronized void resetEncoders() {
        mLeftMaster.setSelectedSensorPosition(0);
        mRightMaster.setSelectedSensorPosition(0);
     }

    @Override
    public void zeroSensors() {
        resetEncoders();
       // mNavXBoard.zeroYaw();
    }


   

    private synchronized double inchesToTicks(double inches){
        return inches*Constants.kDriveTicksPerInch;
    }

    @Override
    public void writeToLog() {
        
    }

    public boolean checkSystem() {
        System.out.println("Testing DRIVE.---------------------------------");
       
        boolean failure = false;
        
        return !failure;
    }
}

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Contains the button mappings for the competition control board. Like the
 * drive code, one instance of the ControlBoard object is created upon startup,
 * then other methods request the singleton ControlBoard instance. Implements
 * the ControlBoardInterface.
 * 
 * @see ControlBoardInterface.java
 */
public class ControlBoard implements ControlBoardInterface {
    private static ControlBoardInterface mInstance = null;

    private static final boolean kUseGamepad = false;

    public static ControlBoardInterface getInstance() {
        if (mInstance == null) {
            //if (kUseGamepad) {
               // mInstance = new GamepadControlBoard();
            //} else {
                mInstance = new ControlBoard();
            //}
        }
        return mInstance;
    }

    private final XboxController mOperator;
    

    protected ControlBoard() {
        mOperator = new XboxController(0);
        
    }

    @Override
    public double getThrottle() {
        return 0;
    }

    @Override
    public double getTurn() {
        return 0;
    }

    @Override
    public boolean getLowGear() {
        return false;
    }

    @Override
    public boolean getAimButton() {
        return false;
    }

    @Override
    public boolean getDriveAimButton() {
        return false;
    }

    @Override
    public boolean getHatchPanelCentering() {
        return mOperator.getXButton();
    }

    @Override
    public boolean getHatchPanelAlignment() {
        return mOperator.getYButton();
    }

    @Override
    public double getHatchPanelJog() {
        double speed=0;
        double left = mOperator.getTriggerAxis(Hand.kLeft);
        double right = mOperator.getTriggerAxis(Hand.kRight);
		if(right>.1) {
			speed=right;			
		}else if(left>.1) {
			speed=-left;
		}else {
			speed=0;
		}
		speed*=.05;//TODO tune this value maybe add in constants
        return speed;
    }

    @Override
    public boolean getHatchPanelDeploy() {
        return mOperator.getAButton();
    }

    @Override
    public boolean getPlateHome() {
        return mOperator.getBButton();
    }

    
}

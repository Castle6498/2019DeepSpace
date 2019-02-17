package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.lib.util.DriveSignal;
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
    private final XboxController mDriver;
    private final XboxController mOperator;
    

    protected ControlBoard() {
        mOperator = new XboxController(1);
        mDriver =new XboxController(0);
    }

    @Override
    public double getThrottle() {
        driverArcadeDrive();
        return throttle;
    }
    

    @Override
    public double getTurn() {
        driverArcadeDrive();
        return turn;
    }

    boolean driveReduction=false;
	double driveReductionAmount = .7; //remember a higher number means less reduction
	
    double turnReduction=.85;
    
    double throttle=0;
    double turn=0;
	
	public void driverArcadeDrive() {
        throttle=0;
		 turn=mDriver.getX(Hand.kLeft)*turnReduction;
		if(mDriver.getTriggerAxis(Hand.kRight)>.05) {
			throttle=mDriver.getTriggerAxis(Hand.kRight);			
		}else if(mDriver.getTriggerAxis(Hand.kLeft)>.05) {
			throttle=-mDriver.getTriggerAxis(Hand.kLeft);
			turn=-turn;
		}else {
			throttle=0;
		}
		if(driveReduction) {
			turn=turn*driveReductionAmount;
			throttle=throttle*driveReductionAmount;
        }
		
    }

    @Override
    public DriveSignal getDriveSignal() {
        boolean squareInputs=true;
        double xSpeed;
        double zRotation;
        
        driverArcadeDrive();

        xSpeed=throttle;
      
        zRotation = turn;
          
      
          // Square the inputs (while preserving the sign) to increase fine control
          // while permitting full power.
          if (squareInputs) {
            xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
            zRotation = Math.copySign(zRotation * zRotation, zRotation);
          }
      
          double leftMotorOutput;
          double rightMotorOutput;
      
          double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);
      
          if (xSpeed >= 0.0) {
            // First quadrant, else second quadrant
            if (zRotation >= 0.0) {
              leftMotorOutput = maxInput;
              rightMotorOutput = xSpeed - zRotation;
            } else {
              leftMotorOutput = xSpeed + zRotation;
              rightMotorOutput = maxInput;
            }
          } else {
            // Third quadrant, else fourth quadrant
            if (zRotation >= 0.0) {
              leftMotorOutput = xSpeed + zRotation;
              rightMotorOutput = maxInput;
            } else {
              leftMotorOutput = maxInput;
              rightMotorOutput = xSpeed - zRotation;
            }
          }
          double m_rightSideInvertMultiplier = -1.0;

          leftMotorOutput=(limit(leftMotorOutput) * 1);
          rightMotorOutput=(limit(rightMotorOutput) * 1 * m_rightSideInvertMultiplier);
     // System.out.println("Rot:"+turn+" xSpeed: "+xSpeed+" Left: "+leftMotorOutput+ " right: "+rightMotorOutput);
          return new DriveSignal(leftMotorOutput,rightMotorOutput,false);
        
    }

    protected double limit(double value) {
        if (value > 1.0) {
          return 1.0;
        }
        if (value < -1.0) {
          return -1.0;
        }
        return value;
      }

    

    @Override
    public boolean getLowGear() {
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
		speed*=.1;
        return speed;
    }

    @Override
    public boolean getHatchPanelDeploy() {
        return mDriver.getXButton();
    }

    @Override
    public boolean getPlateHome() {
        return mOperator.getRawButton(8);
    }

    @Override
    public boolean getBallPickUp() {
        return mOperator.getAButton();
    }

    @Override
    public boolean getBallShootPosition() {
        return mOperator.getPOV()==270;
    }

    @Override
    public boolean getBallShoot() {
        return mDriver.getAButton();
    }

    @Override
    public boolean getCarryBall() {
        return false;
    }

    @Override
    public boolean getBallHome() {
        return mOperator.getRawButton(7);
    }

    @Override
    public double getLiftJog() {
        double speed=mOperator.getY(Hand.kLeft);
        if(Math.abs(speed)<=.1){
            speed=0;
        }

		speed*=.05;
        return speed;
    }

    @Override
    public double getWristJog() {
        double speed=mOperator.getY(Hand.kRight);
        if(Math.abs(speed)<=.1){
            speed=0;
        }

		speed*=.05;
        return speed;
    }

  

    
}

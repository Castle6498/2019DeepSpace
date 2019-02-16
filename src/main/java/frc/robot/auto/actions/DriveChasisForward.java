package frc.robot.auto.actions;

import frc.robot.subsystems.Drive;
import frc.lib.util.math.Rotation2d;

/**
 * Turns the robot to a specified heading
 * 
 * @see Action
 */
public class DriveChasisForward implements Action {

    private double position;
    private Drive mDrive = Drive.getInstance();

    public DriveChasisForward(double targetPosition) {
        position = targetPosition;
    }

    @Override
    public boolean isFinished() {
        return mDrive.atSetpoint();
    }

    @Override
    public void update() {
        // Nothing done here, controller updates in mEnabedLooper in robot
    }

    @Override
    public void done() {
        
    }

    @Override
    public void start() {
        mDrive.setPositionSetpoint(position, position);
    }
}

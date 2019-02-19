
package frc.robot;

import frc.lib.util.DriveSignal;

/**
 * A basic framework for robot controls that other controller classes implement
 */
public interface ControlBoardInterface {
    // DRIVER CONTROLS
    double getThrottle();

   DriveSignal getDriveSignal();

    double getTurn();

    boolean getLowGear();

    // OPERATOR CONTROLS
    boolean getHatchPanelCentering();

    boolean getHatchPanelAlignment();

    boolean getPlateHome();

    double getHatchPanelJog();

    boolean getHatchPanelDeploy();

    boolean getBallPickUp();

    boolean getBallShootPosition();

    boolean getBallShoot();

    boolean getCarryBall();

    boolean getBallHome();

    double getLiftJog();

    double getWristJog();

}

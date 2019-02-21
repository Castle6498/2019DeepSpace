
package frc.robot;

import frc.lib.util.DriveSignal;
import frc.robot.state_machines.BallControlHelper.CarryHeight;
import frc.robot.state_machines.BallControlHelper.PickUpHeight;
import frc.robot.state_machines.BallControlHelper.ShootHeight;

/**
 * A basic framework for robot controls that other controller classes implement
 */
public interface ControlBoardInterface {
    // DRIVER CONTROLS
    double getThrottle();

   DriveSignal getDriveSignal();

    double getTurn();

    boolean getLowGear();

    boolean getDriveInverted();

    // OPERATOR CONTROLS
    boolean getHatchPanelCentering();

    boolean getHatchPanelAlignment();

    boolean getPlateHome();

    double getHatchPanelJog();

    boolean getHatchPanelDeploy();

    boolean getHatchHardStops();

    boolean getHatchReset();

    PickUpHeight getBallPickUp();

    ShootHeight getBallShootPosition();

    boolean getBallShoot();

    CarryHeight getCarryBall();

    boolean getBallHome();

    double getLiftJog();

    double getWristJog();

    double getSuspensionJog();

    double getSuspensionWheelJog();

    boolean getSuspensionHome();

}

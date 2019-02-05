package frc.robot;

/**
 * A basic framework for robot controls that other controller classes implement
 */
public interface ControlBoardInterface {
    // DRIVER CONTROLS
    double getThrottle();

    double getTurn();

    //boolean getQuickTurn();

    boolean getLowGear();

    boolean getAimButton();

    boolean getDriveAimButton();

    // OPERATOR CONTROLS
    boolean getHatchPanelCentering();

    boolean getHatchPanelAlignment();

    boolean getPlateHome();

    double getHatchPanelJog();

    boolean getHatchPanelDeploy();
}

package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;

import frc.robot.auto.actions.WaitAction;


/**
 * Scores the preload gear onto the boiler-side peg then shoots the 10 preloaded fuel
 * 
 * @see AutoModeBase
 */
public class BoilerGearThenShootModeBlue extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new WaitAction(2));
      //  PathContainer gearPath = new StartToBoilerGearBlue();
     //   runAction(new ResetPoseFromPathAction(gearPath));
    //    runAction(new DrivePathAction(gearPath));       
     //   runAction(new DrivePathAction(new BoilerGearToShootBlue()));             
    }
}

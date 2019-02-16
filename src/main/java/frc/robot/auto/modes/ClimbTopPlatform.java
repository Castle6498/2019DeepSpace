package frc.robot.auto.modes;



import frc.robot.Constants;
import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.Action;
//import frc.robot.auto.actions.ClimbUp;
import frc.robot.auto.actions.DriveChasisForward;
//import frc.robot.auto.actions.DriveSuspensionForward;
import frc.robot.auto.actions.ParallelAction;
import frc.robot.auto.actions.WaitAction;

import java.util.ArrayList;
import java.util.List;

/**
 * Scores the preload gear onto the boiler-side peg then shoots the 10 preloaded fuel
 * 
 * @see AutoModeBase
 */
/*
public class ClimbTopPlatform extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        //runAction(new WaitAction(2));
        System.out.println("CLIMBING TOP BOI");
        runAction(new ClimbUp(Constants.firstLiftHeight, Constants.firstLiftHeight));

        List<Action> actions=new ArrayList<Action>();
        actions.add(new DriveSuspensionForward(Constants.firstLiftDriveForward));
        actions.add(new DriveChasisForward(Constants.firstLiftDriveForward));
        runAction(new ParallelAction(actions));

        runAction(new ClimbUp(Constants.firstLiftHeight,Constants.frontRaiseHeight));

        actions.clear();
        actions.add(new DriveSuspensionForward(Constants.secondDriveForward));
        actions.add(new DriveChasisForward(Constants.secondDriveForward));
        runAction(new ParallelAction(actions));

        runAction(new ClimbUp(Constants.backRaiseHeight,Constants.frontRaiseHeight));

        actions.clear();
        actions.add(new DriveSuspensionForward(Constants.thirdDriveForward));
        actions.add(new DriveChasisForward(Constants.thirdDriveForward));
        runAction(new ParallelAction(actions));
    }
}*/

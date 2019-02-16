package frc.robot;

import frc.lib.util.ConstantsBase;

/**
 * A list of constants used by the rest of the robot code. This include physics
 * constants as well as constants determined through calibrations.
 */
public class Constants extends ConstantsBase {
    public static double kLooperDt = 0.005;

    
  

         
    
   

       
        
       

       

    
    //Autonomous
        //Climbing Top Platform TODO: Test climb on platform - how to run AUTO in teleop???
        public static final double firstLiftHeight=6;
        public static final double firstLiftDriveForward=2;
        public static final double frontRaiseHeight=0;
        public static final double secondDriveForward=2;
        public static final double backRaiseHeight=0;
        public static final double thirdDriveForward=2;
    // Drive
         //Talon
        public static final int kDriveLeftTalonID=9;
        public static final double kDriveLeftTalonP=20;
        public static final double kDriveLeftTalonI=0;
        public static final double kDriveLeftTalonD=0;
        public static final double kDriveLeftTalonF=0;

        public static final int kDriveRightTalonID=3;
        public static final double kDriveRightTalonP=20;
        public static final double kDriveRightTalonI=0;
        public static final double kDriveRightTalonD=0;
        public static final double kDriveRightTalonF=0;

        public static final double kDriveTicksPerInch=1680 * 1; //TODO: get wheel diam
        public static final double kDriveTolerance=5;

        //Victor
        public static final int kDriveLeftVictorID=2;
        public static final int kDriveRightVictorID=1;
    //Ball Control Helper TODO: Get all of these crazy things
        //PickUp
            public static final double kLiftPickUpFloor = 10;
            public static final double kWristPickUpFloor = 90;
            public static final double kLiftPickUpLoadingStation = 5;
            public static final double kWristPickUpLoadingStation = 0;

        //Shoot Height
            public static final double kLiftShootCargoShip = 20;
            public static final double kWristShootCargoShip = 45;

            public static final double kLiftShootRocketOne = 0;
            public static final double kWristShootRocketOne = 0;

            public static final double kLiftShootRocketTwo = 0;
            public static final double kWristShootRocketTwo = 0;

            public static final double kLiftShootRocketThree = 0;
            public static final double kWristShootRocketThree = 0;
        
        //Carry Height
            public static final double kLiftCarryLow = 0;
            public static final double kWristCarryLow = 0;

            public static final double kLiftCarryMiddle = 0;
            public static final double kWristCarryMiddle = 0;

        //Shoot
            public static final double kCarryPauseAfterShoot=2;




    //Intake -----------------------------------------------------------
    
        //Talon
        public static final int kIntakeTalonChannel=0;

        public static final double kIntakePickUpSpeed=.5;
        public static final double kIntakeShootSpeed=-.5;

        public static final double kIntakePickUpPause = 2; //TODO - tune intake pause
        public static final double kIntakeShootPause = 2; //TODO - tune shoot pause

        //Photoeye
        public static final int kIntakeSensorPort = 0;
         
    //Wrist -----------------------------------------------------------
    
        //Talon
        public static final int kWristTalonID=0;
        public static final int kWristChildTalonID=12; //TODO:IDK WHAT THIS IS
        public static final double kWristTalonP=1;
        public static final double kWristTalonI=0;
        public static final double kWristTalonD=0;
        public static final double kWristTalonF=0;

        public static final double kWristTicksPerDeg=1024/360; //TODO: get wheel diam
        public static final double kWristSoftLimit=110; //TODO: GET this, IMPORTANT
        public static final double kWristTolerance = 5;

    //Lift -----------------------------------------------------------
    
        //Talon
        public static final int kLiftTalonID=0;
        public static final double kLiftTalonP=1;
        public static final double kLiftTalonI=0;
        public static final double kLiftTalonD=0;
        public static final double kLiftTalonF=0;

        public static final double kLiftTicksPerInch=4096 /(1.4*Math.PI); //TODO: get wheel diam
        public static final double kLiftSoftLimit=28; //TODO: GET this, IMPORTANT

        public static final double kLiftTolerance = 5;

    //Suspension -----------------------------------------------------------
        
        //Talon
        public static final int kSuspensionBackLiftTalonID=0;
            public static final double kSuspensionBackLiftTalonP=20;
            public static final double kSuspensionBackLiftTalonI=0;
            public static final double kSuspensionBackLiftTalonD=0;
            public static final double kSuspensionBackLiftTalonF=0;

        public static final int kSuspensionFrontLiftTalonID=0;
            public static final double kSuspensionFrontLiftTalonP=20;
            public static final double kSuspensionFrontLiftTalonI=0;
            public static final double kSuspensionFrontLiftTalonD=0;
            public static final double kSuspensionFrontLiftTalonF=0;

        public static final double kSuspensionLiftTicksPerInch=1680 * 1; //TODO: get wheel diam
        public static final int kSuspensionLiftSoftLimit=(int) Math.round(5*kSuspensionLiftTicksPerInch); //TODO: GET this, IMPORTANT
        public static final double kSuspensionLiftTolerance =5;

        public static final int kSuspensionWheelTalonID=0;
            public static final double kSuspensionWheelTalonP=20;
            public static final double kSuspensionWheelTalonI=0;
            public static final double kSuspensionWheelTalonD=0;
            public static final double kSuspensionWheelTalonF=0;

            public static final double kSuspensionWheelTicksPerInch=1680 * 1; //TODO: get wheel diam
            public static final double kSuspensionWheelTolerance = 5;
    
    //PlateCenter -----------------------------------------------------------
    
        //Talon
        public static final int kPlateCenterTalonID=0;
            public static final double kPlateCenterTalonP=20;
            public static final double kPlateCenterTalonI=0;
            public static final double kPlateCenterTalonD=0;
            public static final double kPlateCenterTalonF=0;

            public static final double kPlateCenterTicksPerInch=1680 / (2.24*Math.PI); //TODO: get wheel diam
            public static final double kPlateCenterTalonSoftLimit=15.4375; //TODO: GET this, IMPORTANT
            public static final int kPlateCenterTalonTolerance = 5;

        //Pneumatics
        public static final int kPlateCenterSuckSolenoidPort=0;
        public static final int kPlateCenterDeploySolenoidPort=2;
        public static final int kPlateCenterHardStopYeeYeeSolenoidPort=1;
                                                            //suck, push, pause, release
        public static final int[] kPlateCenterDeployPauses = {2,4,6,8}; 
        //Lidar
        public static final int kPlateCenterLeftLidar= 1;
        public static final int kPlateCenterCenterLidar = 2;
        public static final int kPlateCenterRightLidar = 3;
            
        
    

    

    @Override
    public String getFileLocation() {
        return "~/constants.txt";
    }

   
}

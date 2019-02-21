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

        public static final int kDriverShifterPort=3;
    //Ball Control Helper TODO: Get all of these crazy things
        //PickUp
            public static final double kLiftPickUpFloor = 0;
            public static final double kWristPickUpFloor = -140;
            public static final double kLiftPickUpLoadingStation = 20;
            public static final double kWristPickUpLoadingStation = 0;
 
        //Shoot Height
            public static final double kLiftShootCargoShip = 24;
            public static final double kWristShootCargoShip = -25;

            public static final double kLiftShootRocketOne = 6;
            public static final double kWristShootRocketOne = 0;

            public static final double kLiftShootRocketTwo = 34;
            public static final double kWristShootRocketTwo = 0;
        
        //Carry Height
            public static final double kLiftCarryLow = 0;
            public static final double kWristCarryLow = 0;

            public static final double kLiftCarryMiddle = 0;
            public static final double kWristCarryMiddle = 0;

        //Shoot
            public static final double kCarryPauseAfterShoot=1;

        //Auto Features
        public static final boolean kCarryAfterPickUp=true;
        public static final boolean kCarryAfterShoot=true;




    //Intake -----------------------------------------------------------
    
        //Talon
        public static final int kIntakeTalonChannel=0;

        public static final double kIntakePickUpSpeed=.5;
        public static final double kIntakeShootSpeed=-1;

        public static final double kIntakeBallRequiredTime=.35;
        public static final double kIntakeShootPause = 1; 

        //Photoeye
        public static final int kIntakeSensorPort = 0;
         
    //Wrist -----------------------------------------------------------
    
        //Talon
        public static final int kWristTalonID=8;
        public static final int kWristChildTalonID=0; 

        public static final double kWristTalonP=10;
        public static final double kWristTalonI=0;
        public static final double kWristTalonD=0;
        public static final double kWristTalonF=0;

        public static final double kWristTicksPerDeg=1024/360; 
        public static final double kWristSoftLimit=160; 
        public static final double kWristTolerance = .5;

        public static final double kWristMaxOutput=.6;

    //Lift -----------------------------------------------------------
    
        //Talon
        public static final int kLiftTalonID=10;

        public static final double kLiftTalonP=.8;
        public static final double kLiftTalonI=0;
        public static final double kLiftTalonD=0;
        public static final double kLiftTalonF=0;

        public static final double kLiftTicksPerInch=4096 /(1.4*Math.PI); 
        public static final double kLiftSoftLimit=34; 

        public static final double kLiftTolerance = .5;

    //Suspension -----------------------------------------------------------
        
        //Talon
        public static final int kSuspensionBackLiftTalonID=4;
            public static final double kSuspensionBackLiftTalonP=1;
            public static final double kSuspensionBackLiftTalonI=0;
            public static final double kSuspensionBackLiftTalonD=0;
            public static final double kSuspensionBackLiftTalonF=0;

        public static final double kSuspensionLiftTicksPerInch=1024*4*16; 
        public static final int kSuspensionLiftSoftLimit=20; 
        public static final double kSuspensionLiftTolerance =.1;

        public static final int kSuspensionWheelTalonID=6;
            public static final double kSuspensionWheelTalonP=20;
            public static final double kSuspensionWheelTalonI=0;
            public static final double kSuspensionWheelTalonD=0;
            public static final double kSuspensionWheelTalonF=0;

            public static final double kSuspensionWheelTicksPerInch=1680/(2.24*Math.PI); //TODO: get wheel diam
            public static final double kSuspensionWheelTolerance = .2;
    
    //PlateCenter -----------------------------------------------------------
    
        //Talon
        public static final int kPlateCenterTalonID=7;
            public static final double kPlateCenterTalonP=20;
            public static final double kPlateCenterTalonI=0;
            public static final double kPlateCenterTalonD=0;
            public static final double kPlateCenterTalonF=0;

            public static final double kPlateCenterTicksPerInch=1680 / (2.24*Math.PI); 
            public static final double kPlateCenterTalonSoftLimit=15; 
            public static final double kPlateCenterTalonTolerance = .5;

        //Pneumatics
        public static final int kPlateCenterSuckSolenoidPort=2;
        public static final int[] kPlateCenterDeploySolenoidPort={0,4};
        public static final int kPlateCenterHardStopYeeYeeSolenoidPort=1;
                                                            //suck, push, pause, release
        public static final double[] kPlateCenterDeployPauses = {.25,.5,1,1.25}; 
        //Lidar                                             the pause is in the space between
        public static final int kPlateCenterLeftLidar=2;
        public static final int kPlateCenterRightLidar = 3;
            
        //0 extend push off 4 retract
        //1 hardstop
        //suck 2
        //3 shifter
        
    

    

    @Override
    public String getFileLocation() {
        return "~/constants.txt";
    }

   
}

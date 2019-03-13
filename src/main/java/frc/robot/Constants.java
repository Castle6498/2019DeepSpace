package frc.robot;

import frc.lib.util.ConstantsBase;

/**
 * A list of constants used by the rest of the robot code. This include physics
 * constants as well as constants determined through calibrations.
 */
public class Constants extends ConstantsBase {
    public static double kLooperDt = 0.005;

    
  

         
    
   

       
        
       

       

    
    //Autonomous
        public static final double climbReadyMiddleHeight=12;
        public static final double climbReadyMiddleWristAngle=220;

        public static final double climbReadyHighHeight=20.5;
        public static final double climbReadyHighWristAngle=220;

        public static final double climbMiddleHeight=climbReadyMiddleHeight;        

        public static final double climbHighHeight=climbReadyHighHeight;

        public static final boolean autoClimb=true;
        


       
    // Drive
         //Talon
         public static final double regularTurnReduction=.85;

        public static final double kDriveSwivelReduction=.7;


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
    //Ball Control Helper 
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
        public static final int kIntakeTalonID=5;

        public static final double kIntakePickUpSpeed=.8;
        public static final double kIntakeShootSpeed=-1;

        public static final double kIntakeBallRequiredTime=.7;
        public static final double kIntakeShootPause = 1; 

        public static final double kIntakeCurrentThreshold=1.5;

        //Photoeye
        public static final int kIntakeSensorPort = 0;
         
    //Wrist -----------------------------------------------------------
    
        //Talon
        public static final int kWristTalonID=8;
        public static final int kWristChildTalonID=0; 

        public static final double kWristTalonP=12;
        public static final double kWristTalonI=0;
        public static final double kWristTalonD=0;
        public static final double kWristTalonF=0;

        public static final double kWristTicksPerDeg=1024/360; 
        public static final double kWristSoftLimit=220; 
        public static final double kWristTolerance = .5;

        public static final double kWristMaxOutput=.6;
    //CLIMBING CONTROLS --------------------------------------------

        public static final double kWristClimbP=18;//10
        public static final double kWristClimbI=0;
        public static final double kWristClimbD=0;
        public static final double kWristClimbF=0;

        public static final double kLiftClimbP=3;//1
        public static final double kLiftClimbI=0;
        public static final double kLiftClimbD=0;

        public static final double kLiftMaxSpeedClimb=1800;//1800
        public static final double kLiftClimbF=(1.0*1023)/kLiftMaxSpeedClimb;

        public static final double kLiftClimbVelocity=.5;   
        public static final double kLiftClimbAcceleration=.5; 


        
      



    //Lift -----------------------------------------------------------
    
        //Talon
        public static final int kLiftTalonID=10;

        public static final double kLiftTalonP=1.4;
        public static final double kLiftTalonI=0;
        public static final double kLiftTalonD=0;
        public static final double kLiftMaxSpeed=1800;
        public static final double kLiftTalonF=(1.0*1023)/kLiftMaxSpeed;

        public static final double kLiftVelocity=32;   
        public static final double kLiftAcceleration=20; 

        public static final double kLiftTicksPerInch=4096 /(1.4*Math.PI); 
        public static final double kLiftSoftLimit=34; 

        public static final double kLiftTolerance = .5;

    //Suspension -----------------------------------------------------------
        
        //Talon
        public static final int kSuspensionBackLiftTalonID=4;
            public static final double kSuspensionBackLiftTalonP=5;//.1
            public static final double kSuspensionBackLiftTalonI=0;
            public static final double kSuspensionBackLiftTalonD=0;
            public static final double kSuspensionMaxSpeedClimb=90000;//96000;
        public static final double kSuspensionBackLiftF=(1.0*1023)/kSuspensionMaxSpeedClimb;

        public static final double kSuspensionVelocity=10;   
        public static final double kSuspensionAcceleration=2; 
        

            

        public static final double kSuspensionLiftTicksPerInch=(1024*4)*4*16; //added hopefully from test pannel testing
        public static final int kSuspensionLiftSoftLimit=21; 
        public static final double kSuspensionLiftTolerance =.5;

        public static final int kSuspensionWheelTalonID=6;
            public static final double kSuspensionWheelTalonP=30;
            public static final double kSuspensionWheelTalonI=0;
            public static final double kSuspensionWheelTalonD=0;
            public static final double kSuspensionWheelTalonF=0;


            public static final double kSuspensionWheelTicksPerInch=1680/(2.24*Math.PI); 
            public static final double kSuspensionWheelTolerance = .1;
    
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
        public static final int[] kPlateVaccuumReleaseSolenoidPort={6,7};
        
                            //VacOn, Suck, Push, vacOff, Retract, FinalReset, Done
        public static final double[] kPlateCenterDeployPauses = {.1,.2,.2,1,.2,.2}; 
        //Lidar                                            
        public static final int kPlateCenterLidar=1;

        public static final double kPlateCenterCenteringSpeed=.4;

        //Ultrasonic
        public static final int[] kPlateCenterUltraPort={2,3};


        //LimeLight

        public static final double kLimeLightDistancetoTarget=25;
                                  
        public static final double kLimeLightDistanceFromCenter=1; //positive is movement to the left

        public static final boolean kLimeLightAutoDeploy=false;

        public static final double kLimeLightTargetPause = .25;




        public static final double kLimeTargetHeight=30;
        public static final double kLimeCameraAngleFromHorizontal=45;
        public static final double kLimeCameraHeight=43;  

        public static final double kLimeTriggerDistance=30;







            
 
    //LED -------------------------------------------------------------------------------

        public static final int[] kPlateLEDPorts = {4,5,6};
    

    

    @Override
    public String getFileLocation() {
        return "~/constants.txt";
    }

   
}

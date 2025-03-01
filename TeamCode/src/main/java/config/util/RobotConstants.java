package config.util;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {
    //Active Intake Spinner
    public static double spinnerEject = -1;
    public static double spinnerLoad = 1;
    public static double spinnerStop = 0;


    // Specimen Claw
    public static double clawWristInit = 0.88;
    public static double clawWristReturn = 0.88;
    public static double clawWristDeposit = 0.80;
    public static double clawWristGrab = 0.59;
    public static double clawWristPlace = 0.55;


    // Active Intake Arm
    public static double lArmTransfer= 0.066; //.05
    public static double rArmTransfer= lArmTransfer - 0.045;
    public static double lArmDeposit = 0.595;
    public static double rArmDeposit = lArmDeposit - 0.045;
    public static double lArmMiddle = 0.655;
    public static double rArmMiddle = lArmMiddle - 0.045;
    public static double lArmInter = 0.625;
    public static double rArmInter = lArmInter - 0.045;
    public static double lArmInit = 0.195;
    public static double rArmInit = lArmInit - 0.045;
    public static double lArmSpecimenGrab = 0.855;
    public static double rArmSpecimenGrab = lArmSpecimenGrab - 0.045;
    public static double lArmSpecimenReturn = 0.195;
    public static double rArmSpecimenReturn = lArmSpecimenReturn - 0.045;
    public static double lArmSpecimenPlace = 0.805;
    public static double rArmSpecimenPlace = lArmSpecimenPlace - 0.045;
    public static double lArmSpecimenBackPlace = 0.855;
    public static double rArmSpecimenBackPlace = lArmSpecimenBackPlace - 0.045;


    public static int liftToZero = 0;
    public static int liftToHighBucket = 3200;

    //Claw Arm
    public static double armTransfer= 0.066; //.05
    public static double armDeposit = 0.595;
    public static double armMiddle = 0.655;
    public static double armInit = 0.195;
    public static double armSpecimenGrab = 0.77;
    public static double armFolded = 0.13;
    public static double armSpecimenPlace = 0.805;
    public static double armSpecimenBackPlace = 0.855;
    public static double armSpecimenPickup = 0.2;

    //Diffy Servos For Claw Arm
    public static double lDiffyTransfer= 0.066;
    public static double rDiffyTransfer= 0.066;
    public static double lDiffyDeposit = 0.595;
    public static double rDiffyDeposit = 0.595;
    public static double lDiffyMiddle = 0.655;
    public static double rDiffyMiddle = 0.655;
    public static double lDiffyInit = 0.195;
    public static double rDiffyInit = 0.195;
    public static double lDiffySpecimenGrab = 0.855;
    public static double rDiffySpecimenGrab = 0.855;
    public static double lDiffySpecimenPickup = 0.57;
    public static double rDiffySpecimenPickup = 0.82;
    public static double lDiffyFolded = 0.25;
    public static double rDiffyFolded = 0.5;
    public static double lDiffySpecimenPlace = 0.805;
    public static double rDiffySpecimenPlace = 0.805;
    public static double lDiffySpecimenBackPlace = 0.855;
    public static double rDiffySpecimenBackPlace = 0.855;

    //Claw Arm Open/Close
    public static double armClawOpen = 0.65;
    public static double armClawClose = 0.34;

    //Spec Lift
    public static int specLiftToTransfer = 0;
    public static int specLiftToZero = 0;
    public static int specLiftToScore = 1750;
    public static int specLiftToHumanPlayer = 10;
    public static int specLiftToHang = 1700;


    //SpecCLaw
    public static double specClawOpen = 0.15;
    public static double specClawClose = 1.0;


    public static double extendManualIncrements = 0.05;
    public static double extendZero = 0;
    public static double extendFullSample = 0.225;
    public static double extendFullSpecimen = 0.2;

}
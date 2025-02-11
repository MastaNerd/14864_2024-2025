package config.util;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {

    public static double spinnerEject = -1;
    public static double spinnerLoad = 1;
    public static double spinnerStop = 0;

    public static double clawWristInit = 0.12;
    public static double clawWristReturn = 0.12;
    public static double clawWristDeposit = 0.25;
    public static double clawWristGrab = 0.55;
    public static double clawWristPlace = 0.55;
    public static double clawWristBackPlace = 0.55;


    public static double lArmTransfer= 0.066; //.05
    public static double rArmTransfer= lArmTransfer - 0.045;
    public static double lArmDeposit = 0.595;
    public static double rArmDeposit = lArmDeposit - 0.045;
    public static double lArmMiddle = 0.725;
    public static double rArmMiddle = lArmMiddle - 0.045;
    public static double lArmInit = 0.195;
    public static double rArmInit = lArmInit - 0.045;
    public static double lArmSpecimenGrab = 0.855;
    public static double rArmSpecimenGrab = lArmSpecimenGrab - 0.045;
    public static double lArmSpecimenReturn = 0.195;
    public static double rArmSpecimenReturn = lArmSpecimenReturn - 0.045;
    public static double lArmSpecimenPlace = 0.855;
    public static double rArmSpecimenPlace = lArmSpecimenPlace - 0.045;
    public static double lArmSpecimenBackPlace = 0.855;
    public static double rArmSpecimenBackPlace = lArmSpecimenBackPlace - 0.045;



    public static int liftToTransfer = 200;
    public static int specLiftToZero = 0;
    public static int specLiftToHumanPlayer = 25;
    public static int specLiftToScore = 2000;


    public static double specClawOpen = 0.15;
    public static double specClawClose = 1.0;


    public static double extendManualIncrements = 0.05;
    public static double extendZero = 0;
    public static double extendFullSample = 0.225;
    public static double extendFullSpecimen = 0.2;

}
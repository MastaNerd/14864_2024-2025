package pedroPathing.constants;

import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.PINPOINT;

        FollowerConstants.useBrakeModeInTeleOp = true;

        FollowerConstants.leftFrontMotorName = "FL Motor";
        FollowerConstants.leftRearMotorName = "BL Motor";
        FollowerConstants.rightFrontMotorName = "FR Motor";
        FollowerConstants.rightRearMotorName = "BR Motor";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        FollowerConstants.mass = 13.3;

        FollowerConstants.xMovement = 68.46078288045345;
        FollowerConstants.yMovement = 50.05013016052251;

        FollowerConstants.forwardZeroPowerAcceleration = -29.50720189210956;
        FollowerConstants.lateralZeroPowerAcceleration = -64.25179986057246;
        //FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.16,0,0.01,0);
        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.16,0,0.01,0);
        FollowerConstants.useSecondaryTranslationalPID = false;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.1,0,0.01,0); // Not being used, @see useSecondaryTranslationalPID

        //FollowerConstants.headingPIDFCoefficients.setCoefficients(1.8,0,0.02,0);
        FollowerConstants.headingPIDFCoefficients.setCoefficients(1.8,0,0.02,0);
        FollowerConstants.useSecondaryHeadingPID = false;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(1.8,0,0.1,0); // Not being used, @see useSecondaryHeadingPID

        //FollowerConstants.drivePIDFCoefficients.setCoefficients(0.005,0,0,0.6,0);
        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.005,0,0,0.6,0);
        FollowerConstants.useSecondaryDrivePID = false;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.1,0,0,0.6,0); // Not being used, @see useSecondaryDrivePID

        FollowerConstants.zeroPowerAccelerationMultiplier = 4;
        //FollowerConstants.centripetalScaling = 0.0005;
        FollowerConstants.centripetalScaling = 0.0005;

        FollowerConstants.pathEndTimeoutConstraint = 100;
        FollowerConstants.pathEndTValueConstraint = 0.95;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;
    }
}

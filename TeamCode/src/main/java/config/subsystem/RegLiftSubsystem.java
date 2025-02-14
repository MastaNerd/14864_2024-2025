package config.subsystem;

import static config.util.RobotConstants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import config.util.RobotConstants;

public class RegLiftSubsystem {
    private Telemetry telemetry;

    public DcMotor rightLift, leftLift;
    public boolean manual = false;
    public boolean hang = false;
    public int pos, bottom;
    public PIDController liftPID;
    public static int target;
    public static double p = 0.01, i = 0, d = 0;
    public static double f = 0.005;


    public RegLiftSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        rightLift = hardwareMap.get(DcMotor.class, "Right Arm Motor");
        leftLift = hardwareMap.get(DcMotor.class, "Left Arm Motor");

        rightLift.setDirection(DcMotor.Direction.REVERSE);
        leftLift.setDirection(DcMotor.Direction.FORWARD);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        liftPID = new PIDController(p, i, d);

    }

    public void updatePIDF(){
        if (!manual) {
            liftPID.setPID(p,i,d);

            rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            double pid = liftPID.calculate(getPos(), target);
            double ticks_in_degrees = 537.7 / 360.0;
            double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;
            double power = pid + ff;

            rightLift.setPower(power);
            leftLift.setPower(power);

            telemetry.addData("lift pos", getPos());
            telemetry.addData("lift target", target);
        }
    }

    public void manual(double n){
        manual = true;

        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if(hang) {
            n = -0.75;
        }

        rightLift.setPower(n);
        leftLift.setPower(n);
    }

    //Util
    public void targetCurrent() {
        setTarget(getPos());
        manual = false;
    }

    public double getTarget() {
        return target;
    }

    public void setTarget(int b) {
        target = b;
    }

    public void addToTarget(int b) {
        target += b;
    }

    public int getPos() {
        pos = rightLift.getCurrentPosition() - bottom;
        return rightLift.getCurrentPosition() - bottom;
    }

    // OpMode
    public void init() {
        liftPID.setPID(p,i,d);
        bottom = getPos();
    }

    public void start() {
        target = 0;
    }

    //Presets

    public void toZero() {
        manual = false;
        setTarget(liftToZero);
    }

    public void toHighBucket() {
        manual = false;
        setTarget(liftToHighBucket);
    }

}
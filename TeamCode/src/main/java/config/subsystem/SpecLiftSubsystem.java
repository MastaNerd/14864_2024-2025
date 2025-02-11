
package config.subsystem;

import static config.util.RobotConstants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import config.util.RobotConstants;
import config.util.action.RunAction;

public class SpecLiftSubsystem {
    private Telemetry telemetry;

    public DcMotor specLift;
    public boolean manual = false;
    public boolean hang = false;
    public int pos, bottom;
    public PIDController liftPID;
    public static int target;
    public static double p = 0.01, i = 0, d = 0;
    public static double f = 0.005;


    public SpecLiftSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        specLift = hardwareMap.get(DcMotor.class, "Specimen Slide Motor");
        specLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        specLift.setDirection(DcMotorSimple.Direction.REVERSE);

        liftPID = new PIDController(p, i, d);

    }

    public void updatePIDF(){
        if (!manual) {
            liftPID.setPID(p,i,d);

            specLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            double pid = liftPID.calculate(getPos(), target);
            double ticks_in_degrees = 537.7 / 360.0;
            double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;
            double power = pid + ff;

            specLift.setPower(power);

            telemetry.addData("lift pos", getPos());
            telemetry.addData("lift target", target);
        }
    }

    public void manual(double n){
        manual = true;

        specLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if(hang) {
            n = -0.75;
        }

        specLift.setPower(n);
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
        pos = specLift.getCurrentPosition() - bottom;
        return specLift.getCurrentPosition() - bottom;
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
        setTarget(specLiftToZero);
    }

    public void toScore() {
        manual = false;
        setTarget(specLiftToScore);
    }


    public void toHumanPlayer() {
        setTarget(specLiftToHumanPlayer);
    }

    public void toTransfer() {
        setTarget(liftToTransfer);
    }

    public void toPark() {
        setTarget(specLiftToZero);
    }

}

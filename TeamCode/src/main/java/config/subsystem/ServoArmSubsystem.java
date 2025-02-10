package config.subsystem;

import static config.util.RobotConstants.*;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ServoArmSubsystem {

    public enum ArmState {
        TRANSFER, SCORING, INIT, SPECIMENGRAB, SPECIMENPLACE, SPECIMENBACKPLACE, SPECIMENRETURN
    }

    public Servo LeftArmServo, RightArmServo, ClawWrist;
    public CRServo ClawSpinner;
    public ArmState state;
    public ElapsedTime timer;

    public ServoArmSubsystem(HardwareMap hardwareMap, ArmState state) {
        LeftArmServo = hardwareMap.get(Servo.class, "LeftArmServo");
        RightArmServo = hardwareMap.get(Servo.class, "RightArmServo");
        ClawSpinner = hardwareMap.get(CRServo.class, "ClawSpinner");
        ClawWrist = hardwareMap.get(Servo.class, "ClawWrist");
        LeftArmServo.setDirection(Servo.Direction.REVERSE);
        ClawSpinner.setDirection(CRServo.Direction.REVERSE);
        this.state = state;

    }

    public void setArmPos(double lTargetAngle, double rTargetAngle){
        double lCurrentAngle = LeftArmServo.getPosition();
        double rCurrentAngle = RightArmServo.getPosition();


        double angle_step = 5;

        double lastTime = timer.seconds();

        while (Math.abs(lCurrentAngle - lTargetAngle) < 0.01) {

            if (timer.seconds() - lastTime > 0.1) {

                lastTime = timer.seconds();

                if (lCurrentAngle < lTargetAngle) {

                    lCurrentAngle = lTargetAngle + angle_step;
                    rCurrentAngle = rTargetAngle + angle_step;

                }
                LeftArmServo.setPosition(lCurrentAngle);
                RightArmServo.setPosition(rCurrentAngle);
            }
        }
    }

    // State //
    public void setState(ArmState armState) {
        if (armState == ArmState.TRANSFER) {
            LeftArmServo.setPosition(lArmTransfer);
            RightArmServo.setPosition(rArmTransfer);
            ClawSpinner.setPower(spinnerStop);
            ClawWrist.setPosition(clawWristInit);
            this.state = ArmState.TRANSFER;
        } else if (armState == ArmState.SCORING) {
            LeftArmServo.setPosition(lArmDeposit);
            RightArmServo.setPosition(rArmDeposit);
            ClawSpinner.setPower(spinnerEject);
            ClawWrist.setPosition(clawWristDeposit);
            this.state = ArmState.SCORING;
        } else if (armState == ArmState.INIT) {
            LeftArmServo.setPosition(lArmInit);
            RightArmServo.setPosition(rArmInit);
            ClawSpinner.setPower(spinnerStop);
            ClawWrist.setPosition(clawWristInit);
            this.state = ArmState.INIT;
        } else if (armState == ArmState.SPECIMENGRAB) {
            LeftArmServo.setPosition(lArmSpecimenGrab);
            RightArmServo.setPosition(rArmSpecimenGrab);
            ClawSpinner.setPower(spinnerLoad);
            ClawWrist.setPosition(clawWristGrab);
            this.state = ArmState.SPECIMENGRAB;
        } else if (armState == ArmState.SPECIMENBACKPLACE) {
            LeftArmServo.setPosition(lArmSpecimenBackPlace);
            RightArmServo.setPosition(rArmSpecimenBackPlace);
            ClawSpinner.setPower(spinnerEject);
            ClawWrist.setPosition(clawWristBackPlace);
            this.state = ArmState.SPECIMENBACKPLACE;
        } else if (armState == ArmState.SPECIMENPLACE) {
            LeftArmServo.setPosition(lArmSpecimenPlace);
            RightArmServo.setPosition(rArmSpecimenPlace);
            ClawSpinner.setPower(spinnerEject);
            ClawWrist.setPosition(clawWristPlace);
            this.state = ArmState.SPECIMENPLACE;
        }else if (armState == ArmState.SPECIMENRETURN) {
            LeftArmServo.setPosition(lArmSpecimenReturn);
            RightArmServo.setPosition(rArmSpecimenReturn);
            ClawSpinner.setPower(spinnerStop);
            ClawWrist.setPosition(clawWristReturn);
            this.state = ArmState.SPECIMENRETURN;
        }
    }

    public void switchState() {
        if (state == ArmState.TRANSFER) {
            setState(ArmState.SCORING);
        } else if (state == ArmState.SCORING) {
            setState(ArmState.TRANSFER);
        }
    }

    // Preset //

    public void transfer() {
        setState(ArmState.TRANSFER);
    }

    public void score() {
        setState(ArmState.SCORING);
    }

    public void specimenGrab() {
        setState(ArmState.SPECIMENGRAB);
    }

    public void specimenPlace() {
        setState(ArmState.SPECIMENPLACE);
    }

    public void specimenBackPlace() {
        setState(ArmState.SPECIMENBACKPLACE);
    }

    public void specimenReturn() {
        setState(ArmState.SPECIMENRETURN);
    }

    public void initArm() {setState(ArmState.INIT);}

    // Util //
    public void setPos(double armPos) {
        LeftArmServo.setPosition(armPos);
        RightArmServo.setPosition(armPos);
    }

    // Init + Start //
    public void init() {
        initArm();
        timer = new ElapsedTime();
    }

    public void start() {
        transfer();
    }

}
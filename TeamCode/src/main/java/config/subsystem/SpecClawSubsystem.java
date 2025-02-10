package config.subsystem;
import static config.util.RobotConstants.*;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SpecClawSubsystem {

    public enum ClawGrabState {
        CLOSED, OPEN
    }


    public Servo specClaw;
    public ClawGrabState grabState;

    public SpecClawSubsystem(HardwareMap hardwareMap, ClawGrabState clawGrabState) {
        specClaw = hardwareMap.get(Servo.class, "Specimen Servo");
        this.grabState = clawGrabState;
    }


    public void setGrabState(ClawGrabState clawGrabState) {
        if (clawGrabState == ClawGrabState.CLOSED) {
            specClaw.setPosition(specClawClose);
            this.grabState = ClawGrabState.CLOSED;
        } else if (clawGrabState == ClawGrabState.OPEN) {
            specClaw.setPosition(specClawOpen);
            this.grabState = ClawGrabState.OPEN;
        }
    }

    public void switchGrabState() {
        if (grabState == ClawGrabState.CLOSED) {
            setGrabState(ClawGrabState.OPEN);
        } else if (grabState == ClawGrabState.OPEN) {
            setGrabState(ClawGrabState.CLOSED);
        }
    }

    public void open() {
        setGrabState(ClawGrabState.OPEN);
    }

    public void close() {
        setGrabState(ClawGrabState.CLOSED);
    }

    public void init() {
        close();
    }

    public void start() {
        close();
    }
}
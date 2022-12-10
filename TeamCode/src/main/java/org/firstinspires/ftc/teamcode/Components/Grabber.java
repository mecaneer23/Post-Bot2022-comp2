package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Base.Component;

public class Grabber implements Component {
    private final Servo grabber;

    private final Telemetry telemetry;
    private final LinearOpMode opMode;

    public double OPEN;
    public double CLOSED;

    public boolean isGrabbing = false;

    public Grabber(LinearOpMode opMode, String deviceName, HardwareMap hardwareMap, Telemetry telemetry) {
        grabber = hardwareMap.get(Servo.class, deviceName);
        this.OPEN = 1;
        this.CLOSED = 0.7;
        this.telemetry = telemetry;
        this.opMode = opMode;
    }

    @Override
    public void init() {
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {
        updatePos();
    }

    @Override
    public String getTelemetry() {
        return telemetry.toString();
    }

    public void toggle() {
        this.isGrabbing = !this.isGrabbing;
    }

    public void open() {
        this.isGrabbing = false;
        updatePos();
    }

    public void close() {
        this.isGrabbing = true;
        updatePos();
    }

    public void updatePos() {
        grabber.setPosition(this.isGrabbing ? CLOSED : OPEN);
    }
}

package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Base.Component;

public class Grabber implements Component {
    private final Servo grabber;

    private final Telemetry telemetry;

    public double OPEN;
    public double CLOSED;

    public boolean isGrabbing = false;

    public Grabber(String deviceName, HardwareMap hardwareMap, Telemetry telemetry) {
        grabber = hardwareMap.get(Servo.class, deviceName);
        this.OPEN = 1;
        this.CLOSED = 0.7;
        this.telemetry = telemetry;
    }

    @Override
    public void init() {

    }

    @Override
    public void start() {

    }

    @Override
    public void update() {
        grabber.setPosition(isGrabbing ? CLOSED : OPEN);
    }

    @Override
    public String getTelemetry() {
        return telemetry.toString();
    }

    public void toggle() {
        isGrabbing = !isGrabbing;
    }

    public void open() {
        grabber.setPosition(OPEN);
    }

    public void close() {
        grabber.setPosition(CLOSED);
    }
}

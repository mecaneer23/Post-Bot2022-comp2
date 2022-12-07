package org.firstinspires.ftc.teamcode.Base;

public interface Component {

    void init();
    void start();
    void update() throws InterruptedException;

    String getTelemetry();
}

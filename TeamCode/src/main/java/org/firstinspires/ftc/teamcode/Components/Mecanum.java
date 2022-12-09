package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Base.Component;
import org.firstinspires.ftc.teamcode.RMath.*;

public class Mecanum implements Component{
    public DcMotor fl;
    public DcMotor fr;
    public DcMotor bl;
    public DcMotor br;

    public double flPower;
    public double frPower;
    public double blPower;
    public double brPower;

    public Mecanum(HardwareMap hardwareMap, String fl, String fr, String bl, String br, Telemetry telemetry) {
        this.fl = hardwareMap.dcMotor.get(fl);
        this.fr = hardwareMap.dcMotor.get(fr);
        this.bl = hardwareMap.dcMotor.get(bl);
        this.br = hardwareMap.dcMotor.get(br);

        this.fl.setDirection(DcMotorSimple.Direction.REVERSE);
        this.fr.setDirection(DcMotorSimple.Direction.FORWARD);
        this.bl.setDirection(DcMotorSimple.Direction.REVERSE);
        this.br.setDirection(DcMotorSimple.Direction.FORWARD);

        this.fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void move(double fl, double fr, double bl, double br) {
        flPower = fl;
        frPower = fr;
        blPower = bl;
        brPower = br;
    }

    public void polarDrive(double speed, double direction, double rotVel){

//        This needs to be - pi/4 because inputting 0 needs to be strafing sideways
        direction -= Math.PI / 4.0;


        double fl = speed * Math.cos(direction) + rotVel;
        double fr = speed * Math.sin(direction) - rotVel;
        double bl = speed * Math.sin(direction) + rotVel;
        double br = speed * Math.cos(direction) - rotVel;

        double maxWheelPower = Math.abs(fr);
        if(Math.abs(fl) > maxWheelPower) maxWheelPower = Math.abs(fl);
        if(Math.abs(bl) > maxWheelPower) maxWheelPower = Math.abs(bl);
        if(Math.abs(br) > maxWheelPower) maxWheelPower = Math.abs(br);
        if(Math.abs(fr) > maxWheelPower) maxWheelPower = Math.abs(fr);

        maxWheelPower = Math.min(maxWheelPower, speed + Math.abs(rotVel));

        final double multiplier = (speed + Math.abs(rotVel)) / maxWheelPower;

        flPower = fl * multiplier;
        frPower = fr * multiplier;
        blPower = bl * multiplier;
        brPower = br * multiplier;
    }


    public void drive(double x, double y, double rot){
        final double direction = Math.atan2(y, x);
        final double speed = Util.dist(x, y);

        polarDrive(speed, direction, rot);
    }

    public void drive(Gamepad gamepad) {
        double leftStickX = gamepad.left_stick_x;
        double leftStickY = gamepad.left_stick_y;
        double rightStickX = gamepad.right_stick_x;

        final double direction = Math.atan2(-leftStickY, leftStickX);
        final double speed = Util.dist(leftStickX, leftStickY);
        polarDrive(speed, direction, rightStickX);
    }

    @Override
    public void init() {

    }

    @Override
    public void start() {

    }

    @Override
    public void update() {
        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);
    }

    @Override
    public String getTelemetry() {
        return "";
    }
}

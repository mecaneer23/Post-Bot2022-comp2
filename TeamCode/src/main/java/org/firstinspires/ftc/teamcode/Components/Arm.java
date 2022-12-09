package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Base.Component;

public class Arm implements Component {
    private final DcMotor rightArm;
    private final DcMotor leftArm;

    private final Telemetry telemetry;

    public double PULSES_PER_REVOLUTION;
    public int LOWER_BOUND;
    public int ZERO_POSITION;
    public int GROUND_JUNCTION;
    public int PICKUP;
    public int LOW_JUNCTION;
    public int MEDIUM_JUNCTION;
    public int HIGH_JUNCTION;
    public int UPPER_BOUND;

    public double MotorPower;
    public int TotalTicks, StartingPosition;
    public boolean isTeleOp;

    public Arm(
            String rightArmName,
            String leftArmName,
            HardwareMap hardwareMap,
            Telemetry telemetry,
            boolean isTeleOp,
            double lowerBound,
            double zeroPosition,
            double groundJunction,
            double pickup,
            double lowJunction,
            double mediumJunction,
            double highJunction,
            double upperBound
    ) {
        rightArm = hardwareMap.get(DcMotor.class, rightArmName);
        leftArm = hardwareMap.get(DcMotor.class, leftArmName);

        rightArm.setDirection(DcMotorSimple.Direction.FORWARD);
        leftArm.setDirection(DcMotorSimple.Direction.FORWARD);

        this.PULSES_PER_REVOLUTION = 384.5; // gobilda 5202 435 rpm
        this.LOWER_BOUND = (int) (lowerBound * PULSES_PER_REVOLUTION);
        this.ZERO_POSITION = (int) (zeroPosition * PULSES_PER_REVOLUTION);
        this.GROUND_JUNCTION = (int) (groundJunction * PULSES_PER_REVOLUTION);
        this.LOW_JUNCTION = (int) (lowJunction * PULSES_PER_REVOLUTION);
        this.PICKUP = (int) (pickup * PULSES_PER_REVOLUTION);
        this.MEDIUM_JUNCTION = (int) (mediumJunction * PULSES_PER_REVOLUTION);
        this.HIGH_JUNCTION = (int) (highJunction * PULSES_PER_REVOLUTION);
        this.UPPER_BOUND = (int) (upperBound * PULSES_PER_REVOLUTION);
        this.telemetry = telemetry;
        this.isTeleOp = isTeleOp;
    }

    @Override
    public void init() {
        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        move(ZERO_POSITION);
    }

    @Override
    public void start() {
    }

    @Override
    public void update() {
        telemetry.addData("Position", getCurrentPosition());
        telemetry.addData("Target", getTargetPosition());
        if (isTeleOp) {
            if (isBusy()) {
                setPower(MotorPower);
//                setPower(((-4.0 * MotorPower) / Math.pow(TotalTicks, 2.0)) * Math.pow(TotalTicks / 2.0 - getCurrentPosition(), 2.0) + MotorPower);
            } else {
                setPower(0);
                move(getTargetPosition());
            }
        } else {
            if (getCurrentPosition() != getTargetPosition()) move(getTargetPosition());
        }
    }

    @Override
    public String getTelemetry() {
        return this.telemetry.toString();
    }

    public void toZero() {
        move(ZERO_POSITION);
    }

    public void toGround() {
        move(GROUND_JUNCTION);
    }

    public void toPickup() {
        move(PICKUP);
    }

    public void toLow() {
        move(LOW_JUNCTION);
    }

    public void toMedium() {
        move(MEDIUM_JUNCTION);
    }

    public void toHigh() {
        move(HIGH_JUNCTION);
    }

    public void move(int position) {
        move(position, 1);
    }

    public void move(int position, double motorPower) {
        leftArm.setTargetPosition(position);
        rightArm.setTargetPosition(position);
        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorPower = motorPower;
        TotalTicks = position;
        StartingPosition = getCurrentPosition();
        if (!isTeleOp) {
            while (isBusy()) {
                setPower(MotorPower);
            }
            setPower(0);
        }
    }

    public void setPower(double motorPower) {
        rightArm.setPower(motorPower);
        leftArm.setPower(motorPower);
    }

    public boolean isBusy() {
        return leftArm.isBusy() || rightArm.isBusy();
    }

    public int getCurrentPosition() {
        return (leftArm.getCurrentPosition() + rightArm.getCurrentPosition()) / 2;
    }

    public int getTargetPosition() {
        return leftArm.getTargetPosition();
    }
}
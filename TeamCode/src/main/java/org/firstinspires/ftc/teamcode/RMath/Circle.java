package org.firstinspires.ftc.teamcode.RMath;

public class Circle {
    public double h;
    public double k;
    public double r;

    public Circle(double h, double k, double r) {
        this.h = h;
        this.k = k;
        this.r = r;
    }

    public Circle(Point center, double r) {
        this.h = center.x;
        this.k = center.y;
        this.r = r;
    }
}
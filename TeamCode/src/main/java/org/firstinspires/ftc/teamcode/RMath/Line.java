package org.firstinspires.ftc.teamcode.RMath;

public class Line {
    private boolean isVertical;
    private double m;
    private double b;

    //    Standard form
    public Line(double m, double b) {
        isVertical = Math.abs(m) == Double.POSITIVE_INFINITY;
        this.m = m;
        this.b = b;
    }

    //    Vertical line
    public Line(double x) {
        isVertical = true;
        this.m = Double.POSITIVE_INFINITY;
        this.b = x;
    }

    //    Util.Point slope form
    public Line(Point p, double m) {
        isVertical = Math.abs(m) == Double.POSITIVE_INFINITY;
        this.m = m;
        if (isVertical) {
            this.b = p.x;
        } else {
            this.b = p.y - m * p.x;
        }
    }

    //    Two point form
    public Line(Point p1, Point p2) {
        isVertical = p1.x == p2.x;
        if (isVertical) {
            m = Double.POSITIVE_INFINITY;
            b = p1.x;
        } else {
            m = (p2.y - p1.y) / (p2.x - p1.x);
            b = p1.y - m * p1.x;
        }
    }

    public double getM() {
        return m;
    }

    public double getB() {
        return b;
    }

    public double getXIntercept() {
        return Util.getIntersection(this, new Line(0, 0)).x;
    }

    public double getYIntercept() {
        return Util.getIntersection(this, new Line(0)).y;
    }

    public double perpendicular() {
        if (isVertical) return 0;
        else return -1 / m;
    }

    public boolean isVertical() {
        return isVertical;
    }
}
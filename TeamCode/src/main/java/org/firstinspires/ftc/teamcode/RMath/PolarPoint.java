package org.firstinspires.ftc.teamcode.RMath;

public class PolarPoint {

    public double r;
    public double theta;

    public PolarPoint(double r, double theta){
        this.r = r;
        this.theta = theta;
    }

    public PolarPoint(Point p){
        r = Util.dist(p.x, p.y);
        theta = Util.angle(p.x, p.y);
    }

    public double getR() {
        return r;
    }

    public void setR(double r) {
        this.r = r;
    }

    public double getTheta() {
        return theta;
    }

    public void setTheta(double theta) {
        this.theta = theta;
    }

    public void translateR(double dr){
        r += dr;
    }

    public void translateTheta(double dt){
        theta += dt;
    }

    public String toString(){
        return "(" + r + ", " + theta + ")";
    }

}
package org.firstinspires.ftc.teamcode.RMath;

public class Vector implements Cloneable{

    private double x;
    private double y;
    private double r;
    private double theta;

    public Vector(Point p){
        x = p.x;
        y = p.y;
        r = Util.dist(p.x, p.y);
        theta = Util.angle(p.x, p.y);
    }

    public Vector(PolarPoint p){
        r = p.r;
        theta = p.theta;
        x = r * Math.cos(theta);
        y = r * Math.sin(theta);
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getMagnitude() {
        return r;
    }

    public double getTheta() {
        return theta;
    }

    public void setX(double x) {
        this.x = x;
        this.r = Util.dist(x, y);
        this.theta = Util.angle(x, y);
    }

    public void setY(double y) {
        this.y = y;
        this.r = Util.dist(x, y);
        this.theta = Util.angle(x, y);
    }

    public void setMagnitude(double r) {
        this.r = r;
        this.x = r * Math.cos(theta);
        this.y = r * Math.sin(theta);
    }

    public void setTheta(double theta) {
        this.theta = theta;
        this.x = r * Math.cos(theta);
        this.y = r * Math.sin(theta);
    }

    public Vector normalize(){
        final double mag = getMagnitude();
        return new Vector(new Point(x / mag, y / mag));
    }

    public Vector scale(double scale){
        return new Vector(new Point(x * scale, y * scale));
    }

    public Vector clone(){
        return new Vector(new Point(x, y));
    }

    public Vector add(Vector other){
        return new Vector(new Point(x + other.x, y + other.y));
    }

    public Vector subtract(Vector other){
        return new Vector(new Point(x - other.x, y - other.y));
    }

    public Point toPoint(){
        return new Point(x, y);
    }

    public PolarPoint toPolarPoint(){
        return new PolarPoint(r, theta);
    }

    public Segment toSegment(){
        return new Segment(new Point(0,0), toPoint());
    }

    public Segment toSegment(Point start){
        return new Segment(start, toPoint());
    }
}
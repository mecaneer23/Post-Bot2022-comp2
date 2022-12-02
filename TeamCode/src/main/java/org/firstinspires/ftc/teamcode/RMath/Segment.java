package org.firstinspires.ftc.teamcode.RMath;

public class Segment {

    private Point p1, p2;
    private Line l;
    private boolean inclusive;

    public Segment(Point p1, Point p2) {
        this.p1 = p1;
        this.p2 = p2;
        l = new Line(p1, p2);
        inclusive = false;
    }

    public Segment(Point p1, Point p2, boolean inclusive) {
        this.p1 = p1;
        this.p2 = p2;
        l = new Line(p1, p2);
        this.inclusive = inclusive;
    }

    public Line getLine() {
        return l;
    }

    public double getLength(){
        return Util.dist(p1, p2);
    }

    public boolean isWithin(Point p){
        if(l.isVertical()){
            return Util.isWithin(p.y, p1.y, p2.y, inclusive);
        } else{
            return Util.isWithin(p.x, p1.x, p2.x, inclusive);
        }
    }

    public Point getP1() {
        return p1;
    }

    public Point getP2() {
        return p2;
    }
}
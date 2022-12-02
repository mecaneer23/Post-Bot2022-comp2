package org.firstinspires.ftc.teamcode.RMath;

public class Point{
    public double x;
    public double y;

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }
    public Point(double[] p){
        this.x = p[0];
        this.y = p[1];
    }

    public Point(PolarPoint p) {
        x = Math.cos(p.getTheta()) * p.getR();
        y = Math.sin(p.getTheta()) * p.getR();
    }

    public Point closestPoint(Line l) {

        return Util.getIntersection(l, new Line(this, l.perpendicular()));

    }

    public Point closestPoint(Segment s) {
        Point cp = closestPoint(s.getLine());
        if (s.isWithin(cp))
            return cp;
        else if (Util.dist(s.getP1(), this) < Util.dist(s.getP2(), this))
            return s.getP1();
        else
            return s.getP2();
    }

    public Point closestPoint(Circle c) {
        Line l = new Line(this, new Point(c.h, c.k));
        Point[] i = Util.getIntersection(c, l);
        Point closest = i[0];
        for (Point p : i)
            if (Util.dist(p, this) < Util.dist(closest, this))
                closest = p;

        return closest;
    }

    public Point getRelation(Point p){
        Point out;
        out = new Point(p.x - x, p.y - y);

        return out;
    }


    public String toString() {
        return "(" + x + ", " + y + ")";
    }
}
package org.firstinspires.ftc.teamcode.RMath;

import java.util.ArrayList;

public class Util {

    public static double dist(double x, double y){
        return Math.sqrt(x * x + y * y);
    }

    public static double dist(double x2, double x1, double y2, double y1){
        return dist(x2 - x1, y2 - y1);
    }

    public static double dist(Point p1, Point p2){
        return dist(p2.x, p1.x, p2.y, p1.y);
    }

    public static double angle(double x, double y){
        return Math.atan2(y, x);
    }

    public static double angle(double x2, double x1, double y2, double y1){
        return angle(x2 - x1, y2 - y1);
    }

    public static double angle(Point p1, Point p2){
        return angle(p2.x, p1.x, p2.y, p1.y);
    }

    public static double loop(double d, double min, double max){
        double mod = max - min;
        return (((d - min) % mod) + mod) % mod + min;
    }

    public static double cap(double d, double min, double max){
        return d > max? max : d < min? min : d;
    }

    public static double absCap(double d, double min, double max){
        min = Math.abs(min);
        max = Math.abs(max);
        d = cap(d, -max, max);
        d = Math.abs(d) < min? min * sign(d) : d;

        return d;
    }

    public static int sign(double d){
        double out = d / Math.abs(d);
        if (Double.isNaN(out)) return 0;
        else return (int)out;
    }

    public static boolean isWithin(double d, double b1, double b2){
        return d > Math.min(b1, b2) && d < Math.max(b1, b2);
    }

    public static boolean isWithin(double d, double b1, double b2, boolean inclusive){
        if (inclusive) return d >= Math.min(b1, b2) && d <= Math.max(b1, b2);
        else return isWithin(d, b1, b2);
    }

    public static Point getIntersection(Line l1, Line l2){

        if (l1.isVertical() || l2.isVertical())
            if (l1.isVertical() && l2.isVertical())
                if (l1.getB() == l2.getB())
                    return new Point(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
                else
                    return new Point(Double.NaN, Double.NaN);

            else if (l1.isVertical())
                return new Point(l1.getB(), l2.getM() * l1.getB() + l2.getB());
            else
                return new Point(l2.getB(), l1.getM() * l2.getB() + l1.getB());

        else if (l2.getM() == l1.getM())
            if (l1.getB() == l2.getB())
                return new Point(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
            else
                return new Point(Double.NaN, Double.NaN);


        double x = (l2.getB() - l1.getB()) / (l1.getM() - l2.getM());
        double y = l1.getM() * x + l1.getB();

        return new Point(x, y);

    }

    public static Point getIntersection(Segment s1, Segment s2){
        Point i1 = getIntersection(s1, s2.getLine());
        Point i2 = getIntersection(s2, s1.getLine());

        if(Double.isNaN(i1.x) || Double.isNaN(i2.x)){
            return new Point(Double.NaN, Double.NaN);
        }

        return i1;
    }

    public static Point getIntersection(Segment s, Line l){
        Point i = getIntersection(s.getLine(), l);
        if (Double.isNaN(i.x)) {
            return i;
        } else {
            if (!s.isWithin(i))
                return new Point(Double.NaN, Double.NaN);
            else
                return i;
        }
    }
    public static Point getIntersection(Line l, Segment s){
        return getIntersection(s, l);
    }

    public static Point[] getIntersection (Circle c, Line l){
        double x1;
        double y1;
        double x2;
        double y2;

        double r = c.r;
        double h = c.h;
        double k = c.k;

        if(l.isVertical()){
            x1 = l.getB();
            x2 = l.getB();

            y1 = Math.sqrt((r + x1 - h) * (r - x1 + h)) + k;
            y2 = -Math.sqrt((r + x2 - h) * (r - x2 + h)) + k;

        }else{
            double m = l.getM();
            double b = l.getB();
            double a2 = 1 + m * m;
            double b2 = 2 * (m * b - h - m * k);
            double c2 = b * b + k * k + h * h - r * r - 2 * b * k;

            double disc = Math.sqrt(b2 * b2 - 4 * a2 * c2);
            x1 = (-b2 + disc) / (2 * a2);
            y1 = m * x1 + b;
            x2 = (-b2 - disc) / (2 * a2);
            y2 = m * x2 + b;
        }

        if(Double.isNaN(y2)){
            return new Point[]{
                    new Point(Double.NaN, Double.NaN)
            };
        }

        return new Point[]{
                new Point(x1, y1),
                new Point(x2, y2)
        };
    }
    public static Point[] getIntersection (Line l, Circle c){
        return getIntersection(c, l);
    }

    public static Point[] getIntersection (Circle c, Segment s){
        Point[] i = getIntersection(c, s.getLine());
        ArrayList<Point> temp = new ArrayList<>();

        for(Point p : i)
            if(s.isWithin(p))
                temp.add(p);
        Point[] out = new Point[temp.size()];

        for(int j = 0; j < temp.size(); j++)
            out[j] = temp.get(j);

        if(temp.size() == 0)
            return new Point[]{new Point(Double.NaN, Double.NaN)};


        return out;
    }
    public static Point[] getIntersection (Segment s, Circle c){
        return getIntersection(c, s);
    }

    public static double angleDiff(double a1, double a2) {
        a1 = Util.loop(a1, 0, 360);
        a2 = Util.loop(a2, 0, 360);

        double dist = a1 - a2;
        double shortest;
        if (Math.abs(dist) < 180)
            shortest = dist;
        else {
            if (dist > 0) shortest = dist - 360;
            else shortest = dist + 360;
        }

        return shortest;
    }


}
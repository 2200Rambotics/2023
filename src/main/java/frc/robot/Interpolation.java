package frc.robot;


public class Interpolation {

    Point[] points;

    public Interpolation(Point[] points){

        this.points = points;
    }

    public double interpolate(double x){
        for(int i = 0; i < points.length-1; i++){
            Point point1 = points[i];
            Point point2 = points[i+1];

            if(x >= point1.x && x < point2.x){
                return EthanMath.map(x, point1.x, point2.x, point1.y, point2.y);
            }
        }
        if ( x < points[0].x){
            Point point1 = points[0];
            Point point2 = points[1];
            
            return EthanMath.map(x, point1.x, point2.x, point1.y, point2.y);
        }else {
            int length = points.length;
            Point point1 = points[length-2];
            Point point2 = points[length-1];
            
            return EthanMath.map(x, point1.x, point2.x, point1.y, point2.y);
        }

    }

}

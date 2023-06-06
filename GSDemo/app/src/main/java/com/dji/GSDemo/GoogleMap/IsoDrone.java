package com.dji.GSDemo.GoogleMap;

import static java.lang.Math.round;

import android.util.Log;
import android.widget.EditText;

import org.asta.isoObject.*;
import org.locationtech.proj4j.ProjCoordinate;

public class IsoDrone extends TestObject{

    public TrajectoryWaypointVector reducedTraj;
    private double mReduction = 1.0;
    private static int maxNumOfPoints = 99;
   IsoDrone(String ip) {
       super(ip);

       /* Set iso speed/positional fields to initial value and make sure they are valid */
       CartesianPosition cp = new CartesianPosition();
       cp.setHeading_rad(0);
       cp.setIsHeadingValid(true);
       cp.setXCoord_m(0);
       cp.setYCoord_m(0);
       cp.setZCoord_m(0);
       cp.setIsPositionValid(true);
       cp.setIsXcoordValid(true);
       cp.setIsYcoordValid(true);
       cp.setIsZcoordValid(true);

       SpeedType spd = new SpeedType();
       spd.setLateral_m_s(0);
       spd.setLongitudinal_m_s(0);
       spd.setIsLateralValid(true);
       spd.setIsLongitudinalValid(true);

       AccelerationType acc = new AccelerationType();
       acc.setLateral_m_s2(0);
       acc.setLongitudinal_m_s2(0);
       acc.setIsLateralValid(true);
       acc.setIsLongitudinalValid(true);

       this.setPosition(cp);
       this.setSpeed(spd);
       this.setAcceleration(acc);

       System.out.println("Drone init...ip");

    }
    IsoDrone() {
        super();
        System.out.println("Drone init...");

    }

    @Override
    public void handleAbort() {
        System.out.println("Aborting...");
    }


    @Override
    public String getName() {
        return super.getName();
    }

    public void reducePoints(double epsilon){
        TrajectoryWaypointVector traj = this.getTrajectory();
        this.reducedTraj = douglasPeucker(traj, epsilon);
    }

    public void removePointsToClose(){
        TrajectoryWaypointVector newTraj = new TrajectoryWaypointVector();
        CartesianPosition p1 = this.reducedTraj.get(1).getPos();

        for(int i=1; i<this.reducedTraj.size(); i++){
            CartesianPosition p2 = this.reducedTraj.get(i-1).getPos();
            double distance = Math.sqrt((Math.pow(p1.getXCoord_m() - p2.getXCoord_m(), 2) + Math.pow(p1.getYCoord_m() - p2.getYCoord_m(), 2) + Math.pow(p1.getZCoord_m() - p2.getZCoord_m(), 2)));
            if( java.lang.Math.abs(distance) > getMaxDistancePoints()  ){
                newTraj.add(this.reducedTraj.get(i));
                p1 = this.reducedTraj.get(i).getPos();
            }
        }
        this.reducedTraj = newTraj;
    }

    TrajectoryWaypointVector getReducedTraj(){
        return this.reducedTraj;
    }

    private static final double distanceBetweenPoints(double vx, double vy, double wx, double wy) {
        return Math.pow(vx - wx, 2) + Math.pow(vy - wy , 2);
    }

    private static final double distanceToSegmentSquared(double px, double py, double vx, double vy, double wx, double wy) {
        final double l2 = distanceBetweenPoints(vx, vy, wx, wy);
        if (l2 == 0)
            return distanceBetweenPoints(px, py, vx, vy);
        final double t = ((px - vx) * (wx - vx) + (py - vy) * (wy - vy)) / l2;
        if (t < 0)
            return distanceBetweenPoints(px, py, vx, vy);
        if (t > 1)
            return distanceBetweenPoints(px, py, wx, wy);
        return distanceBetweenPoints(px, py, (vx + t * (wx - vx)), (vy + t * (wy - vy)));
    }

    private static final double perpendicularDistance(double px, double py, double vx, double vy, double wx, double wy) {
        return Math.sqrt(distanceToSegmentSquared(px, py, vx, vy, wx, wy));
    }

    private static final void douglasPeucker(TrajectoryWaypointVector traj, int s, int e, double epsilon, TrajectoryWaypointVector resultTraj) {
        // Find the point with the maximum distance
        double dmax = 0;
        int index = 0;
        boolean isColinearY = true;
        boolean isColinearX = true;
        boolean isColinear = true;



        final int start = s;
        final int end = e-1;
        for (int i=start+1; i<end; i++) {
            // Point
            final double px = traj.get(i).getPos().getXCoord_m();
            final double py = traj.get(i).getPos().getYCoord_m();
            // Start
            final double vx = traj.get(start).getPos().getXCoord_m();
            final double vy = traj.get(start).getPos().getYCoord_m();
            // End
            final double wx = traj.get(end).getPos().getXCoord_m();
            final double wy = traj.get(end).getPos().getYCoord_m();
            final double d = perpendicularDistance(px, py, vx, vy, wx, wy);
            double res = (wx -vx) * (px -vy) - (wy-vy) * (px-vx);

            if (res != 0) {
                isColinear = false;
            }

            if (vx != px) isColinearX = false;
            if (vy != py) isColinearY = false;
            if (d > dmax) {
                index = i;
                dmax = d;
            }
        }

        // IF straight line, just remove points so we get maxSize traj
        if (isColinearX || isColinearY) {
            //TODO Use the res variable to control if it's colinear or not. Did not work on test, might need tolerances
            int multiplier = Math.round(traj.size() / maxNumOfPoints);
            //Make sure we are not stuck in infinite loop
            if (multiplier < 1) multiplier = 1;
            for (int i = 0; i < 98 && i < traj.size(); i+= 1) {
                resultTraj.add(traj.get(i));
            }
            return;
        }

//         If max distance is greater than epsilon, call recursively
        if (dmax > epsilon) {
            douglasPeucker(traj, s, index, epsilon, resultTraj);
            douglasPeucker(traj, index, e, epsilon, resultTraj);
        } else {
            if ((end-start)>0) {
                resultTraj.add(traj.get(start));
                resultTraj.add(traj.get(end));
            } else {
                resultTraj.add(traj.get(start));
            }
        }
    }

    public static final TrajectoryWaypointVector douglasPeucker(TrajectoryWaypointVector traj, double epsilon) {
        final TrajectoryWaypointVector trajResult = new TrajectoryWaypointVector();
        douglasPeucker(traj, 0, traj.size(), epsilon, trajResult);
        return trajResult;
    }

    public void setMaxDistancePoints(double reduction){
       this.mReduction = reduction;
    }
    public double getMaxDistancePoints(){
       return mReduction;
    }
}






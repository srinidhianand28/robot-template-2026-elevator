/*
 * Copyright 2025 Tahoma Robotics
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

package org.tahomarobotics.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import org.tinylog.Logger;

import java.util.Optional;

public class BlendedPath {

    private static abstract class Segment {
        private final double startDistance;
        private final double segmentDistance;
        protected final Translation2d start;
        protected final Translation2d end;

        public Translation2d getStart() {
            return start;
        }

        public Translation2d getEnd() {
            return end;
        }

        public double getSegmentDistance() {
            return segmentDistance;
        }

        public double getStartDistance() {
            return startDistance;
        }

        public double endDistance() {
            return startDistance + segmentDistance;
        }

        public boolean inSegment(double distance) {
            return distance > getStartDistance() && distance <= endDistance();
        }
        public Segment(double startDistance, Translation2d start, Translation2d end, double segmentDistance) {
            this.startDistance = startDistance;
            this.start = start;
            this.end = end;
            this.segmentDistance = segmentDistance;
        }

        public abstract Translation2d sample(double distance);
    }

    private static class LineSegment extends Segment {

        public static LineSegment create(double startDistance, double blendDistance, Translation2d p0, Translation2d p1, boolean isFirst, boolean isLast) {

            double dist = p0.getDistance(p1);

            if ( isFirst && isLast ) {
                // straight line
                if (dist == 0d) {
                    throw new IllegalArgumentException("distance between points is too small");
                }
            } else if ( isFirst || isLast ) {
                // line with connecting curve
                if (blendDistance > dist) {
                    throw new IllegalArgumentException("distance between points is too small");
                }
            } else {
                // line with two connection curves
                if (blendDistance > 0.5 * dist) {
                    throw new IllegalArgumentException("distance between points is too small");
                }
            }

            Translation2d start = isFirst ? p0 : p0.interpolate(p1, blendDistance / dist);
            Translation2d end = isLast ? p1 : p0.interpolate(p1, 1.0 - blendDistance / dist);

            return new LineSegment(startDistance, start, end);
        }

        public LineSegment(Translation2d start, Translation2d end) {
            this(0, start, end);
        }

        public LineSegment(double startDistance, Translation2d start, Translation2d end) {
            super(startDistance, start, end, start.getDistance(end));
        }

        @Override
        public Translation2d sample(double distance) {
            if (!inSegment(distance)) {
                return end;
            }
            double t = (distance - getStartDistance()) / getSegmentDistance();
            return start.interpolate(end, t);
        }
    }

    private static class CurveSegment extends Segment {

        private final Translation2d center;
        private final double radius;
        private final double totalAngle;
        private final double firstAngle;
        private final double direction;

        private static Segment create(double startDistance, double blendDistance, Translation2d p0, Translation2d p1, Translation2d p2) {

            double dist0 = p0.getDistance(p1);
            double dist1 = p1.getDistance(p2);

            if(dist0 < blendDistance || dist1 < blendDistance) {
                throw new IllegalArgumentException("distance between points is too small");
            }

            Translation2d start = p0.interpolate(p1, 1d - blendDistance /dist0);
            Translation2d end = p1.interpolate(p2, blendDistance/dist1);

            // tangential and perpendicular unit vectors of first line
            Translation2d vec0 =  p1.minus(p0).div(dist0);
            Translation2d norm0 = new Translation2d(-vec0.getY(), vec0.getX());

            // tangential and perpendicular unit vectors of second line
            Translation2d vec1 = p2.minus(p1).div(dist1);
            Translation2d norm1 = new Translation2d(-vec1.getY(), vec1.getX());

            // angle between line using the unit vector dot product
            double totalAngle = Math.acos(vec0.getX() * vec1.getX() + vec0.getY() * vec1.getY());

            // perpendicular vector intersect at curve radius center
            Translation2d center = lineIntersection(start, norm0, end, norm1);

            // radius is simply the distance from center to start of curve
            double radius = center.getDistance(start);

            // calculate angle from center to curve start
            double firstAngle = Math.atan2(start.getY() - center.getY(), start.getX() - center.getX());

            // calculate angle from center to curve end
            double secondAngle = Math.atan2(end.getY() - center.getY(), end.getX() - center.getX());

            // get curve progression (clockwise or counter-clockwise)
            double curveDirection = Math.signum(MathUtil.angleModulus(secondAngle - firstAngle));

            return new CurveSegment(startDistance, start, end, center, radius, totalAngle, firstAngle, curveDirection);
        }

        private static Translation2d lineIntersection(Translation2d p1, Translation2d u1, Translation2d p2, Translation2d u2) {
            // Center point is a distance R perpendicular (u1 and u2) from p1 and p2 respectively
            // c = p1 + u1 * R, c = p2 + u2 * R

            // Solving for R on the 2nd equation and substituting into the 1st equation
            // R = (c - p2)/u2, c = p1 + u1 * (c - p2)/u2

            // Solving for c and simplifying
            // c * (1 - u1/u2) = p1 - u1/u2 * p2
            // c = (p1 - u1/u2 * p2)/(1 - u1/u2)
            // c = (u2 p1 - u1 p2) / (u2 - u1)

            // calculate center point
            double x = (u2.getX() * p1.getX() - u1.getX() * p2.getX()) / (u2.getX() - u1.getX());
            double y = (u2.getY() * p1.getY() - u1.getY() * p2.getY()) / (u2.getY() - u1.getY());
            return new Translation2d(x,y);
        }

        public CurveSegment(double startDistance, Translation2d start, Translation2d end, Translation2d center, double radius, double totalAngle, double firstAngle, double direction) {
            super(startDistance, start, end, radius * totalAngle);
            this.center = center;
            this.radius = radius;
            this.totalAngle = totalAngle;
            this.firstAngle = firstAngle;
            this.direction = direction;
        }

        @Override
        public Translation2d sample(double distance) {
            if (!inSegment(distance)) {
                return end;
            }

            // portion along curved segment
            double t = (distance - getStartDistance()) / getSegmentDistance();

            // angle from curve center
            double theta = firstAngle + direction * totalAngle * t;

            // get point at that angle and radius from center point
            return center.plus(new Translation2d(Math.cos(theta), Math.sin(theta)).times(radius));
        }
    }

    private final Segment[] segments;

    public static Optional<BlendedPath> create(double blendDistance, Translation2d...points) {
        try {
            return Optional.of(new BlendedPath(blendDistance, points));
        } catch (Exception e) {
            Logger.warn(e, "failed to create BlendedPath");
        }
        return Optional.empty();
    }

    /**
     * Creates a segmented path connecting the start and end points with a curve or fillet connecting the first leg and the second starting
     * at distance back from the intercept point.
     *
     */
    private BlendedPath(double blendDistance, Translation2d...points) {

        this.segments = new Segment[(points.length - 1) * 2 - 1];
        Translation2d start;
        Translation2d end;
        double startDistance = 0;

        for (int i = 0; i < points.length; i++) {


            boolean isFirst = i == 0;
            boolean isLast = i == points.length-2;
            segments[2*i] = LineSegment.create(startDistance, blendDistance, points[i], points[i+1], isFirst, isLast);
            startDistance = segments[2*i].endDistance();


            if (isLast) {
                break;
            }

            // create curve segment
            segments[2*i+1] = CurveSegment.create(startDistance, blendDistance, points[i], points[i+1], points[i+2]);
            startDistance = segments[2*i+1].endDistance();

        }
    }

    public Translation2d sample(double distance) {

        // return end point for distance beyond the bounds
        if (distance <= 0) {
            return segments[0].start;
        } else if (distance >= getTotalDistance()) {
            return segments[segments.length - 1].end;
        } else

        for(Segment segment : segments) {
            if (segment.inSegment(distance)) {
                return segment.sample(distance);
            }
        }

        return segments[segments.length - 1].end;
    }

    public double getTotalDistance() {
        return segments[segments.length - 1].endDistance();
    }
}
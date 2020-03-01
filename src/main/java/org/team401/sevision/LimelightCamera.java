package org.team401.sevision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.team401.taxis.geometry.Pose2d;
import org.team401.taxis.geometry.Rotation2d;
import org.team401.taxis.geometry.Translation2d;
import org.team401.taxis.util.Util;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

public class LimelightCamera {
    private static final double[] ZERO_ARRAY = {0, 0, 0, 0, 0, 0, 0, 0};
    private static final double CONSTANT_LATENCY = 0.011;
    private static final double VPW1x = 2.0 * Math.tan(Math.toRadians(59.6 / 2.0));
    private static final double VPH1x = 2.0 * Math.tan(Math.toRadians(49.7 / 2.0));
    private static final double VPW2x = 2.0 * Math.tan(Math.toRadians(59.6 / 4.0));
    private static final double VPH2x = 2.0 * Math.tan(Math.toRadians(49.7 / 4.0));

    private List<TargetInfo> targets = new ArrayList<>();
    private boolean sawTarget = false;
    private double currentVPW = VPW1x;
    private double currentVPH = VPH1x;

    private NetworkTable table;
    private Pose2d originToLens;
    private double lensHeight;
    private Rotation2d horizontalPlaneToLens;

    private NetworkTableEntry tvEntry;
    private NetworkTableEntry tlEntry;
    private NetworkTableEntry tcornxyEntry;
    private NetworkTableEntry ledModeEntry;
    private NetworkTableEntry pipelineEntry;

    public LimelightCamera(
            String name,
            Pose2d originToLens,
            double lensHeight,
            Rotation2d horizontalPlaneToLens
    ) {
        table = NetworkTableInstance.getDefault().getTable("limelight-" + name);
        this.originToLens = originToLens;
        this.lensHeight = lensHeight;
        this.horizontalPlaneToLens = horizontalPlaneToLens;

        tvEntry = table.getEntry("tv");
        tlEntry = table.getEntry("tl");
        tcornxyEntry = table.getEntry("tcornxy");
        ledModeEntry = table.getEntry("ledMode");
        pipelineEntry = table.getEntry("pipeline");
    }

    public Pose2d getOriginToLens() {
        return originToLens;
    }

    public double getLensHeight() {
        return lensHeight;
    }

    public Rotation2d getHorizontalPlaneToLens() {
        return horizontalPlaneToLens;
    }

    public void setOriginToLens(Pose2d originToLens) {
        this.originToLens = originToLens;
    }

    public void setLensHeight(double lensHeight) {
        this.lensHeight = lensHeight;
    }

    public void setHorizontalPlaneToLens(Rotation2d horizontalPlaneToLens) {
        this.horizontalPlaneToLens = horizontalPlaneToLens;
    }

    public synchronized void markPipeline1xZoom() {
        currentVPW = VPW1x;
        currentVPH = VPH1x;
    }

    public synchronized void markPipeline2xZoom() {
        currentVPW = VPW2x;
        currentVPH = VPH2x;
    }

    public synchronized void setPipeline(int pipeline) {
        pipelineEntry.setDouble(pipeline);
    }

    public synchronized void setLedToPipeline() {
        ledModeEntry.setDouble(0);
    }

    public synchronized void setLedOff() {
        ledModeEntry.setDouble(1);
    }

    public synchronized boolean seesTargetNow() {
        return tvEntry.getDouble(0.0) == 1.0;
    }

    public synchronized boolean sawTarget() {
        return sawTarget;
    }

    public synchronized double getLatency() {
        return 0.0;//(tlEntry.getDouble(0.0) * 1000.0) + CONSTANT_LATENCY;
    }

    /**
     * @return two targets that make up one goal or null if no target is found
     */
    public synchronized List<TargetInfo> getTarget() {
        List<TargetInfo> targets = getRawTargetInfos();
        if (sawTarget && targets != null) {
            return targets;
        }

        return null;
    }

    private synchronized List<TargetInfo> getRawTargetInfos() {
        List<double[]> corners = getTopCorners();
        if (corners == null) {
            return null;
        }

        double slope = 1.0;
        if (Math.abs(corners.get(1)[0] - corners.get(0)[0]) > Util.kEpsilon) {
            slope = (corners.get(1)[1] - corners.get(0)[1]) /
                    (corners.get(1)[0] - corners.get(0)[0]);
        }

        targets.clear();
        for (int i = 0; i < 2; ++i) {
            // Average of y and z;
            double y_pixels = corners.get(i)[0];
            double z_pixels = corners.get(i)[1];

            // Redefine to robot frame of reference.
            double nY = -((y_pixels - 160.0) / 160.0);
            double nZ = -((z_pixels - 120.0) / 120.0);

            double y = currentVPW / 2 * nY;
            double z = currentVPH / 2 * nZ;

            TargetInfo target = new TargetInfo(y, z);
            target.setSkew(slope);
            targets.add(target);
        }

        return targets;
    }

    private double[] xCorners = new double[4];
    private double[] yCorners = new double[4];
    private int numCorners = 4;

    /**
     * Returns raw top-left and top-right corners
     *
     * @return list of corners: index 0 - top left, index 1 - top right
     */
    private List<double[]> getTopCorners() {
        double[] xyCorners = tcornxyEntry.getDoubleArray(ZERO_ARRAY);
        numCorners = xyCorners.length / 2;
        if (numCorners > xCorners.length || numCorners > yCorners.length) {
            //Resize arrays
            xCorners = new double[numCorners];
            yCorners = new double[numCorners];
        }

        sawTarget = seesTargetNow();
        // something went wrong
        if (!sawTarget || Arrays.equals(xyCorners, ZERO_ARRAY) || numCorners < 4) {
            return null;
        }

        //Unpack xyCorners into separate arrays
        for (int i = 0; i < xyCorners.length; i += 2) {
            xCorners[i / 2] = xyCorners[i];
        }

        for (int i = 1; i < xyCorners.length; i += 2) {
            yCorners[i / 2] = xyCorners[i];
        }

        return extractTopCornersFromBoundingBox();
    }

    private static final Comparator<Translation2d> xSort = Comparator.comparingDouble(Translation2d::x);
    private static final Comparator<Translation2d> ySort = Comparator.comparingDouble(Translation2d::y);

    /**
     * Returns raw top-left and top-right corners
     *
     * @return list of corners: index 0 - top left, index 1 - top right
     */
    private List<double[]> extractTopCornersFromBoundingBox() {
        List<Translation2d> corners = new ArrayList<>();
        for (int i = 0; i < numCorners; i++) {
            corners.add(new Translation2d(xCorners[i], yCorners[i]));
        }

        corners.sort(xSort);
        corners.sort(ySort);

        List<Translation2d> top = corners.subList(0, 2);

        top.sort(xSort);

        Translation2d leftCorner = top.get(0);
        Translation2d rightCorner = top.get(1);

        return List.of(new double[]{leftCorner.x(), leftCorner.y()}, new double[]{rightCorner.x(), rightCorner.y()});
    }
}

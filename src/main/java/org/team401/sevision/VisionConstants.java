package org.team401.sevision;

public class VisionConstants {
    public static final double kCameraFrameRate = 90.0;

    public static final double kMaxGoalTrackAge = 6.0;
    public static final double kMaxTrackerDistance = 9.0;
    public static final double kMaxGoalTrackSmoothingTime = 3.0;

    public static final double kTrackStabilityWeight = 0.0;
    public static final double kTrackAgeWeight = 10.0;
    public static final double kTrackSwitchingWeight = 100.0;
}

package frc.robot.lib;

import java.util.ArrayList;
import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;
import java.io.BufferedReader;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * TurretHelpers
 *
 * Static math library: numbers in -> numbers out.
 *
 * What this solver does:
 * 1) Predict robot pose + velocity at the projectile release time.
 * 2) Compute the required initial ball velocity relative to the robot
 * (expressed in the FIELD frame).
 * 3) Convert that velocity into a desired yaw (field), desired pitch, and
 * desired speed.
 * 4) Use an artillery table built from measured data to choose the shooter RPM
 * and hood angle commands.
 *
 * Important table reality:
 * - Ball output angle depends on BOTH hood angle AND shooter RPM.
 * - The artillery table here is therefore indexed by (RPM, HoodAngle) and
 * stores the measured outcome:
 * (BallOutputAngle, BallExitSpeed)
 * - The solver performs an inverse-lookup: given (desiredBallAngle,
 * desiredBallSpeed),
 * find the (RPM, HoodAngle) pair that best matches.
 *
 * Forbidden terms note:
 * - This file intentionally does not use certain mechanical terms.
 */
public final class TurretHelpers {

    // ---------------------------------------------------------------------------
    // Artillery table (what you MEASURE) indexed by (RPM, HoodAngle)
    // ---------------------------------------------------------------------------
    /** Measured outcome at a specific (RPM, HoodAngle). */
    public static class MeasuredBallOutcome {
        /** True ball output angle above horizontal (rad). */
        public final double ballOutputAngleRad;
        /** Ball exit speed relative to robot (m/s). */
        public final double ballExitSpeedMps;
        /** Optional: RPM drop observed when firing one ball at this setting (RPM). */
        public final double measuredRpmDrop;
        /** Optional: time to recover back within your RPM-ready tolerance (sec). */
        public final double measuredRecoveryTimeSec;

        public MeasuredBallOutcome(double ballOutputAngleRad, double ballExitSpeedMps) {
            this(ballOutputAngleRad, ballExitSpeedMps, Double.NaN, Double.NaN);
        }

        public MeasuredBallOutcome(double ballOutputAngleRad, double ballExitSpeedMps,
                double measuredRpmDrop, double measuredRecoveryTimeSec) {
            this.ballOutputAngleRad = ballOutputAngleRad;
            this.ballExitSpeedMps = ballExitSpeedMps;
            this.measuredRpmDrop = measuredRpmDrop;
            this.measuredRecoveryTimeSec = measuredRecoveryTimeSec;
        }
    }

    /**
     * A single command point (what you COMMAND to hardware) plus the measured
     * outcome.
     *
     * This is convenient because inverse-lookup returns commands, but we also want
     * to know
     * what measured outcome those commands represent (for debugging and scoring).
     */
    public static class ShooterCommandsAndMeasuredOutcome {
        public final double shooterRpmCommand;
        public final double hoodCommandAngleRad;
        public final double measuredBallOutputAngleRad;
        public final double measuredBallExitSpeedMps;

        public ShooterCommandsAndMeasuredOutcome(
                double shooterRpmCommand,
                double hoodCommandAngleRad,
                double measuredBallOutputAngleRad,
                double measuredBallExitSpeedMps) {
            this.shooterRpmCommand = shooterRpmCommand;
            this.hoodCommandAngleRad = hoodCommandAngleRad;
            this.measuredBallOutputAngleRad = measuredBallOutputAngleRad;
            this.measuredBallExitSpeedMps = measuredBallExitSpeedMps;
        }
    }

    /**
     * Artillery table indexed by:
     * RPM -> (HoodAngle -> MeasuredBallOutcome)
     *
     * This matches how you collect data:
     * 1) Pick an RPM
     * 2) Sweep hood angle in increments and record ball output angle + speed
     * 3) Increase RPM and repeat
     *
     * This class provides:
     * - adding measured points
     * - optional forward interpolation to estimate outcomes between measured points
     * - inverse lookup: choose (RPM, HoodAngle) commands that best match a desired
     * ball vector
     */
    public static class ArtilleryTableIndexedByShooterRpmAndHoodAngle {
        private final TreeMap<Double, TreeMap<Double, MeasuredBallOutcome>> table = new TreeMap<>();

        /** Add one measured sample: (rpm, hoodAngle) -> (ballAngle, ballSpeed). */
        public void addMeasuredSample(
                double shooterRpm,
                double hoodCommandAngleRad,
                double measuredBallOutputAngleRad,
                double measuredBallExitSpeedMps) {
            table.computeIfAbsent(shooterRpm, k -> new TreeMap<>())
                    .put(hoodCommandAngleRad,
                            new MeasuredBallOutcome(measuredBallOutputAngleRad, measuredBallExitSpeedMps));
        }

        public boolean hasAnyData() {
            return !table.isEmpty();
        }


/**
 * Load an artillery table from a CSV on the roboRIO.
 *
 * <p>Recommended location: put the file under src/main/deploy and load it with
 * {@link #loadFromDeployCsv(String)}.
 *
 * <h3>CSV format</h3>
 * <ul>
 *   <li>Header row is optional. Lines starting with '#' are ignored.</li>
 *   <li>Delimiter: comma</li>
 *   <li>Required columns (either order):
 *     <ul>
 *       <li>shooter_rpm (double)</li>
 *       <li>hood_deg (double) - hood command angle in degrees</li>
 *       <li>measured_ball_angle_deg (double) - measured exit angle above horizontal, degrees</li>
 *       <li>measured_ball_speed_mps (double)</li>
 *     </ul>
 *   </li>
 *   <li>Optional columns:
 *     <ul>
 *       <li>measured_rpm_drop (double) - RPM dip when firing one ball</li>
 *       <li>measured_recovery_time_sec (double) - time to recover to "ready"</li>
 *     </ul>
 *   </li>
 * </ul>
 *
 * <p>Units are explicit to avoid confusion. If you prefer radians in the file, convert before writing.
 *
 * <p>Error handling policy:
 * <ul>
 *   <li>Bad lines are skipped (and counted).</li>
 *   <li>If <b>no valid samples</b> are parsed, {@code hasAnyData()} will be false.</li>
 * </ul>
 */
public static ArtilleryTableIndexedByShooterRpmAndHoodAngle loadFromDeployCsv(String deployRelativePath) {
    Path file = Filesystem.getDeployDirectory().toPath().resolve(deployRelativePath);
    return loadFromCsv(file);
}

/** Same as {@link #loadFromDeployCsv(String)} but takes an absolute/relative {@link Path}. */
public static ArtilleryTableIndexedByShooterRpmAndHoodAngle loadFromCsv(Path csvPath) {
    ArtilleryTableIndexedByShooterRpmAndHoodAngle out = new ArtilleryTableIndexedByShooterRpmAndHoodAngle();
    if (csvPath == null) return out;
    if (!Files.exists(csvPath)) {
        return out;
    }

    int badLines = 0;
    int goodLines = 0;

    try (BufferedReader br = Files.newBufferedReader(csvPath, StandardCharsets.UTF_8)) {
        String line;
        while ((line = br.readLine()) != null) {
            line = line.trim();
            if (line.isEmpty() || line.startsWith("#")) continue;
            // Allow a header by skipping any line that contains non-numeric tokens in the first 2 columns.
            String[] parts = line.split(",");
            if (parts.length < 4) { badLines++; continue; }
            Double rpm = tryParse(parts[0]);
            Double hoodDeg = tryParse(parts[1]);
            Double ballAngDeg = tryParse(parts[2]);
            Double ballSpd = tryParse(parts[3]);
            if (rpm == null || hoodDeg == null || ballAngDeg == null || ballSpd == null) {
                // Header or malformed line
                badLines++;
                continue;
            }
            Double rpmDrop = (parts.length >= 5) ? tryParse(parts[4]) : null;
            Double rec = (parts.length >= 6) ? tryParse(parts[5]) : null;

            double hoodRad = Math.toRadians(hoodDeg);
            double ballAngRad = Math.toRadians(ballAngDeg);
            out.table.computeIfAbsent(rpm, k -> new TreeMap<>())
                .put(hoodRad, new MeasuredBallOutcome(ballAngRad, ballSpd,
                        (rpmDrop != null) ? rpmDrop : Double.NaN,
                        (rec != null) ? rec : Double.NaN));
            goodLines++;
        }
    } catch (IOException e) {
        // Leave table empty; caller can detect via hasAnyData()
        return out;
    }

    // Note: We intentionally do not throw if all lines were bad; empty table => invalid solution.
    return out;
}

private static Double tryParse(String s) {
    if (s == null) return null;
    s = s.trim();
    if (s.isEmpty()) return null;
    try {
        return Double.parseDouble(s);
    } catch (NumberFormatException ex) {
        return null;
    }
}

        /**
         * Inverse lookup:
         * Given a desired ball output angle (rad) and desired ball exit speed (m/s),
         * find the RPM + hood angle commands that best match.
         *
         * This returns a concrete object always.
         * If no data exists, it returns an object filled with NaN and valid=false will
         * be handled by caller.
         *
         * Matching rule:
         * error = angleWeight * |measuredAngle - desiredAngle| + speedWeight *
         * |measuredSpeed - desiredSpeed|
         *
         * Tie-breaking rule (when errors are extremely close):
         * - prefer lower RPM (usually easier shot / less stress)
         * - then prefer smaller hood angle change magnitude (caller can change this
         * later if needed)
         */
        public ShooterCommandsAndMeasuredOutcome findShooterRpmAndHoodAngleCommandsThatBestMatchDesiredBallAngleAndSpeed(
                double desiredBallOutputAngleRad,
                double desiredBallExitSpeedMps,
                double angleWeight,
                double speedWeight) {
            if (table.isEmpty())
                return makeNotARealCommandsAndOutcome();
            ShooterCommandsAndMeasuredOutcome best = null;
            double bestError = Double.POSITIVE_INFINITY;
            // Search all measured grid points. This is simple, robust, and fast enough for
            // typical table sizes.
            for (Map.Entry<Double, TreeMap<Double, MeasuredBallOutcome>> rpmRow : table.entrySet()) {
                double rpm = rpmRow.getKey();
                TreeMap<Double, MeasuredBallOutcome> hoodMap = rpmRow.getValue();
                if (hoodMap == null || hoodMap.isEmpty())
                    continue;
                for (Map.Entry<Double, MeasuredBallOutcome> hoodEntry : hoodMap.entrySet()) {
                    double hood = hoodEntry.getKey();
                    MeasuredBallOutcome out = hoodEntry.getValue();
                    if (out == null)
                        continue;
                    double angleErr = Math.abs(wrapToPi(out.ballOutputAngleRad - desiredBallOutputAngleRad));
                    double speedErr = Math.abs(out.ballExitSpeedMps - desiredBallExitSpeedMps);
                    double error = angleWeight * angleErr + speedWeight * speedErr;
                    if (error < bestError - 1e-12) {
                        bestError = error;
                        best = new ShooterCommandsAndMeasuredOutcome(rpm, hood, out.ballOutputAngleRad,
                                out.ballExitSpeedMps);
                    } else if (Math.abs(error - bestError) <= 1e-12 && best != null) {
                        // Tie-break: prefer lower RPM
                        if (rpm < best.shooterRpmCommand - 1e-9) {
                            best = new ShooterCommandsAndMeasuredOutcome(rpm, hood, out.ballOutputAngleRad,
                                    out.ballExitSpeedMps);
                        }
                    }
                }
            }
            return (best != null) ? best : makeNotARealCommandsAndOutcome();
        }

        /**
         * Optional forward interpolation:
         * Estimate (ballAngle, ballSpeed) for any (rpm, hood) by bilinear-like
         * interpolation.
         *
         * You do NOT need this to be correct, but it can smooth your table.
         * This returns NaNs if table is empty or missing rows.
         */
        public MeasuredBallOutcome estimateMeasuredBallOutcomeForShooterRpmAndHoodAngleUsingInterpolation(
                double shooterRpm,
                double hoodCommandAngleRad) {
            if (table.isEmpty())
                return new MeasuredBallOutcome(Double.NaN, Double.NaN);
            double rpmLow = floorKeyOrUseFirstKey(table, shooterRpm);
            double rpmHigh = ceilKeyOrUseLastKey(table, shooterRpm);
            if (nearlyEqual(rpmLow, rpmHigh)) {
                return interpolateOutcomeAcrossHoodAngleForSingleRpm(table.get(rpmLow), hoodCommandAngleRad);
            }
            MeasuredBallOutcome low = interpolateOutcomeAcrossHoodAngleForSingleRpm(table.get(rpmLow),
                    hoodCommandAngleRad);
            MeasuredBallOutcome high = interpolateOutcomeAcrossHoodAngleForSingleRpm(table.get(rpmHigh),
                    hoodCommandAngleRad);
            if (!isFiniteOutcome(low) || !isFiniteOutcome(high))
                return new MeasuredBallOutcome(Double.NaN, Double.NaN);
            double t = fraction(shooterRpm, rpmLow, rpmHigh);
            double ang = lerpAngle(low.ballOutputAngleRad, high.ballOutputAngleRad, t);
            double spd = lerp(low.ballExitSpeedMps, high.ballExitSpeedMps, t);
            return new MeasuredBallOutcome(ang, spd);
        }

        private static MeasuredBallOutcome interpolateOutcomeAcrossHoodAngleForSingleRpm(
                TreeMap<Double, MeasuredBallOutcome> hoodMap,
                double hoodCommandAngleRad) {
            if (hoodMap == null || hoodMap.isEmpty())
                return new MeasuredBallOutcome(Double.NaN, Double.NaN);
            double hoodLow = floorKeyOrUseFirstKey(hoodMap, hoodCommandAngleRad);
            double hoodHigh = ceilKeyOrUseLastKey(hoodMap, hoodCommandAngleRad);
            if (nearlyEqual(hoodLow, hoodHigh)) {
                MeasuredBallOutcome out = hoodMap.get(hoodLow);
                return (out != null) ? out : new MeasuredBallOutcome(Double.NaN, Double.NaN);
            }
            MeasuredBallOutcome a = hoodMap.get(hoodLow);
            MeasuredBallOutcome b = hoodMap.get(hoodHigh);
            if (a == null || b == null)
                return new MeasuredBallOutcome(Double.NaN, Double.NaN);
            double t = fraction(hoodCommandAngleRad, hoodLow, hoodHigh);
            double ang = lerpAngle(a.ballOutputAngleRad, b.ballOutputAngleRad, t);
            double spd = lerp(a.ballExitSpeedMps, b.ballExitSpeedMps, t);
            return new MeasuredBallOutcome(ang, spd);
        }
    }

    // ---------------------------------------------------------------------------
    // Solver output (similar style to your sim code)
    // ---------------------------------------------------------------------------
    /**
     * Solver output.
     *
     * valid:
     * - true if a feasible solution was found using table data
     *
     * timeOfFlightS:
     * - chosen time (sec)
     *
     * yawFieldRad:
     * - field yaw direction of the required ball exit-relative velocity (rad)
     *
     * desiredBallOutputAngleRad:
     * - pitch (rad) from the computed vector (this is what physics asked for)
     *
     * desiredBallExitSpeedMps:
     * - speed (m/s) from the computed vector (this is what physics asked for)
     *
     * ballExitVelocityRelativeToRobotExpressedInFieldFrame:
     * - the computed exit-relative velocity vector u, expressed in field
     * coordinates
     *
     * shooterRpmCommand / hoodCommandAngleRad:
     * - commands selected from the artillery table
     *
     * tableMatchedBallOutputAngleRad / tableMatchedBallExitSpeedMps:
     * - measured outcome at the chosen command point
     */
    public static class Solution {
        public final boolean valid;
        public final double timeOfFlightS;
        public final double yawFieldRad;
        public final double desiredBallOutputAngleRad;
        public final double desiredBallExitSpeedMps;
        public final Translation3d ballExitVelocityRelativeToRobotExpressedInFieldFrame;
        public final double shooterRpmCommand;
        public final double hoodCommandAngleRad;
        public final double tableMatchedBallOutputAngleRad;
        public final double tableMatchedBallExitSpeedMps;

        public Solution(
                boolean valid,
                double timeOfFlightS,
                double yawFieldRad,
                double desiredBallOutputAngleRad,
                double desiredBallExitSpeedMps,
                Translation3d ballExitVelocityRelativeToRobotExpressedInFieldFrame,
                double shooterRpmCommand,
                double hoodCommandAngleRad,
                double tableMatchedBallOutputAngleRad,
                double tableMatchedBallExitSpeedMps) {
            this.valid = valid;
            this.timeOfFlightS = timeOfFlightS;
            this.yawFieldRad = yawFieldRad;
            this.desiredBallOutputAngleRad = desiredBallOutputAngleRad;
            this.desiredBallExitSpeedMps = desiredBallExitSpeedMps;
            this.ballExitVelocityRelativeToRobotExpressedInFieldFrame = ballExitVelocityRelativeToRobotExpressedInFieldFrame;
            this.shooterRpmCommand = shooterRpmCommand;
            this.hoodCommandAngleRad = hoodCommandAngleRad;
            this.tableMatchedBallOutputAngleRad = tableMatchedBallOutputAngleRad;
            this.tableMatchedBallExitSpeedMps = tableMatchedBallExitSpeedMps;
        }

        /** Robot-relative turret yaw in degrees (0 = robot forward, CCW positive). */
        public double computeTurretYawAngleRelativeToRobotDeg(double robotYawRad) {
            double rel = yawFieldRad - robotYawRad;
            rel = Math.atan2(Math.sin(rel), Math.cos(rel));
            return Math.toDegrees(rel);
        }

        /**
         * Convenience list in the format you asked for: [ShooterRPM, HoodPosition,
         * BallVelocity, BallAngle]
         */
        public ArrayList<Double> toShooterRpmHoodBallSpeedBallAngleList() {
            ArrayList<Double> out = new ArrayList<>(4);
            out.add(shooterRpmCommand);
            out.add(hoodCommandAngleRad);
            out.add(desiredBallExitSpeedMps);
            out.add(desiredBallOutputAngleRad);
            return out;
        }
    }

    // ---------------------------------------------------------------------------
    // Public top-level method with descriptive name
    // ---------------------------------------------------------------------------
    /**
     * Compute shooter RPM + hood angle commands to hit the hub while the robot is
     * moving.
     *
     * This method:
     * - predicts robot pose and velocity at release time using velocity +
     * acceleration (no constant velocity assumption)
     * - tries candidate times-of-flight and computes the required ball
     * exit-relative velocity vector for each
     * - converts that vector to yaw/pitch/speed
     * - uses the artillery table inverse lookup to pick commands that best match
     * pitch/speed
     * - returns the best solution found
     *
     * Inputs:
     * robotPoseAtDecisionTimeField: pose at time t0
     * robotVelocityAtDecisionTimeFieldMps: velocity at time t0
     * robotAccelerationAtDecisionTimeFieldMps2: acceleration at time t0
     * robotYawRateAtDecisionTimeRadPerSec: yaw rate at time t0 (if unknown, pass 0)
     * feedDelayFromDecisionToReleaseSec: delay from t0 to release time
     * turretPivotOffsetFromRobotOriginRobotFrameMeters: turret pivot offset in
     * robot frame (x forward, y left)
     * ballReleaseHeightMeters: release height z0
     * hubTargetPositionFieldMeters: target position (xh,yh,zh)
     * artilleryTable: measured table indexed by (rpm, hood) -> (ballAngle,
     * ballSpeed)
     * timeOfFlightSearchMinSec / Max / Step: search parameters
     * gravityMetersPerSec2: typically 9.81
     * angleWeight / speedWeight: how strongly to prioritize matching angle vs speed
     * in inverse lookup
     */
    public static Solution solveForShooterRpmAndHoodAngleCommandsWhileRobotIsMovingUsingMeasuredTableIndexedByRpmAndHood(
            Pose2d robotPoseAtDecisionTimeField,
            Translation2d robotVelocityAtDecisionTimeFieldMps,
            Translation2d robotAccelerationAtDecisionTimeFieldMps2,
            double robotYawRateAtDecisionTimeRadPerSec,
            double feedDelayFromDecisionToReleaseSec,
            Translation2d turretPivotOffsetFromRobotOriginRobotFrameMeters,
            double ballReleaseHeightMeters,
            Translation3d hubTargetPositionFieldMeters,
            ArtilleryTableIndexedByShooterRpmAndHoodAngle artilleryTable,
            double timeOfFlightSearchMinSec,
            double timeOfFlightSearchMaxSec,
            double timeOfFlightSearchStepSec,
            double gravityMetersPerSec2,
            double angleWeight,
            double speedWeight) {
        if (artilleryTable == null || !artilleryTable.hasAnyData()) {
            return makeInvalidSolution();
        }
        PredictedRobotPoseAndVelocityAtReleaseTime predicted = predictRobotPoseAndVelocityAtReleaseTimeUsingVelocityAndAcceleration(
                robotPoseAtDecisionTimeField,
                robotVelocityAtDecisionTimeFieldMps,
                robotAccelerationAtDecisionTimeFieldMps2,
                robotYawRateAtDecisionTimeRadPerSec,
                feedDelayFromDecisionToReleaseSec);
        Translation2d turretPivotPositionFieldMeters = computeTurretPivotPositionInFieldFrameAtReleaseTimeFromPredictedRobotPose(
                predicted.predictedPoseField,
                turretPivotOffsetFromRobotOriginRobotFrameMeters);
        Translation3d startPositionFieldMeters = new Translation3d(turretPivotPositionFieldMeters.getX(),
                turretPivotPositionFieldMeters.getY(), ballReleaseHeightMeters);
        Translation3d robotVelocityFieldAtReleaseMps = new Translation3d(predicted.predictedVxFieldMps,
                predicted.predictedVyFieldMps, 0.0);
        Solution best = null;
        double bestCost = Double.POSITIVE_INFINITY;
        for (double T = timeOfFlightSearchMinSec; T <= timeOfFlightSearchMaxSec
                + 1e-9; T += timeOfFlightSearchStepSec) {
            if (T <= 1e-6)
                continue;
            Translation3d requiredBallExitVelocityRelativeToRobotFieldCoords = computeRequiredBallExitVelocityRelativeToRobotExpressedInFieldFrameForGivenTimeOfFlight(
                    startPositionFieldMeters,
                    hubTargetPositionFieldMeters,
                    robotVelocityFieldAtReleaseMps,
                    gravityMetersPerSec2,
                    T);
            YawPitchSpeed computedYawPitchSpeed = computeFieldYawAndBallOutputAngleAndBallSpeedFrom3dExitVelocityVector(
                    requiredBallExitVelocityRelativeToRobotFieldCoords);
            ShooterCommandsAndMeasuredOutcome commandsFromTable = artilleryTable
                    .findShooterRpmAndHoodAngleCommandsThatBestMatchDesiredBallAngleAndSpeed(
                            computedYawPitchSpeed.ballOutputAngleRad,
                            computedYawPitchSpeed.ballExitSpeedMps,
                            angleWeight,
                            speedWeight);
            if (!isFiniteCommandsAndOutcome(commandsFromTable))
                continue;
            // Define overall cost based on how well the chosen commands match what physics
            // asked for.
            double chosenAngleErr = Math.abs(
                    wrapToPi(commandsFromTable.measuredBallOutputAngleRad - computedYawPitchSpeed.ballOutputAngleRad));
            double chosenSpeedErr = Math
                    .abs(commandsFromTable.measuredBallExitSpeedMps - computedYawPitchSpeed.ballExitSpeedMps);
            double cost = angleWeight * chosenAngleErr + speedWeight * chosenSpeedErr;
            // Small bias toward lower speed (optional stability)
            cost += 0.02 * computedYawPitchSpeed.ballExitSpeedMps;
            if (cost < bestCost) {
                bestCost = cost;
                best = new Solution(
                        true,
                        T,
                        computedYawPitchSpeed.yawFieldRad,
                        computedYawPitchSpeed.ballOutputAngleRad,
                        computedYawPitchSpeed.ballExitSpeedMps,
                        requiredBallExitVelocityRelativeToRobotFieldCoords,
                        commandsFromTable.shooterRpmCommand,
                        commandsFromTable.hoodCommandAngleRad,
                        commandsFromTable.measuredBallOutputAngleRad,
                        commandsFromTable.measuredBallExitSpeedMps);
            }
        }
        return (best != null) ? best : makeInvalidSolution();
    }

    // ---------------------------------------------------------------------------
    // Helper methods (descriptive names)
    // ---------------------------------------------------------------------------
    private static class PredictedRobotPoseAndVelocityAtReleaseTime {
        public final Pose2d predictedPoseField;
        public final double predictedVxFieldMps;
        public final double predictedVyFieldMps;

        public PredictedRobotPoseAndVelocityAtReleaseTime(Pose2d predictedPoseField, double predictedVxFieldMps,
                double predictedVyFieldMps) {
            this.predictedPoseField = predictedPoseField;
            this.predictedVxFieldMps = predictedVxFieldMps;
            this.predictedVyFieldMps = predictedVyFieldMps;
        }
    }

    private static PredictedRobotPoseAndVelocityAtReleaseTime predictRobotPoseAndVelocityAtReleaseTimeUsingVelocityAndAcceleration(
            Pose2d robotPoseAtDecisionTimeField,
            Translation2d robotVelocityAtDecisionTimeFieldMps,
            Translation2d robotAccelerationAtDecisionTimeFieldMps2,
            double robotYawRateAtDecisionTimeRadPerSec,
            double delayFromDecisionToReleaseSec) {
        double dt = delayFromDecisionToReleaseSec;
        double predictedX = robotPoseAtDecisionTimeField.getX()
                + robotVelocityAtDecisionTimeFieldMps.getX() * dt
                + 0.5 * robotAccelerationAtDecisionTimeFieldMps2.getX() * dt * dt;
        double predictedY = robotPoseAtDecisionTimeField.getY()
                + robotVelocityAtDecisionTimeFieldMps.getY() * dt
                + 0.5 * robotAccelerationAtDecisionTimeFieldMps2.getY() * dt * dt;
        double predictedVx = robotVelocityAtDecisionTimeFieldMps.getX()
                + robotAccelerationAtDecisionTimeFieldMps2.getX() * dt;
        double predictedVy = robotVelocityAtDecisionTimeFieldMps.getY()
                + robotAccelerationAtDecisionTimeFieldMps2.getY() * dt;
        double predictedYawRad = robotPoseAtDecisionTimeField.getRotation().getRadians()
                + robotYawRateAtDecisionTimeRadPerSec * dt;
        Pose2d predictedPoseField = new Pose2d(
                predictedX,
                predictedY,
                new edu.wpi.first.math.geometry.Rotation2d(predictedYawRad));
        return new PredictedRobotPoseAndVelocityAtReleaseTime(predictedPoseField, predictedVx, predictedVy);
    }

    private static Translation2d computeTurretPivotPositionInFieldFrameAtReleaseTimeFromPredictedRobotPose(
            Pose2d predictedRobotPoseField,
            Translation2d turretPivotOffsetFromRobotOriginRobotFrameMeters) {
        double yaw = predictedRobotPoseField.getRotation().getRadians();
        double c = Math.cos(yaw);
        double s = Math.sin(yaw);
        double offsetXField = c * turretPivotOffsetFromRobotOriginRobotFrameMeters.getX()
                - s * turretPivotOffsetFromRobotOriginRobotFrameMeters.getY();
        double offsetYField = s * turretPivotOffsetFromRobotOriginRobotFrameMeters.getX()
                + c * turretPivotOffsetFromRobotOriginRobotFrameMeters.getY();
        return new Translation2d(
                predictedRobotPoseField.getX() + offsetXField,
                predictedRobotPoseField.getY() + offsetYField);
    }

    /**
     * Compute required ball exit-relative velocity (u) in FIELD coordinates for a
     * chosen time-of-flight T.
     *
     * Model:
     * pt = p0 + (vRobot + u)*T + 0.5*gVec*T^2
     * where gVec = (0,0,-g).
     *
     * Solve:
     * u = (pt - p0 - vRobot*T - 0.5*gVec*T^2)/T
     *
     * Note: because gVec is downward, the z term becomes "+ 0.5*g*T^2" in the
     * numerator.
     */
    private static Translation3d computeRequiredBallExitVelocityRelativeToRobotExpressedInFieldFrameForGivenTimeOfFlight(
            Translation3d startPositionFieldMeters,
            Translation3d targetPositionFieldMeters,
            Translation3d robotVelocityFieldAtReleaseMps,
            double gravityMetersPerSec2,
            double timeOfFlightSec) {
        double T = timeOfFlightSec;
        double g = gravityMetersPerSec2;
        double dx = targetPositionFieldMeters.getX() - startPositionFieldMeters.getX();
        double dy = targetPositionFieldMeters.getY() - startPositionFieldMeters.getY();
        double dz = targetPositionFieldMeters.getZ() - startPositionFieldMeters.getZ();
        double ux = (dx - robotVelocityFieldAtReleaseMps.getX() * T) / T;
        double uy = (dy - robotVelocityFieldAtReleaseMps.getY() * T) / T;
        double uz = (dz - robotVelocityFieldAtReleaseMps.getZ() * T + 0.5 * g * T * T) / T;
        return new Translation3d(ux, uy, uz);
    }

    private static class YawPitchSpeed {
        public final double yawFieldRad;
        public final double ballOutputAngleRad;
        public final double ballExitSpeedMps;

        public YawPitchSpeed(double yawFieldRad, double ballOutputAngleRad, double ballExitSpeedMps) {
            this.yawFieldRad = yawFieldRad;
            this.ballOutputAngleRad = ballOutputAngleRad;
            this.ballExitSpeedMps = ballExitSpeedMps;
        }
    }

    /**
     * Convert a 3D velocity vector into field yaw + ball output angle (pitch) +
     * speed.
     */
    private static YawPitchSpeed computeFieldYawAndBallOutputAngleAndBallSpeedFrom3dExitVelocityVector(
            Translation3d ballExitVelocityRelativeToRobotExpressedInFieldFrame) {
        double vx = ballExitVelocityRelativeToRobotExpressedInFieldFrame.getX();
        double vy = ballExitVelocityRelativeToRobotExpressedInFieldFrame.getY();
        double vz = ballExitVelocityRelativeToRobotExpressedInFieldFrame.getZ();
        double yawField = Math.atan2(vy, vx);
        double horizontalSpeed = Math.hypot(vx, vy);
        double ballAngle = Math.atan2(vz, horizontalSpeed);
        double speed = Math.sqrt(vx * vx + vy * vy + vz * vz);
        return new YawPitchSpeed(yawField, ballAngle, speed);
    }

    // ---------------------------------------------------------------------------
    // Validation / utility
    // ---------------------------------------------------------------------------
    private static boolean isFiniteOutcome(MeasuredBallOutcome o) {
        return o != null
                && Double.isFinite(o.ballOutputAngleRad)
                && Double.isFinite(o.ballExitSpeedMps);
    }

    private static boolean isFiniteCommandsAndOutcome(ShooterCommandsAndMeasuredOutcome c) {
        return c != null
                && Double.isFinite(c.shooterRpmCommand)
                && Double.isFinite(c.hoodCommandAngleRad)
                && Double.isFinite(c.measuredBallOutputAngleRad)
                && Double.isFinite(c.measuredBallExitSpeedMps);
    }

    private static ShooterCommandsAndMeasuredOutcome makeNotARealCommandsAndOutcome() {
        return new ShooterCommandsAndMeasuredOutcome(Double.NaN, Double.NaN, Double.NaN, Double.NaN);
    }

    private static Solution makeInvalidSolution() {
        return new Solution(
                false,
                Double.NaN,
                Double.NaN,
                Double.NaN,
                Double.NaN,
                new Translation3d(Double.NaN, Double.NaN, Double.NaN),
                Double.NaN,
                Double.NaN,
                Double.NaN,
                Double.NaN);
    }

    private static boolean nearlyEqual(double a, double b) {
        return Math.abs(a - b) < 1e-9;
    }

    /** Wrap angle to [-pi, pi]. */
    public static double wrapToPi(double rad) {
        return Math.atan2(Math.sin(rad), Math.cos(rad));
    }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    /** Interpolate angles safely by wrapping the delta. */
    private static double lerpAngle(double aRad, double bRad, double t) {
        double d = wrapToPi(bRad - aRad);
        return aRad + d * t;
    }

    private static double fraction(double x, double lo, double hi) {
        if (nearlyEqual(lo, hi))
            return 0.0;
        double t = (x - lo) / (hi - lo);
        if (t < 0.0)
            return 0.0;
        if (t > 1.0)
            return 1.0;
        return t;
    }

    private static <V> double floorKeyOrUseFirstKey(NavigableMap<Double, V> map, double key) {
        Double k = map.floorKey(key);
        return (k != null) ? k : map.firstKey();
    }

    private static <V> double ceilKeyOrUseLastKey(NavigableMap<Double, V> map, double key) {
        Double k = map.ceilingKey(key);
        return (k != null) ? k : map.lastKey();
    }
}
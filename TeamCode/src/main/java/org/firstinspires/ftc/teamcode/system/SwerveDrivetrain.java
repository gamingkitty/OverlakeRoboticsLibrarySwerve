package org.firstinspires.ftc.teamcode.system;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class SwerveDrivetrain {
    // Degrees/s
    private final double MAX_TURN_CORRECTION = 360;
    private final DifferentialSwerveModule[] swerveModules;
    private final OdometryModule odometry;
    private Pose2D currentPosition;
    private Pose2D wantedPosition;
    private boolean doPositionDrive;
    private double positionDriveVelocity;
    private PIDFController positionPIDF;

    public SwerveDrivetrain(
            DifferentialSwerveModule[] swerveModules,
            OdometryModule odometry
    ) {
        this.swerveModules = swerveModules;
        this.odometry = odometry;
        odometry.updatePosition();
        currentPosition = odometry.getPosition();

        positionPIDF = new PIDFController(1, 0, 0, 0);
    }

    public void setPositionPIDF(
            double p,
            double i,
            double d,
            double f
    ) {
        positionPIDF.setPIDF(p, i, d, f);
    }

    public void resetPIDF() {
        for (DifferentialSwerveModule swerveModule : swerveModules) {
            swerveModule.resetPIDF();
        }
        positionPIDF.reset();
    }

    public void drive() {
        for (DifferentialSwerveModule swerveModule : swerveModules) {
            swerveModule.drive();
        }

        if (doPositionDrive) {
            setPositionDrive();
        }
    }

    public void updatePosition() {
        odometry.updatePosition();
        currentPosition = odometry.getPosition();
    }

    private void setRobotVelocity(
            Pose2D robotVelocity
    ) {
        for (DifferentialSwerveModule swerveModule : swerveModules) {
            swerveModule.setRobotVelocity(robotVelocity);
        }
    }

    public void setVelocityDrive(
            Pose2D robotVelocity
    ) {
        setRobotVelocity(robotVelocity);
        doPositionDrive = false;
    }

    public void setPositionDrive(
            Pose2D wantedPosition
    ) {
        this.wantedPosition = wantedPosition;
        doPositionDrive = true;
        setPositionDrive();
    }

    private void setPositionDrive() {
        double dx = (wantedPosition.getX(DistanceUnit.INCH) - currentPosition.getX(DistanceUnit.INCH));
        double dy = (wantedPosition.getY(DistanceUnit.INCH) - currentPosition.getY(DistanceUnit.INCH));
        double direction = Math.atan2(dy, dx);
        double distance = Math.hypot(dx, dy);

        double velocity = positionDriveVelocity * positionPIDF.calculate(0, distance);

        // TODO: Test whether this is better or using a pure PID (This is simpler because less PID tuning)
        double turnVelocity = (velocity / Math.max(distance, 0.01)) * normalize(wantedPosition.getHeading(AngleUnit.DEGREES) - currentPosition.getHeading(AngleUnit.DEGREES));
        turnVelocity = Range.clip(turnVelocity, -MAX_TURN_CORRECTION, MAX_TURN_CORRECTION);

        Pose2D robotVelocity = new Pose2D(
                DistanceUnit.INCH,
                velocity * Math.cos(direction),
                velocity * Math.sin(direction),
                AngleUnit.DEGREES,
                turnVelocity
        );

        setRobotVelocity(robotVelocity);
    }

    public void setPositionDriveVelocity(
            double velocity
    ) {
        positionDriveVelocity = velocity;
    }

    private static double normalize(
            double degrees
    ) {
        double normalizedAngle = degrees;
        while (normalizedAngle > 180) normalizedAngle -= 360;
        while (normalizedAngle <= -180) normalizedAngle += 360;
        return normalizedAngle;
    }
}

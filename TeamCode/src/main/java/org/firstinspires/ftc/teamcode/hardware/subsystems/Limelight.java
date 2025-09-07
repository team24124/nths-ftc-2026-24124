package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.hardware.SubsystemBase;
import org.firstinspires.ftc.teamcode.util.telemetry.TelemetryObservable;

public class Limelight implements SubsystemBase, TelemetryObservable {
    private final Limelight3A limelight;
    public enum Pipeline {
        PSS1(0), // First python snapscript
        AT1(1), // First Apriltag pipeline
        AT2(2); // Second Apriltag pipeline

        public final int pipelineNum;

        Pipeline(int pipelineNum) {
            this.pipelineNum = pipelineNum;
        }
    }

    private Pipeline pipeline;

    private final double cameraAngle = 0; // From facing forward == 0 degrees

    public Limelight(HardwareMap hw){
        limelight = hw.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(Pipeline.AT1.pipelineNum);
        limelight.setPollRateHz(100); // Ask Limelight for data (100 times per second)
        limelight.start();
        pipeline = Pipeline.AT1;
    }

    // Switch pipeline if input is not the same as pipeline
    public void setPipeline(Pipeline pipeline) {
        if (pipeline != this.pipeline) {
            limelight.pipelineSwitch(pipeline.pipelineNum);
            this.pipeline = pipeline;
        }
    }

    // Checks python output & validity
    public boolean isDetected() {
        double[] array = getResult().getPythonOutput();
        boolean valid = getResult().isValid();

        return (array.length > 2 && array[0] == 1) || valid;
    }

    public LLResultTypes.FiducialResult getFiducial() {
        return getResult().getFiducialResults().get(getResult().getFiducialResults().size() - 1);
    }

    // AT type to determine Obelisk pattern (use for state machine within autonomous)
    public int ATType() {
        return getFiducial().getFiducialId();
    }

    private LLResult getResult() { return limelight.getLatestResult(); }

    //---------------------------------- values for PIDF ----------------------------------

    // Tx for rotation
    public double degreeOffset() {
        return getResult().getTx();
    }

    // Distance in inches
    public double distance() {
        return Math.tan(Math.toRadians(cameraAngle + getResult().getTyNC())) * 20;
    }

    //---------------------------------- target position returners ----------------------------------

    // Robot space X and Y offset for detected
    public Vector2d PSSTargetVectorRobotSpace() {
        double YOffset = Math.tan(Math.toRadians(cameraAngle + getResult().getTyNC())) * 20;
        double distanceFromCam = Math.sqrt(Math.pow(YOffset, 2) + Math.pow(20, 2));
        double XOffset = Math.tan(Math.toRadians(0 + getResult().getTxNC())) * distanceFromCam;
        return new Vector2d(XOffset - 13, YOffset - 19);
    }

    // Field space X and Y offset for detected
    public Pose2d PSSTargetPoseFieldSpace(Pose2d botPose) {
        double YOffset = Math.tan(Math.toRadians(cameraAngle + getResult().getTyNC())) * 20;
        double distanceFromCam = Math.sqrt(Math.pow(YOffset, 2) + Math.pow(20, 2));
        double XOffset = Math.tan(Math.toRadians(0 + getResult().getTxNC())) * distanceFromCam;

        double x1 = XOffset - 13;
        double y1 = YOffset - 19;

        double heading = botPose.heading.toDouble();

        double x = botPose.position.x + (x1 * Math.cos(heading) - y1 * Math.sin(heading));
        double y = botPose.position.y + (x1 * Math.sin(heading) + y1 * Math.cos(heading));

        return new Pose2d(x, y, heading);
    }

    // Robot space X and Y offset for apriltag
    public Vector2d ATTargetVectorRobotSpace() {
        setPipeline(Pipeline.AT1);
        Pose3D targetPose = getFiducial().getTargetPoseRobotSpace();
        return new Vector2d(targetPose.getPosition().x * 39.37 - 0, targetPose.getPosition().y * 39.37 - 60);
    }

    // Robot space X and Y offset and heading for apriltag
    public Pose2d ATTargetPoseRobotSpace() {
        setPipeline(Pipeline.AT1);
        Pose3D pose = getFiducial().getRobotPoseTargetSpace();
        double yaw;
        double rawYaw = pose.getOrientation().getYaw(AngleUnit.DEGREES);
        yaw = (rawYaw < 0) ? rawYaw + 360 : rawYaw; // Normalize heading

        return new Pose2d(-pose.getPosition().x * 39.37, pose.getPosition().z * 39.37 - 50, 360 - yaw); // Start trajectory at 0, 0, but heading has to be inputted heading. If robot turns other way, remove 360 -
    }

    // Field space bot pose (this is mapped, face desired AT)
    public Pose2d ATBotPoseFieldSpace() {
        setPipeline(Pipeline.AT2);
        Pose3D pose3d = getResult().getBotpose_MT2();
        return new Pose2d(pose3d.getPosition().x * 39.37, pose3d.getPosition().y * 39.37, pose3d.getOrientation().getYaw(AngleUnit.DEGREES) + 180);
    }

    //---------------------------------- telemetry ----------------------------------

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        LLResult result = getResult();
        telemetry.addData("Status", limelight.getStatus().toString());
        telemetry.addData("pipeline", pipeline.name());
        if (getResult() != null && result.getPythonOutput().length > 2) {
            double[] pythonOutputs = result.getPythonOutput();
            telemetry.addData("validity", pythonOutputs[0]);
            telemetry.addData("targetX", Math.round(pythonOutputs[1]));
            telemetry.addData("targetY", Math.round(pythonOutputs[2]));
            telemetry.addData("angle", Math.round(pythonOutputs[3]));
            telemetry.addData("area %", Math.round(pythonOutputs[4]));
        }
        telemetry.update();
    }

    @Override
    public String getName() {
        return "Limelight";
    }
}

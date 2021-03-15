package org.firstinspires.ftc.teamcode.drive.opmode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

@Autonomous(group = "drive")
public class AutonomousBlueWithOdo extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY =
            "AZLRyGv/////AAABmUBYMKfdK0Aomzp0Nt6VEkguSViSeAoA+D5usIvm/kqre4G73/hhstqirGs3OdwbEScv9C37DcPjYjTLiH/InhJiNOZ2sCHrUQ/pjXVvB+yUgmXFKdyUsiZNDNJgnyc7CnBFQ6FyrVRsLEv7vg17WNPdh7mgumtlb2LwD0N0m1D9ntZg90cLxto9GTfr+V6xS0k/NEsBPuljDPx2Hql3GAD39HZ3Ls62r306oUm9g+UVFNNUdKuvGc3xxtrTQAZHSzjufW7N232cZTPWyE508Bh8sKWUCJMB7ZuQ7TPZqS+VWPMdxUo9N//J5iHFoDidnjHb5Z6d2qKmqDk3/rIa5pdzc5bY+fhkr27+SegoN7WK";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private Servo shooterServo;
    private CRServo claw;
    private DcMotor clawLift;
    private DcMotor shooter;

    @Override
        public void runOpMode() {
        shooter = hardwareMap.get(DcMotor.class,"shooter");
        shooterServo = hardwareMap.get(Servo.class,"shooterServo");
        claw = hardwareMap.get(CRServo .class,"claw");
        clawLift = hardwareMap.get(DcMotor.class,"clawLift");
        clawLift.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        clawLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterServo.scaleRange(-1,1);
        shooterServo.setDirection(Servo.Direction.REVERSE);
        initVuforia();
        initTfod();
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            Pose2d startPose = new Pose2d(-36, -60, Math.toRadians(90));

            drive.setPoseEstimate(startPose);
            Trajectory moveToZoneQuad = drive.trajectoryBuilder(startPose)
                .addDisplacementMarker(() -> {
                    claw.setPower(-1);
                    clawLift.setPower(-1);
                })
                .splineTo(new Vector2d(-56, -60), Math.toRadians(90))
                .splineTo(new Vector2d(-56, 36), Math.toRadians(90))
                .build();


            waitForStart();
            if (isStopRequested()) return;

            if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2.5, 16.0/9.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        Boolean singleDetected = false;
        Boolean quadDetected = false;
        Boolean objectDetected = false;

        waitForStart();
        if (opModeIsActive()) {
            runtime.reset();
            while (objectDetected == false && runtime.seconds() < 3) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                            if (recognition.getLabel() == LABEL_FIRST_ELEMENT) {
                                quadDetected = true;
                                objectDetected = true;

                            }
                            if (recognition.getLabel() == LABEL_SECOND_ELEMENT) {
                                singleDetected = true;
                                objectDetected = true;

                            }
                        }
                        telemetry.update();
                    }
                }
            }
            if (objectDetected == false){
            }
            if (quadDetected == true) {
                drive.followTrajectory(moveToZoneQuad);
            }
            if (singleDetected == true) {
            }
        }
        if (tfod != null) {

            tfod.shutdown();
        }
        PoseStorage.currentPose = drive.getPoseEstimate();
    }

    public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }
    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
    }
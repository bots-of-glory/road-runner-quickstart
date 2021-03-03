package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous", group="Auto")
public class AutonomousBlue extends LinearOpMode {
    private ElapsedTime     runtime = new ElapsedTime();
    private DcMotor frontLeft;
    private DcMotor rearLeft;
    private DcMotor frontRight;
    private DcMotor rearRight;
    private Servo shooterServo;
    private CRServo claw;
    private DcMotor clawLift;
    private DcMotor shooter;

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AZLRyGv/////AAABmUBYMKfdK0Aomzp0Nt6VEkguSViSeAoA+D5usIvm/kqre4G73/hhstqirGs3OdwbEScv9C37DcPjYjTLiH/InhJiNOZ2sCHrUQ/pjXVvB+yUgmXFKdyUsiZNDNJgnyc7CnBFQ6FyrVRsLEv7vg17WNPdh7mgumtlb2LwD0N0m1D9ntZg90cLxto9GTfr+V6xS0k/NEsBPuljDPx2Hql3GAD39HZ3Ls62r306oUm9g+UVFNNUdKuvGc3xxtrTQAZHSzjufW7N232cZTPWyE508Bh8sKWUCJMB7ZuQ7TPZqS+VWPMdxUo9N//J5iHFoDidnjHb5Z6d2qKmqDk3/rIa5pdzc5bY+fhkr27+SegoN7WK";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        shooter = hardwareMap.dcMotor.get("shooter");
        shooterServo = hardwareMap.servo.get("shooterServo");
        claw = hardwareMap.crservo.get("claw");
        clawLift = hardwareMap.dcMotor.get("clawLift");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        rearLeft = hardwareMap.dcMotor.get("rearLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        rearRight = hardwareMap.dcMotor.get("rearRight");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setDirection(DcMotor.Direction.FORWARD);
        clawLift.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        clawLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
            // first.
            initVuforia();
            initTfod();

            /**
             * Activate TensorFlow Object Detection before we wait for the start command.
             * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
             **/
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
                while (objectDetected == false) {
                    if (tfod != null) {
                        // getUpdatedRecognitions() will return null if no new information is available since
                        // the last time that call was made.
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        if (updatedRecognitions != null) {
                            telemetry.addData("# Object Detected", updatedRecognitions.size());
                            // step through the list of recognitions and display boundary info.
                            int i = 0;
                            for (Recognition recognition : updatedRecognitions) {
                                objectDetected = true;
                                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                        recognition.getLeft(), recognition.getTop());
                                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                        recognition.getRight(), recognition.getBottom());
                                if (recognition.getLabel() == LABEL_FIRST_ELEMENT) {
                                    quadDetected = true;
                                }
                                if (recognition.getLabel() == LABEL_SECOND_ELEMENT) {
                                    singleDetected = true;
                                }
                            }
                            telemetry.update();
                        }
                    }
                }
                if (quadDetected == true) {
                    claw.setPower(-1);
                    frontLeft.setPower(-.5);
                    frontRight.setPower(.5);
                    rearLeft.setPower(.5);
                    rearRight.setPower(-.5);
                    sleep(800);
                    claw.setPower(-1);
                    frontLeft.setPower(1);
                    frontRight.setPower(1);
                    rearLeft.setPower(1);
                    rearRight.setPower(1);
                    sleep(600);
                    clawLift.setPower(.75);
                    claw.setPower(-1);
                    sleep(1000);
                    claw.setPower(1);
                    sleep(500);

                }
            }
            if (tfod != null) {
                tfod.shutdown();
            }





        }

        /**
         * Initialize the Vuforia localization engine.
         */
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

        /**
         * Initialize the TensorFlow Object Detection engine.
         */
        public void initTfod() {
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfodParameters.minResultConfidence = 0.8f;
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
        }
    }

        //Initialize claw
        //clawLift.setPower(1);
        //sleep(3000);
        //claw.setPower(-1);
        //sleep(1500);

        //move forward 8 feet
        /*rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setTargetPosition(1000);
        rearLeft.setTargetPosition(1000);
        frontRight.setTargetPosition(1000);
        rearRight.setTargetPosition(1000);*/

//move forward 8 feet

//spin up shooter
//flick servo
//strafe left 8 inches
//flick servo
//strafe left 8 inches
//flick servo
//move forward 8 inches
//stop
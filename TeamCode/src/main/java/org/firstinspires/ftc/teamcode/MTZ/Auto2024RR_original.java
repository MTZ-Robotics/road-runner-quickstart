package org.firstinspires.ftc.teamcode.MTZ;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "Auto2024RR", group = "Test")
public class Auto2024RR_original extends LinearOpMode {
    public class Lift {
        private DcMotorEx lift;

        public Lift(HardwareMap hardwareMap) {
            //todo: fix the name of the lift motor and uncomment the hardwareMap line
            //lift = hardwareMap.get(DcMotorEx.class, "arm");
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(0.8);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 3000.0) {
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(-0.8);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 100.0) {
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftDown(){
            return new LiftDown();
        }
    }

    public class Claw {
        private Servo claw;

        public Claw(HardwareMap hardwareMap) {
            //todo: fix the name of the claw servo and uncomment the hardwareMap line
            //claw = hardwareMap.get(Servo.class, "claw");
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.55);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(1.0);
                return false;
            }
        }
        public Action openClaw() {
            return new OpenClaw();
        }
    }

    int maxPathChoice = 3;
    int minPathChoice = 1;
    int pathToRun = 2;

    @Override
    public void runOpMode() {
        //   todo:  Fill in the start position
        Pose2d initialPose = new Pose2d(9, -66, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // TODO: uncomment the hardwareMap lines when using arm and claw

        //Claw claw = new Claw(hardwareMap);
        //Lift lift = new Lift(hardwareMap);


        mtzButtonBehavior pathChoiceDown = new mtzButtonBehavior();
        mtzButtonBehavior pathChoiceUp = new mtzButtonBehavior();


        // vision here that outputs position
        //
        int visionOutputPosition = 1;
        /******************************************************
         *
         *     PATHS    PATHS     PATHS     PATHS (Trajectories)
         *
         ******************************************************/
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                //Move to submersible
                .splineToSplineHeading(new Pose2d(6, -39, Math.toRadians(90)), Math.toRadians(90))
                //drop off specimen

                //code to drop specimen

                .waitSeconds(1)
                //back away form the submersible
                .strafeToConstantHeading(new Vector2d(6, -45))
                .turnTo(Math.toRadians(0))
                //Move around brace of submersible
                .strafeToConstantHeading(new Vector2d(34, -45))
                .turnTo(Math.toRadians(90))
                //.waitSeconds(1)
                //move to beyond sample 1
                .strafeToConstantHeading(new Vector2d(34, -10.5))
                .turnTo(Math.toRadians(0))
                .strafeToConstantHeading(new Vector2d(43, -10.5))
                //push sample 1
                .turnTo(Math.toRadians(100))
                .strafeToConstantHeading(new Vector2d(50, -55))
                //return
                .strafeToConstantHeading(new Vector2d(34, -10.5))
                //slide over behind sample 2
                .turnTo(Math.toRadians(0))
                .strafeToConstantHeading(new Vector2d(53, -10.5))
                //.splineToSplineHeading(new Pose2d(52, -10.5, Math.toRadians(90)), Math.toRadians(0))
                //push sample 2
                .turnTo(Math.toRadians(100))
                .strafeToConstantHeading(new Vector2d(53, -55))
                //.splineToSplineHeading(new Pose2d(50, -59, Math.toRadians(120)), Math.toRadians(-45))
                //return
                .strafeToConstantHeading(new Vector2d(34, -10.5))
                //slide over behind sample 3
                .turnTo(Math.toRadians(0))
                .strafeToConstantHeading(new Vector2d(55, -10.5))
                //push sample 3
                .turnTo(Math.toRadians(100))
                .strafeToConstantHeading(new Vector2d(55, -55))
                .waitSeconds(3);
        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                //Basket Side Delivery
                .splineToSplineHeading(new Pose2d(-6, -44.5, Math.toRadians(90)), Math.toRadians(90))
                //drop off specimen

                //code to drop specimen

                .waitSeconds(4)
                //back away form the submersible
                .lineToYSplineHeading(-50.5, Math.toRadians(90))
                //move to beyond sample 1
                .splineToSplineHeading(new Pose2d(-41, -20.5, Math.toRadians(90)), Math.toRadians(180))
                //push sample 1
                .splineToSplineHeading(new Pose2d(-50, -59, Math.toRadians(60)), Math.toRadians(45))
                //return
                .splineToSplineHeading(new Pose2d(-41, -20.5, Math.toRadians(90)), Math.toRadians(180))
                //slide over behind sample 2
                .splineToSplineHeading(new Pose2d(-52, -20.5, Math.toRadians(90)), Math.toRadians(90))
                //push sample 2
                .splineToSplineHeading(new Pose2d(-50, -59, Math.toRadians(60)), Math.toRadians(45))
                //return
                .splineToSplineHeading(new Pose2d(-41, -20.5, Math.toRadians(90)), Math.toRadians(180))
                //slide over behind sample 3
                .splineToSplineHeading(new Pose2d(-59, -20.5, Math.toRadians(90)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-61, -20.5, Math.toRadians(90)), Math.toRadians(1800))
                //push sample 3
                .splineToSplineHeading(new Pose2d(-50, -59, Math.toRadians(60)), Math.toRadians(45))
                .waitSeconds(3);
        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
                //Move to submersible
                .splineToSplineHeading(new Pose2d(6, -39, Math.toRadians(90)), Math.toRadians(90))
                //drop off specimen

                //code to drop specimen

                .waitSeconds(1)
                //back away form the submersible
                //Move around brace of submersible
                //.splineToConstantHeading(new Vector2d(34, -45),Math.toRadians(90))
                //move to beyond sample 1
                .splineToConstantHeading(new Vector2d(34, -10.5),Math.toRadians(90))
                //push sample 1
                .strafeToConstantHeading(new Vector2d(50, -55))
                //return
                .strafeToConstantHeading(new Vector2d(34, -10.5))
                //slide over behind sample 2
                .splineToConstantHeading(new Vector2d(53, -10.5),Math.toRadians(90))
                //push sample 2
                .strafeToConstantHeading(new Vector2d(53, -55))
                //return
                .strafeToConstantHeading(new Vector2d(34, -10.5))
                //slide over behind sample 3
                .splineToConstantHeading(new Vector2d(55, -10.5),Math.toRadians(90))
                //push sample 3
                .strafeToConstantHeading(new Vector2d(55, -55))
                .waitSeconds(3);
        Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
                //.strafeTo(new Vector2d(48, 12))
                .build();

        // actions that need to happen on init; for instance, a claw tightening.
        //Actions.runBlocking(claw.closeClaw());


        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Path to run: ", pathToRun);
            telemetry.update();
        }

        //int startPosition = visionOutputPosition;
        pathChoiceUp.update(gamepad1.dpad_up);
        pathChoiceUp.update(gamepad1.dpad_down);
        if(pathChoiceDown.clickedDown){
            pathToRun=pathToRun-1;
        }
        if(pathChoiceUp.clickedDown){
            pathToRun=pathToRun+1;
        }
        if(pathToRun>maxPathChoice){
            pathToRun=minPathChoice;
        }
        if(pathToRun<minPathChoice){
            pathToRun=maxPathChoice;
        }
        telemetry.addData("Path To Run: ", pathToRun);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen;
        if (pathToRun == 1) {
            trajectoryActionChosen = tab1.build();
        } else if (pathToRun == 2) {
            trajectoryActionChosen = tab2.build();
        } else {
            trajectoryActionChosen = tab3.build();
        }

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen,
                        /* lift.liftUp(),
                        claw.openClaw(),
                        lift.liftDown(),*/
                        trajectoryActionCloseOut
                )
        );
    }
}

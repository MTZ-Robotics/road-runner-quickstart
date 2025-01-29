package org.firstinspires.ftc.teamcode.MTZ;

import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.leftClawClosedPosition;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.leftClawOpenPosition;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.rightClawClosedPosition;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.rightClawOpenPosition;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.ticksPerDegreeArm;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.ticksPerInchExtension;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
@Disabled
@Config
@Autonomous(name = "Auto2024RrTest_24Jan2025", group = "Test")
public class Auto2024RrTest_24Jan2025 extends LinearOpMode {
    public static int ticksToTopRail = (int) ((int) 70.0*ticksPerDegreeArm);
    public static int ticksToHumanPlayer = (int) ((int) 20.0*ticksPerDegreeArm);
    public static int ticksToEjectArm = (int) ((int) 65*ticksPerDegreeArm);
    public static int ticksToRail = (int) ((int) 3.0 * ticksPerInchExtension);
    public static int ticksToEject = (int) ((int) ticksToRail - (2.0 * ticksPerInchExtension));
    public static double startX=9.5;
    public static double startY=-66;
    public static double deliverRailX=6;
    public static double deliverRailY=-34;
    public static double avoidSubX=37;
    public static double avoidSubY=-40;
    public static double sampleY=-18;
    public static double sample1X=47;
    public static double sample2X=53;
    public static double sample3X=63;
    public static double humanPlayerY=-57;
    public static double humanPlayerX=50;


    public class Arm {
        private DcMotorEx arm;

        public Arm(HardwareMap hardwareMap) {
            //todo: fix the name of the arm motor and uncomment the hardwareMap line
            arm = hardwareMap.get(DcMotorEx.class, "arm");
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            arm.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class ArmToRail implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    arm.setPower(0.8);
                    initialized = true;
                }

                double pos = arm.getCurrentPosition();
                packet.put("armPos", pos);
                if (pos < ticksToTopRail) {
                    return true;
                } else {
                    arm.setPower(0);
                    return false;
                }
            }

        }

        /*
        public class ArmToRail implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    arm.setTargetPosition(ticksToTopRail);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setPower(0.8);
                    initialized = true;
                }

                double pos = arm.getCurrentPosition();
                packet.put("armPos", pos);
                if (arm.isBusy()) {
                    return true;
                } else {
                    arm.setPower(0);
                    return false;
                }
            }

        }

         */
        public class ArmToHuman implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    arm.setTargetPosition(ticksToHumanPlayer);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setPower(0.8);
                    initialized = true;
                }

                double pos = arm.getCurrentPosition();
                packet.put("armPos", pos);
                if (arm.isBusy()) {
                    return true;
                } else {
                    arm.setPower(0);
                    return false;
                }
            }

        }
        public class ArmToEject implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    arm.setTargetPosition(ticksToEjectArm);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setPower(0.8);
                    initialized = true;
                }

                double pos = arm.getCurrentPosition();
                packet.put("armPos", pos);
                if (arm.isBusy()) {
                    return true;
                } else {
                    arm.setPower(0);
                    return false;
                }
            }

        }
        public Action armToRail() {
            return new ArmToRail();
        }
        public Action armToHuman() {
            return new ArmToHuman();
        }
        public Action armToEject() {
            return new ArmToEject();
        }

        public class ArmDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    arm.setTargetPosition(0);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setPower(0.8);
                    initialized = true;
                }

                double pos = arm.getCurrentPosition();
                packet.put("armPos", pos);
                if (arm.isBusy()) {
                    return true;
                } else {
                    arm.setPower(0);
                    return false;
                }
            }
        }
        public Action armDown(){
            return new ArmDown();
        }
    }

    public class Extend {
        private DcMotorEx extend;

        public Extend(HardwareMap hardwareMap) {
            //todo: fix the name of the extend motor and uncomment the hardwareMap line
            extend = hardwareMap.get(DcMotorEx.class, "armExtension");
            extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            extend.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class ExtendToRail implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    extend.setTargetPosition(ticksToRail);
                    extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    extend.setPower(0.8);
                    initialized = true;
                }

                double pos = extend.getCurrentPosition();
                packet.put("ejectPos", pos);
                if (extend.isBusy()) {
                    return true;
                } else {
                    extend.setPower(0);
                    return false;
                }
            }

        }

        public class ExtendToEject implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    extend.setTargetPosition(ticksToEject);
                    extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    extend.setPower(0.8);
                    initialized = true;
                }

                double pos = extend.getCurrentPosition();
                packet.put("ejectPos", pos);
                if (extend.isBusy()) {
                    return true;
                } else {
                    extend.setPower(0);
                    return false;
                }
            }

        }

        public class ExtendRetract implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    extend.setTargetPosition(0);
                    extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    extend.setPower(0.8);
                    initialized = true;
                }

                double pos = extend.getCurrentPosition();
                packet.put("ejectPos", pos);
                if (extend.isBusy()) {
                    return true;
                } else {
                    extend.setPower(0);
                    return false;
                }
            }
        }
        public Action extendToRail() {
            return new ExtendToRail();
        }
        public Action extendToEject() {
            return new ExtendToEject();
        }
        public Action extendRetract() {
            return new ExtendRetract();
        }
    }


    public class Claw {
        private Servo leftClaw;
        private Servo rightClaw;

        public Claw(HardwareMap hardwareMap) {
            //todo: fix the name of the claw servo and uncomment the hardwareMap line
            leftClaw = hardwareMap.get(Servo.class, "leftClaw");
            rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftClaw.setPosition(leftClawClosedPosition);
                rightClaw.setPosition(rightClawClosedPosition);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftClaw.setPosition(leftClawOpenPosition);
                rightClaw.setPosition(rightClawOpenPosition);
                return false;
            }
        }
        public Action openClaw() {
            return new OpenClaw();
        }
    }
    int defaultPathToRun = 1;
    int minPathChoice = 1;
    int maxPathChoice = 3;

    @Override
    public void runOpMode() {
        autonomousPathRunner(defaultPathToRun);
    }
    public void autonomousPathRunner(int pathToRun) {
        //   todo:  Fill in the start position for each path

        Pose2d initialPose;
        //Pose2d deliverPose;

        if(pathToRun==1) {
            initialPose = new Pose2d(9, -66, Math.toRadians(90));
        } else if (pathToRun==2){
            initialPose = new Pose2d(-9, -66, Math.toRadians(90));
        } else {
            initialPose = new Pose2d(9, -66, Math.toRadians(90));
        }
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // TODO: uncomment the hardwareMap lines when using arm and claw

        Claw claw = new Claw(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Extend extend = new Extend(hardwareMap);


        mtzButtonBehavior pathChoiceDown = new mtzButtonBehavior();
        mtzButtonBehavior pathChoiceUp = new mtzButtonBehavior();


        // vision here that outputs position
        //
        int visionOutputPosition = 1;
        TrajectoryActionBuilder tab1;
        TrajectoryActionBuilder tab2 = null;
        TrajectoryActionBuilder tab3 = null;
        //TrajectoryActionBuilder tabDeliver;

        /******************************************************
         *
         *     PATHS    PATHS     PATHS     PATHS (Trajectories)
         *
         ******************************************************/



        /************************************
         *
         * 11111111111111111111111111111111
         *
         * Push Bot Turning Specimen Side
         *
         * 11111111111111111111111111111111
         *
         ************************************/

        int neg=1; //This variable can be set to negative 1 to reverse the directions and distances

        if (pathToRun==1) {
            neg=1;

            initialPose = new Pose2d(neg*startX, startY, Math.toRadians(90));

            tab1 = drive.actionBuilder(initialPose)
                    //Move to submersible
                    .waitSeconds(1)
                    .splineToSplineHeading(new Pose2d(neg*deliverRailX, deliverRailY, Math.toRadians(90)), Math.toRadians(90));
                    //drop off specimen


            tab2 = tab1.endTrajectory().fresh()
                    //.waitSeconds(1)
                    //back away form the submersible
                    //Move around brace of submersible
                    //.splineToConstantHeading(new Vector2d(11.7, -37.2),Math.toRadians(90))
                    .strafeToConstantHeading(new Vector2d(neg*(deliverRailX+6), avoidSubY))
                    //avoid submersible
                    .strafeTo(new Vector2d(neg*avoidSubX, avoidSubY))
                    .turnTo(Math.toRadians(270))
                    //move to beyond sample 1
                    .strafeToConstantHeading(new Vector2d(neg*(sample1X-10), sampleY))
                    //line up with sample 1
                    .strafeToConstantHeading(new Vector2d(neg*sample1X, sampleY))
                    //push sample 1
                    .strafeToConstantHeading(new Vector2d(neg*sample1X, humanPlayerY))
                    //return to samples
                    .strafeToConstantHeading(new Vector2d(neg*(sample2X-10), sampleY))
                    //line up with sample 2
                    .strafeToConstantHeading(new Vector2d(neg*sample2X, sampleY))
                    //push sample 2
                    .splineToConstantHeading(new Vector2d(neg*sample2X, humanPlayerY), Math.toRadians(90))

                    .splineTo(new Vector2d(50, -57), Math.toRadians(180))
                    //return to samples
                    .strafeToConstantHeading(new Vector2d(neg*(sample3X-10), sampleY))
                    //line up with sample 3
                    .strafeToConstantHeading(new Vector2d(neg*sample3X, sampleY))
                    //push sample 3
                    .strafeToConstantHeading(new Vector2d(neg*sample3X, humanPlayerY));
                    //.waitSeconds(3);

            tab3 = tab2.endTrajectory().fresh()
                    //turn to grab block
                    .turnTo(Math.toRadians(270))
                    //go to submersible
                    .strafeTo(new Vector2d(6, -36))
                    //go park
                    .strafeTo(new Vector2d(50, -65));

        }





        /*********************
         *
         * 2222222222222222222222222
         *
         * Push Bot Spline Net Side
         *
         * 222222222222222222222222
         *
         *********************/




        else if (pathToRun==2) {
            initialPose = new Pose2d(-9, -66, Math.toRadians(90));
            tab1 = drive.actionBuilder(initialPose)


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

        }
        /*********************
         *
         * Any Other Number Path
         *
         * Push Bot Spline Specimen Side
         *
         *********************/
        else {
            initialPose = new Pose2d(9, -66, Math.toRadians(90));
            tab1 = drive.actionBuilder(initialPose)
                    //Move to submersible
                    .splineToSplineHeading(new Pose2d(6, -39, Math.toRadians(90)), Math.toRadians(90))
                    //drop off specimen

                    //code to drop specimen

                    .waitSeconds(1)
                    //back away form the submersible
                    //Move around brace of submersible
                    //.splineToConstantHeading(new Vector2d(34, -45),Math.toRadians(90))
                    //move to beyond sample 1
                    .splineToConstantHeading(new Vector2d(34, -10.5), Math.toRadians(90))
                    //push sample 1
                    .strafeToConstantHeading(new Vector2d(50, -55))
                    //return
                    .strafeToConstantHeading(new Vector2d(34, -10.5))
                    //slide over behind sample 2
                    .splineToConstantHeading(new Vector2d(53, -10.5), Math.toRadians(90))
                    //push sample 2
                    .strafeToConstantHeading(new Vector2d(53, -55))
                    //return
                    .strafeToConstantHeading(new Vector2d(34, -10.5))
                    //slide over behind sample 3
                    .splineToConstantHeading(new Vector2d(55, -10.5), Math.toRadians(90))
                    //push sample 3
                    .strafeToConstantHeading(new Vector2d(55, -55))
                    .waitSeconds(3);
        }
        Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
                //.strafeTo(new Vector2d(48, 12))
                .build();

        /************************
         *
         *    Initialize Op Mode
         *
         ************************/
        // actions that need to happen on init; for instance, a claw tightening.
        Actions.runBlocking(claw.closeClaw());


        while (!isStopRequested() && !opModeIsActive()) {
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
            telemetry.addData("Path to run: ", pathToRun);
            telemetry.update();
        }


        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryAction1;
        Action trajectoryAction2;
        Action trajectoryAction3;


        Action squeezeClaw = new SequentialAction(
                claw.closeClaw(),
                new SleepAction(1) //wait for claw to move
        );
        Action secureSpecimen = new SequentialAction(
                extend.extendToEject(), //pull down
                arm.armToEject(), //pull down
                claw.openClaw(),//let go
                new SleepAction(1) //wait for claw to move
        );


        trajectoryAction1 = tab1.build();

        assert tab2 != null;
        trajectoryAction2 = tab2.build();
        trajectoryAction3 = tab3.build();

        Actions.runBlocking(
                new SequentialAction(
                        squeezeClaw, //Grab Specimen
                        new ParallelAction( //Move to Submersible
                                arm.armToRail(),
                                extend.extendToRail(),
                                trajectoryAction1
                        ),
                        secureSpecimen,
                        new ParallelAction( //Move to human player
                                trajectoryAction2,
                                extend.extendToRail(),
                                arm.armToHuman()
                        ),
                        squeezeClaw,
                        new ParallelAction( //Move to Submersible
                                trajectoryAction3,
                                arm.armToRail()
                        ),
                        secureSpecimen,
                        new ParallelAction(
                                arm.armDown(),
                                extend.extendRetract(),
                                trajectoryActionCloseOut
                        )
                )
        );
    }
}

package org.firstinspires.ftc.teamcode.MTZ;

import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.defaultArmExtensionPower;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.defaultArmPower;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.earlyDelayPauseTime;
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
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "Auto2024Rr", group = "Test")
public class Auto2024Rr extends LinearOpMode {

    public static double degreesToTopRail = 70.0;
    public static double degreesToPlaceSpecimen = 60.0;
    public static double degreesToHumanPlayer = 70.0;
    public static double degreesToEjectArm = 65.0;
    public static double degreesToTheSkyArm = 98.0;
    public static double inchesToTopRailExtend = 3.0;
    public static double inchesToEjectExtend = 1.0;
    public static int ticksToTopRail = (int) ((int) degreesToTopRail*ticksPerDegreeArm);
    public static int ticksToHumanPlayer = (int) ((int) degreesToHumanPlayer*ticksPerDegreeArm);
    public static int ticksToEjectArm = (int) ((int) degreesToEjectArm*ticksPerDegreeArm);
    public static int ticksToTheSkyArm = (int) ((int) degreesToTheSkyArm*ticksPerDegreeArm);
    public static int ticksToRailExtend = (int) ((int) inchesToTopRailExtend * ticksPerInchExtension);
    public static int ticksToEjectExtend = (int) ((int) inchesToEjectExtend * ticksPerInchExtension);
    public static double startX=9.5;
    public static double startY=-66;
    public static double deliverRailX=3;
    public static double deliverRailY=-38;
    public static double avoidSubX=37;
    public static double avoidSubY=-40;
    public static double sampleY=-18;
    public static double sample1X=47;
    public static double sample2X=60;
    public static double sample3X=63;
    public static double humanPlayerY=-57;
    public static double humanPlayerX=50;
    public static double parkX=50;
    public static double parkY=-65;
    public static int defaultPathToRun = 1;
    public static int version = 1;

    public class Documentation {

        public Documentation(HardwareMap hardwareMap) {
        }
        public class ReportVersion implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    packet.put("Version: ", version);
                    initialized = true;
                }
                return true;
            }
        }
        public Action reportVersion() {return new ReportVersion();}
    }
    public class ArmClass {
        private DcMotorEx armMotor;

        public ArmClass(HardwareMap hardwareMap) {
            //todo: fix the name of the arm motor and uncomment the hardwareMap line
            armMotor = hardwareMap.get(DcMotorEx.class, "arm");
            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        public class ArmToRail implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    armMotor.setTargetPosition(ticksToTopRail);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(defaultArmPower/2);
                    initialized = true;
                }

                double pos = armMotor.getCurrentPosition();
                packet.put("armPos", pos);
                if (armMotor.isBusy()) {
                    return true;
                } else {
                    armMotor.setPower(0);
                    return false;
                }
            }

        }
        public class ArmReset implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    initialized = true;
                }
                double pos = armMotor.getCurrentPosition();
                packet.put("armPos", pos);
                if (armMotor.isBusy()) {
                    return true;
                } else {
                    armMotor.setPower(0);
                    return false;
                }
            }
        }

        public class ArmToHuman implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    armMotor.setTargetPosition(ticksToHumanPlayer);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(defaultArmPower);
                    initialized = true;
                }
                double pos = armMotor.getCurrentPosition();
                packet.put("armPos", pos);
                if (armMotor.isBusy()) {
                    return true;
                } else {
                    armMotor.setPower(0);
                    return false;
                }
            }

        }
        public class ArmToEject implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    armMotor.setTargetPosition(ticksToEjectArm);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(defaultArmPower);
                    initialized = true;
                }
                double pos = armMotor.getCurrentPosition();
                packet.put("armPos", pos);
                if (armMotor.isBusy()) {
                    return true;
                } else {
                    armMotor.setPower(0);
                    return false;
                }
            }

        }
        public class ArmToTheSky implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    armMotor.setTargetPosition(ticksToTheSkyArm);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(defaultArmPower);
                    initialized = true;
                }
                double pos = armMotor.getCurrentPosition();
                packet.put("armPos", pos);
                if (armMotor.isBusy()) {
                    return true;
                } else {
                    armMotor.setPower(0);
                    return false;
                }
            }

        }
        public class ArmDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    armMotor.setTargetPosition(0);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(defaultArmPower);
                    initialized = true;
                }

                double pos = armMotor.getCurrentPosition();
                packet.put("armPos", pos);
                if (armMotor.isBusy()) {
                    return true;
                } else {
                    armMotor.setPower(0);
                    return false;
                }
            }
        }

        public Action armReset() {return new ArmReset();}
        public Action armToRail() {return new ArmToRail();}
        public Action armToHuman() {return new ArmToHuman();}
        public Action armToEject() {return new ArmToEject();}
        public Action armToTheSky() {return new ArmToTheSky();}
        public Action armDown(){return new ArmDown();}
    }

    public static class ExtendClass {
        private DcMotorEx extendMotor;
        public ExtendClass(HardwareMap hardwareMap) {
            //todo: fix the name of the extend motor and uncomment the hardwareMap line
            extendMotor = hardwareMap.get(DcMotorEx.class, "armExtension");
            extendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            extendMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        public class ExtendToRail implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    extendMotor.setTargetPosition(ticksToRailExtend);
                    extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    extendMotor.setPower(defaultArmExtensionPower);
                    initialized = true;
                }

                double pos = extendMotor.getCurrentPosition();
                packet.put("extendPos", pos);
                if (extendMotor.isBusy()) {
                    return true;
                } else {
                    extendMotor.setPower(0);
                    return false;
                }
            }

        }
        public class ExtendReset implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    extendMotor.setTargetPosition(0);
                    extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    extendMotor.setPower(defaultArmExtensionPower);
                    initialized = true;
                }

                double pos = extendMotor.getCurrentPosition();
                packet.put("extendPos", pos);
                if (extendMotor.isBusy()) {
                    return true;
                } else {
                    extendMotor.setPower(0);
                    return false;
                }
            }

        }
        public class ExtendToEject implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    extendMotor.setTargetPosition(ticksToEjectExtend);
                    extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    extendMotor.setPower(defaultArmExtensionPower);
                    initialized = true;
                }

                double pos = extendMotor.getCurrentPosition();
                packet.put("extendPos", pos);
                if (extendMotor.isBusy()) {
                    return true;
                } else {
                    extendMotor.setPower(0);
                    return false;
                }
            }

        }
        public class ExtendRetract implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    extendMotor.setTargetPosition(0);
                    extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    extendMotor.setPower(defaultArmExtensionPower);
                    initialized = true;
                }

                double pos = extendMotor.getCurrentPosition();
                packet.put("extendPos", pos);
                if (extendMotor.isBusy()) {
                    return true;
                } else {
                    extendMotor.setPower(0);
                    return false;
                }
            }
        }
        public Action extendReset() {return new ExtendReset();}
        public Action extendToRail() {return new ExtendToRail();}
        public Action extendToEject() {return new ExtendToEject();}
        public Action extendRetract() {return new ExtendRetract();}
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
        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftClaw.setPosition(leftClawOpenPosition);
                rightClaw.setPosition(rightClawOpenPosition);
                return false;
            }
        }
        public Action openClaw() {return new OpenClaw();}
        public Action closeClaw() {return new CloseClaw();}
    }
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
            initialPose = new Pose2d(startX, startY, Math.toRadians(90));
        } else if (pathToRun==2){
            initialPose = new Pose2d(-startX, startY, Math.toRadians(90));
        } else {
            initialPose = new Pose2d(startX, startY, Math.toRadians(90));
        }
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // TODO: uncomment the hardwareMap lines when using arm and claw

        Claw claw = new Claw(hardwareMap);
        ArmClass armRotate = new ArmClass(hardwareMap);
        ExtendClass armExtend = new ExtendClass(hardwareMap);
        Documentation documentation = new Documentation(hardwareMap);


        mtzButtonBehavior pathChoiceDown = new mtzButtonBehavior();
        mtzButtonBehavior pathChoiceUp = new mtzButtonBehavior();


        // vision here that outputs position
        //
        int visionOutputPosition = 1;
        TrajectoryActionBuilder tab1;
        TrajectoryActionBuilder tab2 = null;
        TrajectoryActionBuilder tab3 = null;
        TrajectoryActionBuilder tabClose = null;

        /******************************************************
         *
         *     PATHS    PATHS     PATHS     PATHS (Trajectories)
         *
         ******************************************************/



        /************************************
         *
         * 11111111111111111111111111111111
         *
         * Push Bot Specimen Side
         *
         * 11111111111111111111111111111111
         *
         ************************************/

        int neg=1; //This variable can be set to negative 1 to reverse the directions and distances

        if (pathToRun==1 || pathToRun==2) {
            if (pathToRun == 2) {
                neg=-1;
            }

            initialPose = new Pose2d(neg*startX, startY, Math.toRadians(90));

            tab1 = drive.actionBuilder(initialPose)
                    //Move to submersible
                    //.splineToSplineHeading(new Pose2d(neg*deliverRailX, deliverRailY, Math.toRadians(90)), Math.toRadians(90))
                    .splineToSplineHeading(new Pose2d(neg*deliverRailX, deliverRailY, Math.toRadians(90)), Math.toRadians(90));
                    //.strafeTo(new Vector2d(neg*deliverRailX, deliverRailY));
                    //drop off specimen

            tab2 = tab1.endTrajectory().fresh()
                    //.waitSeconds(1)
                    //back away form the submersible
                    //Move around brace of submersible
                    //.splineToConstantHeading(new Vector2d(11.7, -37.2),Math.toRadians(90))
                    .strafeToConstantHeading(new Vector2d(neg*(deliverRailX+4), avoidSubY))
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
                    .strafeToConstantHeading(new Vector2d(neg*sample2X, humanPlayerY))
                            //.strafeToConstantHeading(new Vector2d(neg*sample2X, humanPlayerY))
                    .splineTo(new Vector2d(50, -57), Math.toRadians(180));


            tab3 = tab2.endTrajectory().fresh()
                    //go to submersible
                    .strafeTo(new Vector2d(neg*(deliverRailX-4), deliverRailY));
                    //.splineTo(new Vector2d(neg*(deliverRailX-4), deliverRailY), Math.toRadians(90));
                    //go park now in Closeout

            tabClose = tab3.endTrajectory().fresh()
                    //.strafeTo(new Vector2d(48, 12))
                    .strafeTo(new Vector2d(neg*parkX, parkY));

        }



        /*********************
         *
         * Any Other Number Path
         *
         * Wait and deliver 2 Specimen Side
         *
         *********************/
        else {
            initialPose = new Pose2d(9, -66, Math.toRadians(90));
            tab1 = drive.actionBuilder(initialPose)
                    .waitSeconds(earlyDelayPauseTime/1000)
                    .splineToSplineHeading(new Pose2d(neg*deliverRailX, deliverRailY, Math.toRadians(90)), Math.toRadians(90));//Move to submersible
            tab2 = tab1.endTrajectory().fresh()
                    .strafeToConstantHeading(new Vector2d(neg*(deliverRailX+4), avoidSubY))//Back Away
                    .splineToConstantHeading(new Vector2d(neg*humanPlayerX, humanPlayerY), Math.toRadians(270)); //Go To Human Player
            tab3 = tab2.endTrajectory().fresh()
                    //go to submersible
                    .strafeTo(new Vector2d(neg*(deliverRailX-4), deliverRailY));

            tabClose = tab3.endTrajectory().fresh()
                    .strafeTo(new Vector2d(neg*parkX, parkY));
        }


        /************************
         *
         *    Initialize Op Mode
         *
         ************************/
        // actions that need to happen on init; for instance, a claw tightening.
        Actions.runBlocking(
                new SequentialAction(
                        //documentation.reportVersion(),
                        armRotate.armReset(),
                        //armExtend.extendReset(),
                        claw.closeClaw()
                        //new SleepAction(1), //wait for claw to move



                        //claw.openClaw(),
                        //new SleepAction(1), //wait for claw to move
                        //claw.closeClaw()
                )
        );



        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryAction1;
        Action trajectoryAction2;
        Action trajectoryAction3;
        Action trajectoryActionCloseOut;


        Action squeezeClaw = new SequentialAction(
                claw.closeClaw(),
                new SleepAction(1) //wait for claw to move
        );
        Action clipSpecimen = new SequentialAction(
                armExtend.extendToEject(), //pull down
                //armRotate.armToEject(), //pull down
                armRotate.armToHuman(),
                claw.openClaw(),//let go
                new SleepAction(1) //wait for claw to move
        );

        trajectoryAction1 = tab1.build();
        trajectoryAction2 = tab2.build();
        trajectoryAction3 = tab3.build();
        trajectoryActionCloseOut = tabClose.build();
/*
        Actions.runBlocking(
                armRotate.armToRail()
        );

 */

        //I think the ParallelActions messed it up and that is why it didn't work
        /*Actions.runBlocking(
                new SequentialAction(
                        //squeezeClaw, //Grab Specimen
                        armRotate.armToHuman(),
                        new ParallelAction( //Move to Submersible
                                armExtend.extendToRail(),
                                armRotate.armToRail(),
                                trajectoryAction1
                        ),
                        clipSpecimen,
                        new ParallelAction( //Move to human player
                                trajectoryAction2,
                                armExtend.extendToRail(),
                                armRotate.armToTheSky()
                        ),
                        armRotate.armToHuman(),
                        squeezeClaw,
                        new ParallelAction( //Move to Submersible
                                trajectoryAction3,
                                armRotate.armToRail()
                        ),
                        clipSpecimen,
                        new ParallelAction( //Park
                                armRotate.armToHuman(),
                                armExtend.extendRetract(),
                                trajectoryActionCloseOut
                        ),
                        armRotate.armDown()
                )
        );

         */

        Actions.runBlocking(
                new SequentialAction(
                        //squeezeClaw, //Grab Specimen
                        armRotate.armToHuman(),
                         //Move to Submersible
                        armExtend.extendToRail(),
                        armRotate.armToRail(),
                        trajectoryAction1,

                        clipSpecimen,
                        //Move to human player
                        trajectoryAction2,
                        armExtend.extendToRail(),
                        armRotate.armToTheSky(),

                        armRotate.armToHuman(),
                        squeezeClaw,
                       //Move to Submersible
                        trajectoryAction3,
                        armRotate.armToRail(),

                        clipSpecimen,
                        //Park
                        armRotate.armToHuman(),
                        armExtend.extendRetract(),
                        trajectoryActionCloseOut,

                        armRotate.armDown()
                )
        );


    }
}

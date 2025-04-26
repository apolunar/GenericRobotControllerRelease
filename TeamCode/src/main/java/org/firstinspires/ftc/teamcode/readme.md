
# GenericRobotController

Robot legos.

# TeamCode Module

.
├── calibration
│     ├── chassis
│     │     ├── ForwardPushTuner.java
│     │     ├── ForwardRampLogger.java
│     │     ├── LateralPushTuner.java
│     │     └── Tuning.java
│     └── vision
│         ├── ConceptAprilTagOptimizeExposure.java
│         └── UtilityCameraFrameCapture.java
├── math
│     ├── controller
│     │     ├── DriveController.java
│     │     └── HolonomicDriveController.java
│     ├── navigation
│     │     ├── CoordSys.java
│     │     ├── Navigator.java
│     │     └── navigators
│     │         └── AStarNavigator.java
│     ├── trajectory
│     │     ├── TrajectorySegment.java
│     │     └── TrajectorySequence.java
│     └── util
│         ├── Color.java
│         ├── Geometry.java
│         ├── Range.java
│         └── Units.java
├── readme.md
├── robotcore
│     ├── command
│     │     ├── action
│     │     │     ├── GetPropPosition.java
│     │     │     ├── InitAprilTagVision.java
│     │     │     ├── LaunchDrone.java
│     │     │     ├── MoveSlide.java
│     │     │     ├── MoveSpatula.java
│     │     │     ├── MoveTonsils.java
│     │     │     ├── SwitchCamera.java
│     │     │     ├── TrajectoryToAprilTag.java
│     │     │     └── WaitSeconds.java
│     │     ├── AutoCommand.java
│     │     ├── chassis
│     │     │     ├── DriveToPoint.java
│     │     │     └── FollowTrajectory.java
│     │     ├── DriverCommand.java
│     │     ├── scorer
│     │     │     ├── DroneScorer.java
│     │     │     ├── NavigationScorer.java
│     │     │     └── RandomizationScorer.java
│     │     └── sequence
│     │         └── AlignAndDeposit.java
│     ├── drivebase
│     │     └── MyMecanumDrive.java
│     ├── game
│     │     ├── Alliance.java
│     │     ├── AllianceTrajectory.java
│     │     ├── element
│     │     │     ├── Pixel.java
│     │     │     └── Prop.java
│     │     ├── enviroment
│     │     │     ├── maps
│     │     │     │     └── CenterStageObstacleMap.java
│     │     │     ├── obstacle
│     │     │     │     ├── CircleObstacle.java
│     │     │     │     ├── Obstacle.java
│     │     │     │     ├── RectangleObstacle.java
│     │     │     │     └── RobotObstacle.java
│     │     │     ├── ObstacleMap.java
│     │     │     └── StartingPose.java
│     │     ├── GameScorer.java
│     │     └── GameStage.java
│     ├── hardware
│     │     ├── MyRobot.java
│     │     ├── RobotOpMode.java
│     │     ├── RobotState.java
│     │     ├── RobotSubsystem.java
│     │     └── subsystem
│     │         ├── BlinkinSubsystem.java
│     │         ├── CameraSubsystem.java
│     │         ├── ChassisSubsystem.java
│     │         ├── DroneSubsystem.java
│     │         ├── IMUSubsystem.java
│     │         ├── IntakeSubsystem.java
│     │         ├── ScannerSubsystem.java
│     │         ├── ScrewSubsystem.java
│     │         ├── SlideSubsystem.java
│     │         └── SpatulaSubsystem.java
│     ├── opmode
│     │     ├── auto
│     │     │     ├── archive
│     │     │     │     ├── ResetRobotStateTracker.java
│     │     │     │     ├── TurnTheRobot.java
│     │     │     │     └── TurnTheRobotToo.java
│     │     │     ├── AutoOpMode.java
│     │     │     ├── BlueAudienceHolonomicAuto.java
│     │     │     ├── BlueAudienceTwoPlusOneAuto.java
│     │     │     ├── BlueBackstageHolonomicAuto.java
│     │     │     ├── GenericHolonomicAuto.java
│     │     │     ├── RedAudienceHolonomicAuto.java
│     │     │     ├── RedAudienceTwoPlusOneAuto.java
│     │     │     ├── RedBackstageHolonomicAuto.java
│     │     │     ├── SimpleHolonomicAuto.java
│     │     │     ├── SimpleHolonomicTuner.java
│     │     │     ├── SimpleOpenCvPipeline.java
│     │     │     ├── tests
│     │     │     │     ├── TestGetPropPositionBlue.java
│     │     │     │     ├── TestHolonomicTrajectory.java
│     │     │     │     ├── TestThreeCamera.java
│     │     │     │     └── TestTrajectoryToAprilTag.java
│     │     │     ├── TwoPlusOneGenericHolonomicAuto.java
│     │     │     └── TwoPlusOneRecodedTrajectoryAuto.java
│     │     ├── tele
│     │     │     ├── ConfigureRobot.java
│     │     │     ├── TeleOpModeBlue.java
│     │     │     ├── TeleOpMode.java
│     │     │     └── TeleOpModeRed.java
│     │     └── test
│     │         ├── SwerveDemo.java
│     │         └── SwerveViz.java
│     └── vision
│         └── PropCircleAnalysis.java
└── tests.md

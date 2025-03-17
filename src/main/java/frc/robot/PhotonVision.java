/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

 package frc.robot;

 //import static frc.robot.Constants.PhotonVision.*;
 /* unused. Commented out to redice annoyance.
 import static edu.wpi.first.units.Units.Microseconds;
 import static edu.wpi.first.units.Units.Milliseconds;
 import static edu.wpi.first.units.Units.Seconds;
 */
 
 import edu.wpi.first.math.Matrix;
 import edu.wpi.first.math.VecBuilder;
 import edu.wpi.first.math.geometry.Pose2d;
 import edu.wpi.first.math.geometry.Rotation2d;
 import edu.wpi.first.math.geometry.Rotation3d;
 import edu.wpi.first.math.numbers.N1;
 import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.List;
 import java.util.Optional;
 import org.photonvision.EstimatedRobotPose;
 import org.photonvision.PhotonCamera;
 import org.photonvision.PhotonPoseEstimator;
 import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.VisionEstimation;
import org.photonvision.simulation.PhotonCameraSim;
 import org.photonvision.simulation.SimCameraProperties;
 import org.photonvision.simulation.VisionSystemSim;
 import org.photonvision.targeting.PhotonTrackedTarget;

 import edu.wpi.first.apriltag.AprilTagDetection;
 import edu.wpi.first.apriltag.AprilTagFieldLayout;
 import edu.wpi.first.apriltag.AprilTagFields;
 import edu.wpi.first.math.geometry.Transform3d;
 import edu.wpi.first.math.geometry.Translation3d;


 /*//Unused imports. Commented out to reduce annoyance.
  import edu.wpi.first.apriltag.AprilTagFields;
  import edu.wpi.first.math.geometry.Rotation3d;
  
  import edu.wpi.first.math.geometry.Pose3d;
  import edu.wpi.first.math.geometry.Transform2d;
  import edu.wpi.first.math.util.Units;
  import edu.wpi.first.networktables.NetworkTablesJNI;
  import edu.wpi.first.wpilibj.Alert;
  import edu.wpi.first.wpilibj.Alert.AlertType;
  import frc.robot.XCaliper;
  import java.awt.Desktop;
  import java.util.ArrayList;
  import java.util.function.Supplier;
  import org.photonvision.PhotonUtils;
  import org.photonvision.targeting.PhotonPipelineResult;
  import edu.wpi.first.math.Matrix;
  import edu.wpi.first.math.VecBuilder;
  import edu.wpi.first.math.geometry.Pose2d;
  import edu.wpi.first.math.geometry.Rotation2d;
  import edu.wpi.first.math.numbers.N1;
  import edu.wpi.first.math.numbers.N3;
  import edu.wpi.first.wpilibj.smartdashboard.Field2d;
  import java.util.List;
  import java.util.Optional;
  import org.photonvision.EstimatedRobotPose;
  import org.photonvision.PhotonCamera;
  import org.photonvision.PhotonPoseEstimator;
  import org.photonvision.PhotonPoseEstimator.PoseStrategy;
  import org.photonvision.simulation.PhotonCameraSim;
  import org.photonvision.simulation.SimCameraProperties;
  import org.photonvision.simulation.VisionSystemSim;
  import org.photonvision.targeting.PhotonTrackedTarget;
 //*/
 import frc.robot.subsystems.CommandSwerveDrivetrain;
 import frc.robot.CustomMoveMeters;
 /* servelib was causing import problems
 import swervelib.SwerveDrive;
 import swervelib.telemetry.SwerveDriveTelemetry;
 */
 import frc.robot.generated.TunerConstants;
 
 public class PhotonVision {
     private final PhotonCamera camera;
     private final PhotonPoseEstimator photonEstimator;
     private Matrix<N3, N1> curStdDevs;
     private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();;
     //private final CustomMoveMeters customMoveMeters = new CustomMoveMeters(0, drivetrain, false);
 
     public static class PhotonVisionConstants {
         // The standard deviations of our vision estimated poses, which affect
         // correction rate
         // (Fake values. Experiment and determine estimation noise on an actual robot.)
         // n1 is x, n2 is y, n3 is rotation if I remember correctly
         public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(2, 2, 8);
         public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
         public static final String camera1 = "HD_USB_Camera";
         public static final String camera2 = "Arducam_OV9281_USB_Camera";

        //Camera transformations
        public static final Transform3d camera1Transform = new Transform3d(
            new Translation3d(-0.3, 0, 0.5), 
            new Rotation3d(0, 0, 0));
        public static final Transform3d camera2Transform = new Transform3d(
            new Translation3d(-0.3, 0, 0.5),
            new Rotation3d(0, 0, Math.PI));
         public static final String kCameraName = camera2;
        
        


         // Cam mounted facing forward, half a meter forward of center, half a meter up
         // from center --> (new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0))

         public static final Transform3d kRobotToCam = camera1Transform;
         public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout
                 .loadField(AprilTagFields.k2025ReefscapeWelded);
     }
 
     // Constants
     String kCameraName = PhotonVision.PhotonVisionConstants.kCameraName;
     AprilTagFieldLayout kTagLayout = PhotonVision.PhotonVisionConstants.kTagLayout;
     Transform3d kRobotToCam = PhotonVision.PhotonVisionConstants.kRobotToCam;
     Matrix<N3, N1> kSingleTagStdDevs = PhotonVision.PhotonVisionConstants.kSingleTagStdDevs;
     Matrix<N3, N1> kMultiTagStdDevs = PhotonVision.PhotonVisionConstants.kSingleTagStdDevs;
 
     
     public PhotonVision(String passedCamera, Transform3d passedTransormation) {
         camera = new PhotonCamera(passedCamera);
 
         photonEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, passedTransormation);
         photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
     }
 
     /**
      * The latest estimated robot pose on the field from vision data. This may be
      * empty. This should
      * only be called once per loop.
      *
      * <p>
      * Also includes updates for the standard deviations, which can (optionally) be
      * retrieved with
      * {@link getEstimationStdDevs}
      *
      * @return An {@link EstimatedRobotPose} with an estimated pose, estimate
      *         timestamp, and targets
      *         used for estimation.
      */

          //Pretty much making the scope bigger for the aprilTagThing
          Optional<EstimatedRobotPose> visionEstimation = null;
          List<PhotonTrackedTarget> photonTargets = null;
     
          public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
              Optional<EstimatedRobotPose> visionEst = Optional.empty();
              var cameraResults = camera.getAllUnreadResults();
      
              for (var change : cameraResults) {
                  visionEst = photonEstimator.update(change);
                  updateEstimationStdDevs(visionEst, change.getTargets());
                  photonTargets = change.getTargets();
              }
              visionEstimation = visionEst;
              return visionEst;
          }
      
          /**
           * Calculates new standard deviations This algorithm is a heuristic that creates
           * dynamic standard
           * deviations based on number of tags, estimation strategy, and distance from
           * the tags.
           *
           * @param estimatedPose The estimated pose to guess standard deviations for.
           * @param targets       All targets in this camera frame
           */
      
          private void updateEstimationStdDevs(
                  Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
              if (estimatedPose.isEmpty()) {
                  // No pose input. Default to single-tag std devs
                  curStdDevs = kSingleTagStdDevs;
      
              } else {
                  // Pose present. Start running Heuristic
                  var estStdDevs = kSingleTagStdDevs;
                  int numTags = 0;
                  double avgDist = 0;
      
                  // Precalculation - see how many tags we found, and calculate an
                  // average-distance metric
                  for (var tgt : targets) {
                      var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                      if (tagPose.isEmpty())
                          continue;
                      numTags++;
                      avgDist += tagPose
                              .get()
                              .toPose2d()
                              .getTranslation()
                              .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
                  }
                  
                  if (numTags == 0) {
                      // No tags visible. Default to single-tag std devs
                      curStdDevs = kSingleTagStdDevs;
                  } else {
                      // One or more tags visible, run the full heuristic.
                      avgDist /= numTags;
                      // Decrease std devs if multiple targets are visible
                      if (numTags > 1)
                          estStdDevs = kMultiTagStdDevs;
                      // Increase std devs based on (average) distance
                      if (numTags == 1 && avgDist > 4)
                          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                      else
                          estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                      curStdDevs = estStdDevs;
                  }
              }
          }
          
          /**
           * Returns the latest standard deviations of the estimated pose from {@link
           * #getEstimatedGlobalPose()}, for use with {@link
           * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
           * SwerveDrivePoseEstimator}. This should
           * only be used when there are targets visible.
           */
          public Matrix<N3, N1> getEstimationStdDevs() {
              return curStdDevs;
          }
      
          // ----- Simulation
      
          public void simulationPeriodic(Pose2d robotSimPose) {
          }
      
          /** Reset pose history of the robot in the vision system simulation. */
          public void resetSimPose(Pose2d pose) {
          }
      
          /** A Field2d for visualizing our robot and objects on the field. */
          public Field2d getSimDebugField() {
              return null;
          }
     
          //None of the code in this class has been tested due to various errors
          //public class AprilTagThing /*extends Command*/{
            double distanceToMoveWhy = 0;
            //moves to the right or left and aligns us to the reef stack
            public PhotonVision AprilTagThing(
                boolean doWeMoveLeft
                ){  
                    Boolean areWeBlueM = DriverStation.getAlliance().equals("blue");
                    String areWeBlue = areWeBlueM.toString();
                    System.out.println(areWeBlue);
                    Optional<EstimatedRobotPose> estimatedPose = visionEstimation;
                    List<PhotonTrackedTarget> targets = photonTargets;

                    int numOfTags = 0;
                    double avgDistance = 0;
                    double tagY = 0;
                    double tagX = 0;
                    double robotY = 0;
                    double robotX = 0;
                    double distanceToMoveY = 0;
                    double distanceToMoveX = 0;

                    for (var tgt : targets) {
                        var tagId = tgt.getFiducialId();
                        var tagPose = photonEstimator.getFieldTags().getTagPose(tagId);
                        if (tagPose.isEmpty())
                            continue;

                        switch(areWeBlue){
                            case "false":
                            case "False":
                            case "No":
                                switch(tagId){
                                    //add the other tags too. More than four.
                                    case 6:
                                    case 7:
                                    case 8:
                                    case 9:
                                    case 10:
                                    case 11:
                                        //continue with the code
                                        break;
                                    default:
                                        //not the correct tag, search for correct tag.
                                        continue;
                                }
                                break;

                            case "Yes":
                            case "True":
                            case "true":
                                switch(tagId){
                                    case 17:
                                    case 18: 
                                    case 19:
                                    case 20:
                                    case 21:
                                    case 22:
                                        //continue with the code
                                        break;
                                    default:
                                        //not the correct tag; search for correct tag.
                                        continue;
                                }
                            default:
                                continue;

                        }
                    

                        numOfTags++;
                        avgDistance += tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());

                        tagY = tagPose.get().getY();
                        robotY = estimatedPose.get().estimatedPose.toPose2d().getY();
                        distanceToMoveY = robotY - tagY;
                    }
                        //I have a new thing here because otherwise it complains about the scope.
                        double distanceToMoveWWWHHHHHYYYY = distanceToMoveY;
                        distanceToMoveWhy = distanceToMoveY;
                        new InstantCommand(() -> 
                            new CustomMoveMeters(distanceToMoveWWWHHHHHYYYY, drivetrain, false)./*Chat GPT said to add schedual*/schedule()
                        );
                        SmartDashboard.putNumber("RobotY", robotY);
                        SmartDashboard.putNumber("tagY", tagY);
                        SmartDashboard.putNumber("DistanceToMoveY", distanceToMoveY);

                    
                SmartDashboard.putNumber("RobotY", robotY);
                SmartDashboard.putNumber("tagY", tagY);
                SmartDashboard.putNumber("DistanceToMoveY", distanceToMoveY);
                return null;
            }

            //public static Constant someRandomFunction = null;
            //static PhotonVision.AprilTagThing hello = null;
            /*
                @Override
                public void initialize() {
                    
                }

                @Override
                public void execute() {
                    new CustomMoveMeters(distanceToMoveWhy, drivetrain, false);
                    }
             */
        //}
            /* 
        public PhotonVision AprilTagThing(boolean b) {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'AprilTagThing'");
        }*/
 }
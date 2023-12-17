package org.firstinspires.ftc.teamcode.helpers;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Config
public class Storage {
    public static int PLANE_SHOOT = 0;
    public static int PLANE_ARM = 0;
    public static int MINIMUM_SLIDE_PLACE_L = -4440;
    public static int MINIMUM_SLIDE_PLACE_R = 4440;
    public static Pose2d currentPose;
    public static SampleMecanumDrive robot;
    public interface TRAJECTORIES {
        interface BACKSTAGE {
            interface BLUE {
                interface LEFT {

                }
                interface MIDDLE {

                }
                interface RIGHT {

                }
            }
            interface RED {
                interface LEFT {

                }
                interface MIDDLE {

                }
                interface RIGHT {

                }
            }
        }
        interface AUDIENCE {
            interface BLUE {
                Pose2d startPose = new Pose2d(-36, 60, Math.toRadians(270));
                interface LEFT {
                    TrajectorySequence trajOne = robot.trajectorySequenceBuilder(startPose)
                            .lineTo(new Vector2d(25, 61))
                            .lineTo(new Vector2d(25, 40))
                            .build();
                }
                interface MIDDLE {

                }
                interface RIGHT {

                }
            }
            interface RED {
                interface LEFT {

                }
                interface MIDDLE {

                }
                interface RIGHT {

                }
            }
        }
    }
}

package org.firstinspires.ftc.teamcode.roadrunner.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.teamcode.roadrunner.drive.opmode.*;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;

import java.util.ArrayList;

public final class RegisterRoadRunnerTuning {
    public static final boolean DISABLED = true;
    public static final String GROUP = "roadrunner";
    private RegisterRoadRunnerTuning() {}

    private static OpModeMeta metaForClass(Class<? extends OpMode> cls) {
        return new OpModeMeta.Builder()
                .setName(cls.getSimpleName())
                .setGroup(GROUP)
                .setFlavor(OpModeMeta.Flavor.TELEOP)
                .setSource(OpModeMeta.Source.BLOCKLY)
                .build();
    }
    
    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        if (DISABLED) return;
        ArrayList<Class<? extends OpMode>> list = new ArrayList<>();
        list.add(AutomaticFeedforwardTuner.class);
        list.add(BackAndForth.class);
        list.add(DriveVelocityPIDTuner.class);
        list.add(FollowerPIDTuner.class);
        list.add(LocalizationTest.class);
        list.add(ManualFeedforwardTuner.class);
        list.add(MaxAngularVeloTuner.class);
        list.add(MaxVelocityTuner.class);
        list.add(MotorDirectionDebugger.class);
        list.add(SplineTest.class);
        list.add(StrafeTest.class);
        list.add(StraightTest.class);
        list.add(TrackWidthTuner.class);
        list.add(TurnTest.class);
        list.add(XYTest.class);

        for (Class<? extends OpMode> opmode : list) {
            try {
                manager.register(metaForClass(opmode), opmode.newInstance());
            } catch (IllegalAccessException | InstantiationException e) {
                throw new RuntimeException(e);
            }
        }

        FtcDashboard.getInstance().withConfigRoot(configRoot -> {
            for(Class<? extends OpMode> opmode : list) {
                configRoot.putVariable(opmode.getSimpleName(), ReflectionConfig.createVariableFromClass(opmode));
            }
        });
    }
}

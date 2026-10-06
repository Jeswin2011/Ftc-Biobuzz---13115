package org.firstinspires.ftc.teamcode;

import com.pedropathing.tuning.autotune.*;

import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;

public class Tuning {
    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }
}

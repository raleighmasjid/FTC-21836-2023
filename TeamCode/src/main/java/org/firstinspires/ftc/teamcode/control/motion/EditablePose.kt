package org.firstinspires.ftc.teamcode.control.motion

import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.opmodes.MainAuton.X_START_LEFT
import org.firstinspires.ftc.teamcode.opmodes.MainAuton.X_START_RIGHT
import org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot.isRed
import org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot.isRight

data class EditablePose

@JvmOverloads
constructor(
    @JvmField var x: Double = 0.0,
    @JvmField var y: Double = 0.0,
    @JvmField var heading: Double = 0.0,
) {
    fun byAlliance(): EditablePose {
        val alliance: Double = if (isRed) 1.0 else -1.0
        y *= alliance
        heading *= alliance
        return this
    }

    fun bySide(): EditablePose {
        x += if (isRight == isRed) 0.0 else X_START_LEFT - X_START_RIGHT
        return this
    }

    fun byBoth(): EditablePose {
        return byAlliance().bySide()
    }

    fun toPose2d(): Pose2d {
        return Pose2d(x, y, heading)
    }

    fun flipBySide(): EditablePose {
        if (!isRight) heading = Math.PI - heading
        if (!isRight) x = (X_START_LEFT + X_START_RIGHT) / 2 - x
        return this
    }
}
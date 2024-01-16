package com.example.meepmeeptesting

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.example.meepmeeptesting.MeepMeepTesting.*

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
        if (backdropSide != isRed) x += X_START_LEFT - X_START_RIGHT
        return this
    }

    fun byBoth(): EditablePose {
        return byAlliance().bySide()
    }

    fun toPose2d(): Pose2d {
        return Pose2d(x, y, heading)
    }

    fun flipBySide(): EditablePose {
        if (!backdropSide) heading = Math.PI - heading
        if (!backdropSide) x = (X_START_LEFT + X_START_RIGHT) - x
        return this
    }
}
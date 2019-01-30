package com.acmerobotics.roadrunner.followers

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.path.Path
import com.acmerobotics.roadrunner.util.NanoClock
import kotlin.math.abs

/**
 * Generic [Path] follower for time-independent pose reference tracking.
 *
 * @param clock clock
 */
abstract class PathFollower @JvmOverloads constructor(
        private val admissibleError: Pose2d,
        protected val clock: NanoClock = NanoClock.system()
) {
    private var startTimestamp: Double = 0.0
    private var admissible = false

    /**
     * Path being followed if [isFollowing] is true.
     */
    var path: Path = Path()
        protected set

    /**
     * Robot pose error computed in the last [update] call.
     */
    abstract var lastError: Pose2d
        protected set

    /**
     * Follow the given [path].
     *
     * @param path path
     * @param
     */
    open fun followPath(path: Path) {
        this.startTimestamp = clock.seconds()
        this.path = path
        this.admissible = false
    }

    /**
     * Returns true if the current path has finished executing.
     */
    open fun isFollowing(): Boolean {
        return !admissible
    }

    /**
     * Run a single iteration of the path follower.
     *
     * @param currentPose current robot pose
     */
    open fun update(currentPose: Pose2d) {
        val pathEndError = path.end() - currentPose
        admissible = abs(pathEndError.x) < admissibleError.x
                && abs(pathEndError.y) < admissibleError.y
                && abs(pathEndError.heading) < admissibleError.heading
    }
}
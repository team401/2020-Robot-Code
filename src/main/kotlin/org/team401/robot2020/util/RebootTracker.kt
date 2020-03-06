package org.team401.robot2020.util

import org.snakeskin.dsl.isHardware
import java.io.File

object RebootTracker {
    /**
     * True if the RIO has rebooted right before code started, false otherwise
     */
    val hasRebooted by lazy {
        if (isHardware) {
            //Check if the file exists
            val f = File("/dev/shm/reboot_tracker")
            if (f.exists()) {
                //File exists, system has not rebooted
                false
            } else {
                //File does not exist, system has rebooted, create the file
                f.createNewFile()
                true
            }
        } else {
            true //Always has rebooted on non-hardware platforms
        }
    }
}
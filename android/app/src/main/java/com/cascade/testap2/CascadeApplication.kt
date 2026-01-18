package com.cascade.testap2

import android.app.Application

/**
 * Application class for Cascade Autopilot
 *
 * Holds shared resources like the BLE manager that need to persist
 * across activity lifecycle changes.
 */
class CascadeApplication : Application() {

    lateinit var bleManager: AutopilotBleManager
        private set

    override fun onCreate() {
        super.onCreate()
        bleManager = AutopilotBleManager(this)
    }

    override fun onTerminate() {
        super.onTerminate()
        bleManager.close()
    }
}

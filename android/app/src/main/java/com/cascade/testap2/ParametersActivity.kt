package com.cascade.testap2

import android.os.Bundle
import android.widget.Toast
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import com.cascade.testap2.databinding.ActivityParametersBinding
import com.cascade.testap2.databinding.ParamRowBinding
import com.google.android.material.slider.Slider

class ParametersActivity : AppCompatActivity() {

    private lateinit var binding: ActivityParametersBinding
    private lateinit var bleManager: AutopilotBleManager

    // Parameter bindings
    private data class ParamBinding(
        val id: Int,
        val rowBinding: ParamRowBinding,
        var min: Float = 0f,
        var max: Float = 100f,
        var currentValue: Float = 0f
    )

    private val paramBindings = mutableListOf<ParamBinding>()

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityParametersBinding.inflate(layoutInflater)
        setContentView(binding.root)

        // Get shared BLE manager from application
        bleManager = (application as CascadeApplication).bleManager

        setupParamRows()
        setupCallbacks()
        setupButtons()

        // Request all parameters
        for (i in 0..8) {
            bleManager.getParam(i)
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        // BLE manager is now managed by CascadeApplication
    }

    private fun setupParamRows() {
        // Map param IDs to their row bindings
        val rows = listOf(
            ParamBinding(0, ParamRowBinding.bind(binding.paramKpHeading.root)),
            ParamBinding(1, ParamRowBinding.bind(binding.paramKiHeading.root)),
            ParamBinding(2, ParamRowBinding.bind(binding.paramKdHeading.root)),
            ParamBinding(3, ParamRowBinding.bind(binding.paramKpServo.root)),
            ParamBinding(4, ParamRowBinding.bind(binding.paramDeadbandEnter.root)),
            ParamBinding(5, ParamRowBinding.bind(binding.paramDeadbandExit.root)),
            ParamBinding(6, ParamRowBinding.bind(binding.paramMinSpeed.root)),
            ParamBinding(7, ParamRowBinding.bind(binding.paramMaxSpeed.root)),
            ParamBinding(8, ParamRowBinding.bind(binding.paramSlewRate.root))
        )

        rows.forEach { param ->
            param.rowBinding.paramName.text = ParamData.getParamName(param.id)
            param.rowBinding.paramSlider.addOnChangeListener { _, value, fromUser ->
                if (fromUser) {
                    param.currentValue = value
                    param.rowBinding.paramValue.text = formatValue(param.id, value)
                    // Send value to device
                    bleManager.setParam(param.id, value)
                }
            }
            paramBindings.add(param)
        }
    }

    private fun setupCallbacks() {
        bleManager.onParamReceived = { paramData ->
            runOnUiThread {
                updateParamDisplay(paramData)
            }
        }

        bleManager.onError = { error ->
            runOnUiThread {
                Toast.makeText(this, error, Toast.LENGTH_SHORT).show()
            }
        }
    }

    private fun setupButtons() {
        binding.btnSaveAll.setOnClickListener {
            bleManager.saveAllParams()
            Toast.makeText(this, "Parameters saved to flash", Toast.LENGTH_SHORT).show()
        }

        binding.btnResetDefaults.setOnClickListener {
            AlertDialog.Builder(this)
                .setTitle("Reset Parameters")
                .setMessage("Reset all parameters to factory defaults?")
                .setPositiveButton("Reset") { _, _ ->
                    bleManager.resetParams()
                    Toast.makeText(this, "Parameters reset to defaults", Toast.LENGTH_SHORT).show()
                    // Re-request all parameters
                    for (i in 0..8) {
                        bleManager.getParam(i)
                    }
                }
                .setNegativeButton("Cancel", null)
                .show()
        }
    }

    private fun updateParamDisplay(paramData: ParamData) {
        val binding = paramBindings.find { it.id == paramData.id } ?: return

        binding.min = paramData.min
        binding.max = paramData.max
        binding.currentValue = paramData.value

        binding.rowBinding.paramSlider.valueFrom = paramData.min
        binding.rowBinding.paramSlider.valueTo = paramData.max
        binding.rowBinding.paramSlider.value = paramData.value.coerceIn(paramData.min, paramData.max)
        binding.rowBinding.paramValue.text = formatValue(paramData.id, paramData.value)
    }

    private fun formatValue(paramId: Int, value: Float): String {
        return when (paramId) {
            6, 7 -> String.format("%.0f%%", value)  // Motor speeds as percentages
            else -> String.format("%.2f", value)
        }
    }
}

package com.wildbridge.rc.pages

import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.fragment.app.activityViewModels
import com.wildbridge.rc.R
import com.wildbridge.rc.databinding.FragMegaphonePageBinding
import com.wildbridge.rc.databinding.FragMegaphoneRealtimeBinding
import com.wildbridge.rc.databinding.FragMopCenterPageBinding
import com.wildbridge.rc.models.MegaphoneVM

class RealTimeFragment: DJIFragment() {
    private val megaphoneVM: MegaphoneVM by activityViewModels()
    private var recordStarted:Boolean = false
    private var binding: FragMegaphoneRealtimeBinding? = null

    override fun onCreateView(
        inflater: LayoutInflater,
        container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View? {
        binding = FragMegaphoneRealtimeBinding.inflate(inflater, container, false)
        return binding?.root
    }

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        initBtnListener()
    }

    private fun initBtnListener() {
        binding?.btnRealtime?.setOnClickListener {
            if (!recordStarted) {
                megaphoneVM.startRecord()
                binding?.btnRealtime?.setImageDrawable(requireContext().getDrawable(R.drawable.ic_official_speaker_real_time_stop))
                binding?.tvSpeakingControl?.setText(R.string.btn_press_to_stop_record)
                recordStarted = true
            } else {
                megaphoneVM.stopRecord()
                binding?.btnRealtime?.setImageDrawable(requireContext().getDrawable(R.drawable.ic_official_speaker_real_time_start))
                binding?.tvSpeakingControl?.setText(R.string.btn_press_to_start_record)
                recordStarted = false
            }
        }

        binding?.cbAgc?.setOnCheckedChangeListener { _, isChecked ->
           megaphoneVM.enableAgc(isChecked)
        }

    }
}
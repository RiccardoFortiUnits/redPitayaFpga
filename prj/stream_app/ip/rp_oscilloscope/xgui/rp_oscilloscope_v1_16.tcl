# Definitional proc to organize widgets for parameters.
proc init_gui { IPINST } {
  ipgui::add_param $IPINST -name "Component_Name"
  #Adding Page
  set Page_0 [ipgui::add_page $IPINST -name "Page 0"]
  ipgui::add_param $IPINST -name "ADC_DATA_BITS" -parent ${Page_0}
  ipgui::add_param $IPINST -name "EVENT_SRC_NUM" -parent ${Page_0}
  ipgui::add_param $IPINST -name "M_AXI_OSC1_ADDR_BITS" -parent ${Page_0}
  ipgui::add_param $IPINST -name "M_AXI_OSC1_DATA_BITS" -parent ${Page_0}
  ipgui::add_param $IPINST -name "M_AXI_OSC2_ADDR_BITS" -parent ${Page_0}
  ipgui::add_param $IPINST -name "M_AXI_OSC2_DATA_BITS" -parent ${Page_0}
  ipgui::add_param $IPINST -name "S_AXI_REG_ADDR_BITS" -parent ${Page_0}
  ipgui::add_param $IPINST -name "TRIG_SRC_NUM" -parent ${Page_0}
  ipgui::add_param $IPINST -name "NUM_CHANNELS" -parent ${Page_0}
  ipgui::add_param $IPINST -name "ID_WIDTHS" -parent ${Page_0}


}

proc update_PARAM_VALUE.ADC_DATA_BITS { PARAM_VALUE.ADC_DATA_BITS } {
	# Procedure called to update ADC_DATA_BITS when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.ADC_DATA_BITS { PARAM_VALUE.ADC_DATA_BITS } {
	# Procedure called to validate ADC_DATA_BITS
	return true
}

proc update_PARAM_VALUE.EVENT_SRC_NUM { PARAM_VALUE.EVENT_SRC_NUM } {
	# Procedure called to update EVENT_SRC_NUM when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.EVENT_SRC_NUM { PARAM_VALUE.EVENT_SRC_NUM } {
	# Procedure called to validate EVENT_SRC_NUM
	return true
}

proc update_PARAM_VALUE.ID_WIDTHS { PARAM_VALUE.ID_WIDTHS } {
	# Procedure called to update ID_WIDTHS when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.ID_WIDTHS { PARAM_VALUE.ID_WIDTHS } {
	# Procedure called to validate ID_WIDTHS
	return true
}

proc update_PARAM_VALUE.M_AXI_OSC1_ADDR_BITS { PARAM_VALUE.M_AXI_OSC1_ADDR_BITS } {
	# Procedure called to update M_AXI_OSC1_ADDR_BITS when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.M_AXI_OSC1_ADDR_BITS { PARAM_VALUE.M_AXI_OSC1_ADDR_BITS } {
	# Procedure called to validate M_AXI_OSC1_ADDR_BITS
	return true
}

proc update_PARAM_VALUE.M_AXI_OSC1_DATA_BITS { PARAM_VALUE.M_AXI_OSC1_DATA_BITS } {
	# Procedure called to update M_AXI_OSC1_DATA_BITS when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.M_AXI_OSC1_DATA_BITS { PARAM_VALUE.M_AXI_OSC1_DATA_BITS } {
	# Procedure called to validate M_AXI_OSC1_DATA_BITS
	return true
}

proc update_PARAM_VALUE.M_AXI_OSC2_ADDR_BITS { PARAM_VALUE.M_AXI_OSC2_ADDR_BITS } {
	# Procedure called to update M_AXI_OSC2_ADDR_BITS when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.M_AXI_OSC2_ADDR_BITS { PARAM_VALUE.M_AXI_OSC2_ADDR_BITS } {
	# Procedure called to validate M_AXI_OSC2_ADDR_BITS
	return true
}

proc update_PARAM_VALUE.M_AXI_OSC2_DATA_BITS { PARAM_VALUE.M_AXI_OSC2_DATA_BITS } {
	# Procedure called to update M_AXI_OSC2_DATA_BITS when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.M_AXI_OSC2_DATA_BITS { PARAM_VALUE.M_AXI_OSC2_DATA_BITS } {
	# Procedure called to validate M_AXI_OSC2_DATA_BITS
	return true
}

proc update_PARAM_VALUE.M_AXI_OSC3_ADDR_BITS { PARAM_VALUE.M_AXI_OSC3_ADDR_BITS } {
	# Procedure called to update M_AXI_OSC3_ADDR_BITS when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.M_AXI_OSC3_ADDR_BITS { PARAM_VALUE.M_AXI_OSC3_ADDR_BITS } {
	# Procedure called to validate M_AXI_OSC3_ADDR_BITS
	return true
}

proc update_PARAM_VALUE.M_AXI_OSC3_DATA_BITS { PARAM_VALUE.M_AXI_OSC3_DATA_BITS } {
	# Procedure called to update M_AXI_OSC3_DATA_BITS when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.M_AXI_OSC3_DATA_BITS { PARAM_VALUE.M_AXI_OSC3_DATA_BITS } {
	# Procedure called to validate M_AXI_OSC3_DATA_BITS
	return true
}

proc update_PARAM_VALUE.M_AXI_OSC4_ADDR_BITS { PARAM_VALUE.M_AXI_OSC4_ADDR_BITS } {
	# Procedure called to update M_AXI_OSC4_ADDR_BITS when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.M_AXI_OSC4_ADDR_BITS { PARAM_VALUE.M_AXI_OSC4_ADDR_BITS } {
	# Procedure called to validate M_AXI_OSC4_ADDR_BITS
	return true
}

proc update_PARAM_VALUE.M_AXI_OSC4_DATA_BITS { PARAM_VALUE.M_AXI_OSC4_DATA_BITS } {
	# Procedure called to update M_AXI_OSC4_DATA_BITS when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.M_AXI_OSC4_DATA_BITS { PARAM_VALUE.M_AXI_OSC4_DATA_BITS } {
	# Procedure called to validate M_AXI_OSC4_DATA_BITS
	return true
}

proc update_PARAM_VALUE.NUM_CHANNELS { PARAM_VALUE.NUM_CHANNELS } {
	# Procedure called to update NUM_CHANNELS when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.NUM_CHANNELS { PARAM_VALUE.NUM_CHANNELS } {
	# Procedure called to validate NUM_CHANNELS
	return true
}

proc update_PARAM_VALUE.S_AXI_REG_ADDR_BITS { PARAM_VALUE.S_AXI_REG_ADDR_BITS } {
	# Procedure called to update S_AXI_REG_ADDR_BITS when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.S_AXI_REG_ADDR_BITS { PARAM_VALUE.S_AXI_REG_ADDR_BITS } {
	# Procedure called to validate S_AXI_REG_ADDR_BITS
	return true
}

proc update_PARAM_VALUE.TRIG_SRC_NUM { PARAM_VALUE.TRIG_SRC_NUM } {
	# Procedure called to update TRIG_SRC_NUM when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.TRIG_SRC_NUM { PARAM_VALUE.TRIG_SRC_NUM } {
	# Procedure called to validate TRIG_SRC_NUM
	return true
}


proc update_MODELPARAM_VALUE.S_AXI_REG_ADDR_BITS { MODELPARAM_VALUE.S_AXI_REG_ADDR_BITS PARAM_VALUE.S_AXI_REG_ADDR_BITS } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.S_AXI_REG_ADDR_BITS}] ${MODELPARAM_VALUE.S_AXI_REG_ADDR_BITS}
}

proc update_MODELPARAM_VALUE.M_AXI_OSC1_ADDR_BITS { MODELPARAM_VALUE.M_AXI_OSC1_ADDR_BITS PARAM_VALUE.M_AXI_OSC1_ADDR_BITS } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.M_AXI_OSC1_ADDR_BITS}] ${MODELPARAM_VALUE.M_AXI_OSC1_ADDR_BITS}
}

proc update_MODELPARAM_VALUE.M_AXI_OSC1_DATA_BITS { MODELPARAM_VALUE.M_AXI_OSC1_DATA_BITS PARAM_VALUE.M_AXI_OSC1_DATA_BITS } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.M_AXI_OSC1_DATA_BITS}] ${MODELPARAM_VALUE.M_AXI_OSC1_DATA_BITS}
}

proc update_MODELPARAM_VALUE.M_AXI_OSC2_ADDR_BITS { MODELPARAM_VALUE.M_AXI_OSC2_ADDR_BITS PARAM_VALUE.M_AXI_OSC2_ADDR_BITS } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.M_AXI_OSC2_ADDR_BITS}] ${MODELPARAM_VALUE.M_AXI_OSC2_ADDR_BITS}
}

proc update_MODELPARAM_VALUE.M_AXI_OSC2_DATA_BITS { MODELPARAM_VALUE.M_AXI_OSC2_DATA_BITS PARAM_VALUE.M_AXI_OSC2_DATA_BITS } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.M_AXI_OSC2_DATA_BITS}] ${MODELPARAM_VALUE.M_AXI_OSC2_DATA_BITS}
}

proc update_MODELPARAM_VALUE.M_AXI_OSC3_ADDR_BITS { MODELPARAM_VALUE.M_AXI_OSC3_ADDR_BITS PARAM_VALUE.M_AXI_OSC3_ADDR_BITS } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.M_AXI_OSC3_ADDR_BITS}] ${MODELPARAM_VALUE.M_AXI_OSC3_ADDR_BITS}
}

proc update_MODELPARAM_VALUE.M_AXI_OSC3_DATA_BITS { MODELPARAM_VALUE.M_AXI_OSC3_DATA_BITS PARAM_VALUE.M_AXI_OSC3_DATA_BITS } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.M_AXI_OSC3_DATA_BITS}] ${MODELPARAM_VALUE.M_AXI_OSC3_DATA_BITS}
}

proc update_MODELPARAM_VALUE.M_AXI_OSC4_ADDR_BITS { MODELPARAM_VALUE.M_AXI_OSC4_ADDR_BITS PARAM_VALUE.M_AXI_OSC4_ADDR_BITS } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.M_AXI_OSC4_ADDR_BITS}] ${MODELPARAM_VALUE.M_AXI_OSC4_ADDR_BITS}
}

proc update_MODELPARAM_VALUE.M_AXI_OSC4_DATA_BITS { MODELPARAM_VALUE.M_AXI_OSC4_DATA_BITS PARAM_VALUE.M_AXI_OSC4_DATA_BITS } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.M_AXI_OSC4_DATA_BITS}] ${MODELPARAM_VALUE.M_AXI_OSC4_DATA_BITS}
}

proc update_MODELPARAM_VALUE.ADC_DATA_BITS { MODELPARAM_VALUE.ADC_DATA_BITS PARAM_VALUE.ADC_DATA_BITS } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.ADC_DATA_BITS}] ${MODELPARAM_VALUE.ADC_DATA_BITS}
}

proc update_MODELPARAM_VALUE.EVENT_SRC_NUM { MODELPARAM_VALUE.EVENT_SRC_NUM PARAM_VALUE.EVENT_SRC_NUM } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.EVENT_SRC_NUM}] ${MODELPARAM_VALUE.EVENT_SRC_NUM}
}

proc update_MODELPARAM_VALUE.TRIG_SRC_NUM { MODELPARAM_VALUE.TRIG_SRC_NUM PARAM_VALUE.TRIG_SRC_NUM } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.TRIG_SRC_NUM}] ${MODELPARAM_VALUE.TRIG_SRC_NUM}
}

proc update_MODELPARAM_VALUE.NUM_CHANNELS { MODELPARAM_VALUE.NUM_CHANNELS PARAM_VALUE.NUM_CHANNELS } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.NUM_CHANNELS}] ${MODELPARAM_VALUE.NUM_CHANNELS}
}

proc update_MODELPARAM_VALUE.ID_WIDTHS { MODELPARAM_VALUE.ID_WIDTHS PARAM_VALUE.ID_WIDTHS } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.ID_WIDTHS}] ${MODELPARAM_VALUE.ID_WIDTHS}
}


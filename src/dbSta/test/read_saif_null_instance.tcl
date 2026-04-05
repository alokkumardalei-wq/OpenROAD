# read_saif with SAIF hierarchy not present in netlist.
# Should skip missing instances without crashing.
source "helpers.tcl"
read_liberty read_saif_null_instance.lib
read_lef liberty1.lef
read_verilog read_saif_null_instance.v
link_design top
read_saif -scope TOP read_saif_null_instance.saif
puts SUCCESS

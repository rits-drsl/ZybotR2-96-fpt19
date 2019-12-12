open_project preimproc_prj
set_top preimproc
add_files ../src/hls/preimproc/preimproc.cpp
add_files ../src/hls/preimproc/HlsImProc.hpp
open_solution "solution1"
set_part {xczu3eg-sbva484-1-e} -tool vivado
create_clock -period 6.67 -name default
csynth_design
export_design -rtl verilog -format ip_catalog

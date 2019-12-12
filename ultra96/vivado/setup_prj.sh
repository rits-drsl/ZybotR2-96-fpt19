#!/bin/sh
cd prj
env SWT_GTK3=0 vivado_hls generate_hls_ip.tcl
env SWT_GTK3=0 vivado -source create_project.tcl

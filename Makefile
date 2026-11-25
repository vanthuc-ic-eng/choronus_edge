# ============================================================================
# Project: CHRONOS — Zero-DSP Logarithmic Selective State-Space Hardware Engine
# File   : Makefile
# Desc   : Automated Lint (Verilator -Wall), Full Regression (4 Testbenches),
#          and Gate-Level 0-DSP Synthesis Proof (Yosys).
# ============================================================================

VERILATOR ?= verilator
YOSYS     ?= yosys
BUILD_DIR := build

RTL_PKG := \
	rtl/pkg/chronos_pkg.sv

RTL_ARITH := \
	rtl/arith/chronos_icg.sv \
	rtl/arith/chronos_lod_u8.sv \
	rtl/arith/chronos_lod_s16.sv \
	rtl/arith/chronos_log_antilog_s16.sv \
	rtl/arith/chronos_log_zoh_unit.sv \
	rtl/arith/chronos_pwl_nonlinear.sv \
	rtl/arith/chronos_conv1d_log8.sv \
	rtl/arith/chronos_pe_log8_mac.sv

RTL_MEM := \
	rtl/mem/chronos_sram_macro_stub.sv \
	rtl/mem/chronos_tcdm_input_ring.sv \
	rtl/mem/chronos_tcdm_state_dp.sv \
	rtl/mem/chronos_tcdm_weight_log8.sv \
	rtl/mem/chronos_tcdm_bias_param.sv

RTL_ENGINE := \
	rtl/engine/chronos_activity_gate.sv \
	rtl/engine/chronos_proj_engine_1x4.sv \
	rtl/engine/chronos_ssm_core_1lane.sv \
	rtl/engine/chronos_delta_skip_ctrl.sv

RTL_CTRL := \
	rtl/ctrl/chronos_argmax_tree.sv \
	rtl/ctrl/chronos_csr_if.sv \
	rtl/ctrl/chronos_stream_agu.sv

RTL_SOC := \
	rtl/soc/chronos_pmu.sv \
	rtl/soc/chronos_dma_ctrl.sv \
	rtl/soc/chronos_riscv_rv32i.sv \
	rtl/soc/chronos_soc_bus.sv \
	rtl/soc/chronos_uart_duplex.sv

RTL_TOP := \
	rtl/top/chronos_ssm_hwpe_top.sv \
	rtl/top/chronos_soc_top.sv

ALL_HWPE_RTL := $(RTL_PKG) $(RTL_ARITH) $(RTL_MEM) $(RTL_ENGINE) $(RTL_CTRL) rtl/top/chronos_ssm_hwpe_top.sv
ALL_SOC_RTL  := $(ALL_HWPE_RTL) $(RTL_SOC) rtl/top/chronos_soc_top.sv

VFLAGS    := --sv -Wall -Wno-DECLFILENAME --timescale 1ns/1ps
TB_VFLAGS := $(VFLAGS) -Wno-WIDTHTRUNC -Wno-REALCVT -Wno-BLKSEQ -Wno-UNUSEDSIGNAL

.PHONY: all lint sim sim_arith sim_drift sim_tcdm sim_sys sim_soc sim_beyond_q1 synth_check benchmark_q1 clean

all: lint sim synth_check

benchmark_q1:
	@echo ">>> [Q1-BENCH] Running Analytical Drift Bound, Pareto DSE & Yosys Gate-Level PPA Comparison..."
	python3 scripts/q1_dse_ppa_benchmark.py

lint:
	@echo ">>> [LINT] Running Verilator -Wall strict lint on FULL BEYOND-Q1 CHRONOS SoC (28 RTL files)..."
	$(VERILATOR) --lint-only $(VFLAGS) --top-module chronos_soc_top $(ALL_SOC_RTL)
	@echo ">>> [LINT] PASS — 0 warnings, 0 errors across entire SoC!"

sim: sim_arith sim_drift sim_tcdm sim_sys sim_soc sim_beyond_q1

sim_arith:
	@mkdir -p $(BUILD_DIR)
	$(VERILATOR) --binary --timing $(TB_VFLAGS) \
		--Mdir $(BUILD_DIR)/obj_arith -o Vtb_arith \
		--top-module tb_arith_units $(RTL_PKG) $(RTL_ARITH) tb/tb_arith_units.sv
	./$(BUILD_DIR)/obj_arith/Vtb_arith

sim_drift:
	@mkdir -p $(BUILD_DIR)
	$(VERILATOR) --binary --timing $(TB_VFLAGS) \
		--Mdir $(BUILD_DIR)/obj_drift -o Vtb_drift \
		--top-module tb_ssm_core_drift $(RTL_PKG) $(RTL_ARITH) $(RTL_MEM) \
		rtl/engine/chronos_ssm_core_1lane.sv tb/tb_ssm_core_drift.sv
	./$(BUILD_DIR)/obj_drift/Vtb_drift

sim_tcdm:
	@mkdir -p $(BUILD_DIR)
	$(VERILATOR) --binary --timing $(TB_VFLAGS) \
		--Mdir $(BUILD_DIR)/obj_tcdm -o Vtb_tcdm \
		--top-module tb_tcdm_zerobubble $(RTL_PKG) $(RTL_MEM) tb/tb_tcdm_zerobubble.sv
	./$(BUILD_DIR)/obj_tcdm/Vtb_tcdm

sim_sys:
	@mkdir -p $(BUILD_DIR)
	$(VERILATOR) --binary --timing $(TB_VFLAGS) \
		--Mdir $(BUILD_DIR)/obj_sys -o Vtb_sys \
		--top-module tb_chronos_system $(ALL_HWPE_RTL) tb/tb_chronos_system.sv
	./$(BUILD_DIR)/obj_sys/Vtb_sys

sim_soc:
	@mkdir -p $(BUILD_DIR)
	$(VERILATOR) --binary --timing $(TB_VFLAGS) \
		--Mdir $(BUILD_DIR)/obj_soc -o Vtb_soc \
		--top-module tb_chronos_soc $(ALL_SOC_RTL) tb/tb_chronos_soc.sv
	./$(BUILD_DIR)/obj_soc/Vtb_soc

sim_beyond_q1:
	@mkdir -p $(BUILD_DIR)
	$(VERILATOR) --binary --timing $(TB_VFLAGS) \
		--Mdir $(BUILD_DIR)/obj_bq1 -o Vtb_bq1 \
		--top-module tb_chronos_beyond_q1 $(ALL_SOC_RTL) tb/tb_chronos_beyond_q1.sv
	./$(BUILD_DIR)/obj_bq1/Vtb_bq1

synth_check:
	@mkdir -p $(BUILD_DIR)
	@echo ">>> [YOSYS] Verifying Synthesizability & 0-DSP (0 mul cells) on FULL chronos_soc_top..."
	@bash -o pipefail -c '$(YOSYS) -p "read_verilog -sv -DSYNTHESIS_FPGA_NO_GATED_CLK $(ALL_SOC_RTL); \
		hierarchy -check -top chronos_soc_top; \
		proc; opt; memory -nomap; opt; stat" > $(BUILD_DIR)/yosys_stat.rpt'
	@tail -n 35 $(BUILD_DIR)/yosys_stat.rpt
	@if grep -q '\$$mul' $(BUILD_DIR)/yosys_stat.rpt; then \
		echo "[FAIL] Hardware multiplier detected! Zero-DSP violated!"; exit 1; \
	else \
		echo ">>> [YOSYS] PASS — FULL SoC 100% Synthesizable & GUARANTEED 0 HARDWARE MULTIPLIERS (0 DSP)!"; \
	fi

clean:
	rm -rf $(BUILD_DIR)

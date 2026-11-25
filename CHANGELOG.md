# CHANGELOG — CHRONOS-PULP Log-SSM Edge AI Accelerator SoC

Tài liệu quản lý lịch sử phát triển và đóng gói phiên bản theo chuẩn công nghiệp (**Semantic Versioning** & **Conventional Commits**).

---

## [`v2.0.0-q1-release`] — 2026-11-25
### Added
- **Bộ công cụ Đánh giá & Khảo sát Không gian Thiết kế cấp Q1 (`scripts/q1_dse_ppa_benchmark.py` & `make benchmark_q1`):**
  - Kiểm chứng giải tích **Định lý 1 (Analytical Drift Error Bound)** đối chiếu với quỹ đạo mô phỏng 1.000 bước thời gian: chứng minh cắt cụt Mitchell bị khuếch đại độ chệch tĩnh $\frac{1}{1-|\bar{A}|}$, trong khi `0.5-ULP + 1st-Order ΔΣ` giảm **99,4% sai số RMSE** với độ chệch trung bình tiến về `0.000000`.
  - **Khảo sát đường biên Pareto (Pareto DSE)** cho độ rộng phần lẻ bảng tra $M \in \{3, 4, 5, \mathbf{6}, 7, 8\}$ bit: chứng minh $M=6$ bit ($64$ mục) nằm tại điểm "khuỷu tay" tối ưu (Pareto Knee), khớp 100% với 1 phần tử `LUT6` trên mỗi bit đầu ra của FPGA.
  - **Tổng hợp cổng logic đối chứng trực tiếp bằng Yosys (`Gate-Level PPA Comparison`):** So sánh trực tiếp giữa lõi **Linear INT16/8 Mamba + Exp/SiLU LUT**, **CHRONOS v1.0.0-baseline**, và **CHRONOS v2.0.0-q1-release**.

---

## [`v1.3.0-coalesced-skip-multilayer`] — 2026-11-25
### Added
- **Cơ chế Gộp bước suy giảm thời gian chống "Mù thời gian khoảng RR" trên sóng ECG (`Coalesced Multi-Step Log-ZOH Decay`):**
  - Bổ sung bộ đếm số khung tĩnh liên tiếp `silent_run_cnt_q` và bảng tra `coalesce_log_q3_6` ($\log_2(1 + K_{\text{skip}})$) trong `chronos_stream_agu.sv` và `chronos_log_zoh_unit.sv` (kích hoạt qua cờ `CSR_CTRL[8]`).
  - Khi bỏ qua $K_{\text{skip}}$ khung tín hiệu phẳng liên tiếp, phần cứng vẫn tắt 100% chu kỳ đọc/ghi SRAM trong suốt $K_{\text{skip}}$ bước đó, nhưng ngay tại bước tích cực tiếp theo chỉ tốn **1 chu kỳ duy nhất** để cộng bù $\log_2(1 + K_{\text{skip}})$ vào số mũ suy giảm ($\bar{A}^{1+K_{\text{skip}}} = \exp(-(1+K_{\text{skip}})\Delta|A|)$), bảo toàn chính xác khoảng cách thời gian nhịp tim RR (sai số $< 0,0026$ tại $K_{\text{skip}}=3$ trong `TEST 3c`).
- **Suy luận Mamba Đa tầng Ghép kênh theo Thời gian (`Time-Multiplexed Multi-Layer Mamba`):**
  - Mở rộng `chronos_csr_if.sv` (`CSR_CTRL[11:10] = cfg_num_layers`) và máy trạng thái `chronos_stream_agu.sv` (`layer_q`) cho phép chạy tuần tự nhiều tầng Mamba trên cùng diện tích lõi `Engine A` và `Engine B`.

---

## [`v1.2.0-arch-area-opt`] — 2026-11-25
### Changed
- **Tối ưu diện tích `Engine B` (`rtl/engine/chronos_ssm_core_1lane.sv`):**
  - Chia sẻ trực tiếp kết quả mã hóa logarit `delta_log_q3_6` từ khối `u_log_zoh` cho nhánh nhân 3 biến $\Delta_d \cdot B_n \cdot x_d$ ở Stage 1 (**loại bỏ hoàn toàn khối `u_lod_delta` dư thừa**).
  - Truyền kết quả mã hóa `{x_sign, x_zero, x_log_q3_6}` từ Stage 1 (`u_lod_x`) qua thanh ghi pipeline `S1/S2` và `S2/S3` sang Stage 3 cho nhánh kết nối tắt $D_d \cdot x_d$ (**loại bỏ hoàn toàn khối `u_lod_x_skip` dư thừa** ở Stage 3).
  - Giảm tổng số cổng logic của `Engine B` từ `7.050` cells xuống `6.826` cells ngay cả khi đã tích hợp thêm bộ hồi tiếp $\Delta\Sigma$.
- **Chuẩn hóa Wrapper Khóa xung nhịp Đa đích (`rtl/arith/chronos_icg.sv`):**
  - Hỗ trợ biên dịch đa đích cho **ASIC SkyWater 130nm** (`sky130_fd_sc_hd__dlclkp_1`) và **FPGA Xilinx** (`BUFGCE`).

---

## [`v1.1.0-arith-q1`] — 2026-11-25
### Changed
- **Hợp nhất hằng số $\log_2(\log_2 e)$ trong miền Logarit tại khối `Log-ZOH` (`rtl/arith/chronos_log_zoh_unit.sv`):**
  - Thay vì giải mã `Antilog16` rồi mới nhân $\log_2(e) \approx 1,442695$ bằng cây cộng dịch bit CSD 5 tầng rộng 26-bit trên đường găng của Stage 1, phép nhân $\log_2(e)$ được hợp nhất trực tiếp vào tổng số mũ trước bộ giải mã Antilog thông qua hằng số tĩnh $\text{round}(\log_2(\log_2 e)\times 64) = +34$ (`ZOH_EXP_OFFSET = -256 + 34 = -222`).
  - **Kết quả đo Yosys:**
    - Độ sâu đường găng (`Longest Topological Path`) của `chronos_log_zoh_unit` giảm từ **71 tầng cổng xuống 39 tầng cổng (giảm 45,1% Delay)**.
    - Số lượng tế bào logic nội bộ của `chronos_log_zoh_unit` giảm từ **732 cells xuống 235 cells (giảm 67,9% Area nội bộ, giảm 38,6% tổng Area toàn khối)**.
    - Sai số tuyệt đối trung bình (MAE) của hàm mũ $\exp(-\Delta|A|)$ đạt `0,0014` (MaxAbsErr = `0,0053`).
- **Tích hợp bộ điều chế hồi tiếp phần dư trạng thái bậc 1 (`1st-Order ΔΣ Error-Feedback`) trong `Engine B` Stage 2:**
  - Giữ lại phần dư dưới 1 LSB (`sub-LSB residual`) của trạng thái suy giảm và nhánh đầu vào trên 8 thanh ghi trạng thái `ds_err_n_q[0..7]`, tạo hình phổ nhiễu lượng tử $H_e(z) = \frac{1-z^{-1}}{1-\bar{A}z^{-1}}$, đưa độ lệch trôi trung bình qua 1.000 bước (`4.000` lượt cập nhật) xuống **`-0,000014`**.

---

## [`v1.0.0-baseline`] — 2026-11-25
### Added
- Kiến trúc cơ sở 28 module RTL SystemVerilog của hệ thống **CHRONOS-PULP** (Lõi số học Log8/LOD16/Antilog16 làm tròn 0.5-ULP, CSD Log-ZOH, 16-Seg Symmetric PWL, 4-Bank TCDM với Bank 1 True Dual-Port 0-Bubble & 1-Cycle Virtual Reset, Engine A 1x4 SIMD Shared-LOD, Engine B 3-Stage Pipeline, Stream AGU 4+1 Overlap, Dual-Granularity $\Delta$-Skipping, Host RISC-V RV32I + DMA + UART + 4-Domain PMU) cùng 6 testbench tự kiểm tra và hồ sơ báo cáo LaTeX/Word.

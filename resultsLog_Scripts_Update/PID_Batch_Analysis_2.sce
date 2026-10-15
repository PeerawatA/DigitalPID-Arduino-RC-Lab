// =============================================================================
// Script Name   : PID_Batch_Analysis.sce
// Description   : Batch analysis of all experiment logs in one folder
//                 (2 PID algorithms x 2 sampling architectures x 2 setpoints,
//                  repeated runs + summary-only runs).
// Related Paper : "Experimental Comparison of Digital PID Algorithms and
//                 Sampling Architectures on Arduino-Based Second-Order RC
//                 Control"
// Environment   : Scilab 6.x / 2023 / 2024 / 2025
// Firmware      : logs from PID_2order_RCRC_..._v3_4_TIMING.ino (v3.x)
//
// INPUT FILES (in data_dir), names like
//     SIMPLE_PAPER_ref0.9_N1.txt      (LOG_STREAM run, replicate 1)
//     TUSTIN_MY_TIMER_ref0.95_N3.txt  (LOG_STREAM run, replicate 3)
//     SIMPLE_MY_TIMER_ref0.9_SUM.txt  (LOG_SUMMARY run, timing only)
//
// OUTPUT FILES (written to data_dir)
//     PID_batch_results.csv            one row per file
//     PID_batch_condition_summary.csv  mean and SD per condition (N runs)
//
// DEFINITIONS (identical to PID_Experimental_Data_Analysis_v2.sce)
//     Ts        = mean(diff(t))  (period between logged rows, ~10.24 ms)
//     RMS       = sqrt(mean(e^2))
//     IAE       = Ts * sum(|e|)
//     ISE       = Ts * sum(e^2)
//     ITAE      = Ts * sum(t .* |e|)
//     y_ss      = mean of the last 101 samples
//     Tr        = t(y >= 0.9 y_ss) - t(y >= 0.1 y_ss)   (first crossings)
//     Ts_robust = last time a centred moving average of y (0.5 s window,
//                 interior samples only) leaves the +/-2% band of y_ss
//     Ts_strict = last time the raw y leaves the +/-2% band (reported only
//                 for completeness; very sensitive to single noisy samples)
//     Timing (J, L, Texec) and y_ss_std_lsb are read from the "#" summary
//     printed by the firmware (computed from every ISR cycle).
// =============================================================================

// --------------------------------------------------------------------
// 1. USER CONFIGURATION
// --------------------------------------------------------------------
// Folder with the .txt logs. Default: the folder this script is saved in
// (put the script next to the logs and run it with exec / SciNotes "Execute").
// Or set it explicitly, e.g. data_dir = "D:\VRU\...\RCRC_Arduino_results_record";
try
    data_dir = get_absolute_file_path("PID_Batch_Analysis.sce");
catch
    data_dir = pwd();
end
if part(data_dir, length(data_dir)) == filesep() then
    data_dir = part(data_dir, 1:length(data_dir) - 1);
end
settle_band     = 0.02;      // +/-2% settling band
settle_smooth_s = 0.50;      // moving-average window for Ts_robust (s)
make_figures    = %t;        // draw Figures A-D
export_figures  = %t;        // save them as PNG and PDF into data_dir
Ts_row_expected = 10 * 1.024e-3;   // SERIAL_DECIMATION x hardware ISR period

// --------------------------------------------------------------------
// 2. FUNCTIONS
// --------------------------------------------------------------------
function [D, skeys, svals, ndrop, JH, TH] = pid_load(fname)
    // Returns data rows [t x y e u], summary keys/values and dropped-row count
    L = mgetl(fname);
    L = stripblanks(L);
    L = L(L <> "");
    hdr = grep(L, "PID Architecture Test");
    if hdr <> [] then
        L = L(max(hdr):$);
    end
    f1 = part(L, 1);
    isnum = (f1 == "-") | (f1 == ".");
    for dch = ["0" "1" "2" "3" "4" "5" "6" "7" "8" "9"]
        isnum = isnum | (f1 == dch);
    end
    numL = L(isnum);
    sumL = L(f1 == "#");

    // keep only rows with exactly 5 fields
    // number of fields = number of commas + 1 (vectorised, fast)
    nf = length(numL) - length(strsubst(numL, ",", "")) + 1;
    ndrop = sum(nf <> 5);
    numL = numL(nf == 5);
    if numL <> [] then
        D = csvTextScan(numL, ",");
        bad = find(or(isnan(D), "c"));
        ndrop = ndrop + size(bad, "*");
        D(bad, :) = [];
    else
        D = [];
    end

    skeys = []; svals = []; JH = []; TH = [];
    for k = 1:size(sumL, "*")
        s = stripblanks(part(sumL(k), 2:length(sumL(k))));
        f = strsplit(s, ",");
        if size(f, "*") == 2 then
            v = strtod(f(2));
            if ~isnan(v) then
                skeys = [skeys; stripblanks(f(1))];
                svals = [svals; v];
            end
        elseif size(f, "*") == 3 then
            // histogram lines: [bin lower edge, count]
            if f(1) == "J_hist" then
                JH = [JH; strtod(f(2)) strtod(f(3))];
            elseif f(1) == "Texec_hist" then
                TH = [TH; strtod(f(2)) strtod(f(3))];
            end
        end
    end
endfunction

function v = getkey(skeys, svals, key)
    v = %nan;
    if skeys <> [] then
        k = find(skeys == key);
        if k <> [] then
            v = svals(k(1));
        end
    end
endfunction

function R = pid_indices(D, band_frac, smooth_s, Ts_exp)
    // R = [RMS IAE ISE ITAE Tr Ts_robust Ts_strict y_ss Mp ss_e_mean gaps Ts_row]
    t = D(:, 1); y = D(:, 3); e = D(:, 4);
    Ts = mean(diff(t));
    RMS  = sqrt(mean(e.^2));
    IAE  = Ts * sum(abs(e));
    ISE  = Ts * sum(e.^2);
    ITAE = Ts * sum(t .* abs(e));

    ny = size(y, "*");
    Ntail = min(100, ny - 1);
    y_ss = mean(y($-Ntail:$));

    i10 = find(y >= 0.1 * y_ss, 1);
    i90 = find(y >= 0.9 * y_ss, 1);
    if i10 <> [] & i90 <> [] then
        Tr = t(i90) - t(i10);
    else
        Tr = %nan;
    end

    band = band_frac * y_ss;
    io = find(abs(y - y_ss) > band);
    if io <> [] then Ts_strict = t(max(io)); else Ts_strict = 0; end

    N = round(smooth_s / Ts);
    N = max(1, 2 * floor(N / 2) + 1);
    h = (N - 1) / 2;
    i_in = (h + 1):(ny - h);
    c = cumsum([0; y]);
    y_f = (c(i_in + h + 1) - c(i_in - h)) / N;
    io = find(abs(y_f - y_ss) > band);
    if io <> [] then Ts_robust = t(i_in(max(io))); else Ts_robust = 0; end

    Mp = max(0, (max(y) - y_ss) / y_ss * 100);
    ss = t > 20;
    if ~or(ss) then ss = t >= t(1) + 0.8 * (t($) - t(1)); end
    ss_e_mean = mean(abs(e(ss)));
    gaps = size(find(diff(t) > 1.5 * Ts_exp), "*");

    R = [RMS IAE ISE ITAE Tr Ts_robust Ts_strict y_ss Mp ss_e_mean gaps Ts];
endfunction

// --------------------------------------------------------------------
// 3. FIND FILES
// --------------------------------------------------------------------
files = gsort(findfiles(data_dir, "*.txt"), "g", "i");
keep = [];
for k = 1:size(files, "*")
    fn = files(k);
    if (part(fn, 1:7) == "SIMPLE_" | part(fn, 1:7) == "TUSTIN_") & grep(fn, "_ref") <> [] then
        keep = [keep; fn];
    end
end
files = keep;
if files == [] then
    error("No log files (SIMPLE_*/TUSTIN_* ... _ref...) found in " + data_dir);
end
mprintf("\nFolder : %s\nFiles  : %d\n", data_dir, size(files, "*"));

// --------------------------------------------------------------------
// 4. PROCESS EVERY FILE
// --------------------------------------------------------------------
idx_names = ["RMS" "IAE" "ISE" "ITAE" "Tr" "Ts_robust" "Ts_strict" "y_ss" "Mp" "ss_e_mean" "gaps" "Ts_row"];
tim_names = ["J_std_us" "J_rms_us" "J_nonzero_frac" "J_min_us" "J_max_us" ..
             "L_mean_us" "L_min_us" "L_max_us" ..
             "Texec_mean_us" "Texec_std_us" "Texec_min_us" "Texec_max_us" ..
             "y_ss_std_lsb" "y_ss_min_lsb" "y_ss_max_lsb" "cycles_J"];

nfile = size(files, "*");
ALG = emptystr(nfile, 1); ARCH = ALG; REF = ALG; REP = ALG;
IDX = %nan * ones(nfile, size(idx_names, "*"));
TIM = %nan * ones(nfile, size(tim_names, "*"));
NROW = zeros(nfile, 1); NDROP = zeros(nfile, 1);
TS = list(); JHL = list(); THL = list();

for k = 1:nfile
    fn = files(k);
    base = strsubst(fn, ".txt", "");
    tok = strsplit(base, "_");
    ALG(k)  = tok(1);
    if grep(base, "TIMER") <> [] then ARCH(k) = "TIMER"; else ARCH(k) = "PAPER"; end
    r = tok(grep(tok, "ref"));
    REF(k)  = strsubst(r(1), "ref", "");
    REP(k)  = tok($);

    [D, sk, sv, nd, JH, TH] = pid_load(data_dir + filesep() + fn);
    // keep what the figures need: time series of replicate N1, histograms of all runs
    if REP(k) == "N1" then
        TS(k) = D(:, [1 3]);
    else
        TS(k) = [];
    end
    JHL(k) = JH;
    THL(k) = TH;
    NROW(k) = size(D, 1);
    NDROP(k) = nd;
    if size(D, 1) > 100 then
        IDX(k, :) = pid_indices(D, settle_band, settle_smooth_s, Ts_row_expected);
    end
    for j = 1:size(tim_names, "*")
        TIM(k, j) = getkey(sk, sv, tim_names(j));
    end
    mprintf("  %-34s rows %5d  dropped %d  gaps %s  RMS %s  y_ss_std %5.2f LSB\n", fn, NROW(k), NDROP(k), ..
            string(IDX(k, 11)), string(IDX(k, 1)), TIM(k, 13));
    if NROW(k) > 100 then
        if abs(IDX(k, 12) - Ts_row_expected) / Ts_row_expected > 0.05 then
            mprintf("    *** WARNING: row period %.5f s does not match %.5f s\n", IDX(k, 12), Ts_row_expected);
        end
    end
end

// --------------------------------------------------------------------
// 5. WRITE PER-FILE CSV
// --------------------------------------------------------------------
function s = num2s(v)
    if isnan(v) then
        s = "";
    else
        s = msprintf("%.6g", v);
    end
endfunction

hdr = strcat(["file" "alg" "arch" "ref" "rep" "rows" "dropped" idx_names tim_names], ",");
out = hdr;
for k = 1:nfile
    vals = [NROW(k) NDROP(k) IDX(k, :) TIM(k, :)];
    sv = emptystr(1, size(vals, "*"));
    for j = 1:size(vals, "*")
        sv(j) = num2s(vals(j));
    end
    out = [out; strcat([files(k) ALG(k) ARCH(k) REF(k) REP(k) sv], ",")];
end
f_res = data_dir + filesep() + "PID_batch_results.csv";
mputl(out, f_res);
mprintf("\nWritten: %s\n", f_res);

// --------------------------------------------------------------------
// 6. CONDITION SUMMARY (mean and SD over replicate runs N1..Nn)
// --------------------------------------------------------------------
sum_names = ["RMS" "IAE" "ISE" "ITAE" "Tr" "Ts_robust" "y_ss_std_lsb" ..
             "J_std_us" "J_nonzero_frac" "L_mean_us" "Texec_mean_us" "Texec_std_us"];
VAL = [IDX TIM];
ALLN = [idx_names tim_names];

out2 = "ref,alg,arch,n";
for j = 1:size(sum_names, "*")
    out2 = out2 + "," + sum_names(j) + "_mean," + sum_names(j) + "_sd";
end

mprintf("\n===== CONDITION MEANS (replicate runs only) =====\n");
mprintf("%-5s %-7s %-6s %2s %9s %8s %8s %8s %7s %7s %8s %8s %8s\n", ..
        "ref", "alg", "arch", "n", "RMS", "IAE", "ISE", "ITAE", "Tr", "Ts_rob", "yss_std", "J_std", "Texec");
refs = unique(REF); algs = unique(ALG); archs = unique(ARCH);
for a = 1:size(refs, "*")
    for b = 1:size(algs, "*")
        for c = 1:size(archs, "*")
            sel = find(REF == refs(a) & ALG == algs(b) & ARCH == archs(c) & REP <> "SUM");
            if sel == [] then continue; end
            line = strcat([refs(a) algs(b) archs(c) string(size(sel, "*"))], ",");
            m = [];
            for j = 1:size(sum_names, "*")
                col = find(ALLN == sum_names(j));
                v = VAL(sel, col);
                mj = mean(v);
                if size(v, "*") > 1 then sj = stdev(v); else sj = %nan; end
                line = line + "," + num2s(mj) + "," + num2s(sj);
                m = [m mj];
            end
            out2 = [out2; line];
            mprintf("%-5s %-7s %-6s %2d %9.5f %8.4f %8.4f %8.3f %7.3f %7.2f %8.2f %8.3f %8.2f\n", ..
                    refs(a), algs(b), archs(c), size(sel, "*"), m(1), m(2), m(3), m(4), m(5), m(6), m(7), m(8), m(11));
        end
    end
end
f_sum = data_dir + filesep() + "PID_batch_condition_summary.csv";
mputl(out2, f_sum);
mprintf("Written: %s\n", f_sum);

// --------------------------------------------------------------------
// 7. SUMMARY-ONLY RUNS (no serial streaming during the run)
// --------------------------------------------------------------------
sel = find(REP == "SUM");
if sel <> [] then
    mprintf("\n===== SUMMARY-ONLY RUNS (timing without UART traffic) =====\n");
    mprintf("%-34s %8s %10s %8s %8s %9s %9s\n", "file", "J_std", "J_nonzero", "J_min", "L_mean", "Texec", "Texec_sd");
    for k = sel'
        mprintf("%-34s %8.3f %10.5f %8.1f %8.2f %9.2f %9.2f\n", files(k), TIM(k, 1), TIM(k, 3), TIM(k, 4), ..
                TIM(k, 6), TIM(k, 9), TIM(k, 10));
    end
end

// --------------------------------------------------------------------
// 8. FIGURES A-D  (English labels, journal style)
//    Colour = sampling architecture (blue ADC-based, orange Timer-based)
//    Line style / marker = algorithm (solid & filled = Tustin,
//                                     dashed & open  = Simple)
// --------------------------------------------------------------------
function c = rgb(h)
    // "#rrggbb" -> colour index in the current colormap
    c = color(hex2dec(part(h, 2:3)), hex2dec(part(h, 4:5)), hex2dec(part(h, 6:7)));
endfunction

function k = find_run(ALG, ARCH, REF, REP, a, b, r, n)
    k = find(ALG == a & ARCH == b & REF == r & REP == n);
endfunction

function H = pool_hist(HL, sel)
    // sum histogram counts over the files in sel -> [bin, percent]
    H = [];
    for k = sel(:)'
        h = HL(k);
        if h == [] then continue; end
        if H == [] then H = h; else H(:, 2) = H(:, 2) + h(:, 2); end
    end
    if H <> [] then H(:, 2) = 100 * H(:, 2) / sum(H(:, 2)); end
endfunction

function draw_bars(x, h, w, col, ybase)
    // filled rectangles from ybase to h (works on log axes)
    for i = 1:size(x, "*")
        if h(i) > ybase then
            xfpoly([x(i) - w/2, x(i) + w/2, x(i) + w/2, x(i) - w/2], [ybase, ybase, h(i), h(i)]);
            e = gce(); e.fill_mode = "on"; e.background = col; e.foreground = col;
        end
    end
endfunction

function style_axes(a)
    a.axes_visible = ["on" "on" "on"]; a.tight_limits = "on";
    a.font_size = 2; a.grid = [-1 color(228, 227, 223)]; a.grid_style = [1 1];
    a.box = "off"; a.foreground = color(82, 81, 78); a.font_color = color(82, 81, 78);
    a.title.font_size = 2; a.x_label.font_size = 2; a.y_label.font_size = 2;
endfunction

function legend_row(bounds, kinds, cols, styles, labels)
    // Draws a one-row legend in its own invisible axes.
    //   kinds: "line" | "patch" | "dot" | "square";  styles: line_style for "line"
    al = newaxes(); al.axes_bounds = bounds; al.data_bounds = [0 0; size(labels, "*") 1];
    n = size(labels, "*");
    for c = 1:n
        x0 = c - 1 + 0.05;
        select kinds(c)
        case "line" then
            plot([x0 x0 + 0.18], [0.5 0.5]); e = gce(); e = e.children(1);
            e.foreground = cols(c); e.line_style = styles(c); e.thickness = 2;
        case "patch" then
            xfpoly([x0 x0 + 0.12 x0 + 0.12 x0], [0.3 0.3 0.7 0.7]); e = gce();
            e.fill_mode = "on"; e.background = cols(c); e.foreground = cols(c);
        else
            plot(x0 + 0.08, 0.5); e = gce(); e = e.children(1); e.line_mode = "off"; e.mark_mode = "on";
            e.mark_size_unit = "point"; e.mark_size = 7; e.mark_foreground = cols(c);
            if kinds(c) == "dot" then e.mark_style = 9; e.mark_background = cols(c);
            else e.mark_style = 11; e.mark_background = color(255, 255, 255); end
        end
        xstring(x0 + 0.22, 0.3, labels(c)); t = gce(); t.font_size = 2; t.font_foreground = color(11, 11, 11);
    end
    al.data_bounds = [0 0; n 1]; al.tight_limits = "on";
    al.axes_visible = ["off" "off" "off"]; al.box = "off"; al.filled = "off";
endfunction

function save_fig(f, name, dir_out, do_export)
    if do_export then
        xs2png(f, dir_out + filesep() + name + ".png");
        xs2pdf(f, dir_out + filesep() + name + ".pdf");
        mprintf("Written: %s(.png/.pdf)\n", name);
    end
endfunction

if make_figures then
    // NOTE: colour indices belong to ONE figure's colormap, so they are
    // re-created right after each scf() below (otherwise colours turn grey)
    archs_ = ["PAPER" "TIMER"]; algs_ = ["TUSTIN" "SIMPLE"];
    archlab = ["ADC-based" "Timer-based"]; alglab = ["Tustin" "Simple"];

    lstyle  = [1 2];      // 1 = solid (Tustin), 2 = dashed (Simple)

    // ---------------- Figure A: time response ----------------
    fA = scf(101); clf(fA); fA.figure_size = [1500 720]; fA.background = color(255, 255, 255);
    CB = rgb("#2a78d6"); CO = rgb("#eb6834"); CK = color(11, 11, 11); CG = color(82, 81, 78);
    archcol = [CB CO];
    refs_ = ["0.9" "0.95"]; spv = [921/1023 973/1023];
    for p = 1:2
        a = newaxes(); a.axes_bounds = [(p - 1) * 0.5, 0, 0.5, 0.88];
        a.margins = [0.14 0.04 0.10 0.16];
        leg_h = []; leg_s = [];
        for g = 1:2
            for b = 1:2
                k = find_run(ALG, ARCH, REF, REP, algs_(g), archs_(b), refs_(p), "N1");
                if k == [] then continue; end
                d = TS(k);
                plot(d(:, 1), d(:, 2));
                e = gce(); e = e.children(1); e.foreground = archcol(b); e.line_style = lstyle(g); e.thickness = 1;
                leg_h = [leg_h e]; leg_s = [leg_s alglab(g) + " & " + archlab(b)];
            end
        end
        plot([0 100], spv(p) * [1 1]); e = gce(); e = e.children(1); e.foreground = CK; e.line_style = 3;
        a.data_bounds = [0 0; 100 1.05]; style_axes(a);
        xlabel("Time (s)"); ylabel("Output y (normalized)"); title("Setpoint " + refs_(p));
        // inset: steady state 40-45 s
        ai = newaxes(); ai.axes_bounds = [(p - 1) * 0.5 + 0.24, 0.40, 0.24, 0.30];
        ai.margins = [0.16 0.04 0.14 0.14];
        for g = 1:2
            for b = 1:2
                k = find_run(ALG, ARCH, REF, REP, algs_(g), archs_(b), refs_(p), "N1");
                if k == [] then continue; end
                d = TS(k); m = d(:, 1) >= 40 & d(:, 1) <= 45;
                plot(d(m, 1), d(m, 2));
                e = gce(); e = e.children(1); e.foreground = archcol(b); e.line_style = lstyle(g);
            end
        end
        ai.data_bounds = [40 spv(p) - 0.05; 45 spv(p) + 0.025]; style_axes(ai); ai.font_size = 1;
        title("Steady state, 40-45 s"); ai.title.font_size = 1;
    end
    legend_row([0.05 0.88 0.9 0.1], ["line" "line" "line" "line"], [CB CO CB CO], [1 1 2 2], ..
               ["Tustin & ADC-based" "Tustin & Timer-based" "Simple & ADC-based" "Simple & Timer-based"]);
    save_fig(fA, "Fig_A_time_response", data_dir, export_figures);

    // ---------------- Figure B: |J| histogram (setpoint 0.9) ----------------
    fB = scf(102); clf(fB); fB.figure_size = [1500 640]; fB.background = color(255, 255, 255);
    CB = rgb("#2a78d6"); CO = rgb("#eb6834"); CK = color(11, 11, 11); CG = color(82, 81, 78);
    archcol = [CB CO];
    algB = ["SIMPLE" "TUSTIN"]; algBl = ["Simple" "Tustin"];
    for p = 1:2
        a = newaxes(); a.axes_bounds = [(p - 1) * 0.5, 0, 0.5, 0.88];
        a.margins = [0.14 0.04 0.10 0.16];
        a.data_bounds = [-0.4 0.005; 8.4 150];
        a.log_flags = "nln";
        for b = 1:2
            sel = find(ALG == algB(p) & ARCH == archs_(b) & REF == "0.9" & REP <> "SUM");
            H = pool_hist(JHL, sel);
            if H <> [] then
                draw_bars(H(:, 1) + (b - 1.5) * 0.2, H(:, 2), 0.18, archcol(b), 0.005);
            end
        end
        style_axes(a); a.data_bounds = [-0.4 0.005; 8.4 150]; a.log_flags = "nln";
        a.x_ticks = tlist(["ticks", "locations", "labels"], 0:8, ["0" "1" "2" "3" "4" "5" "6" "7" ">=8"]);
        xlabel("|J| (us)"); ylabel("Share of sampling cycles (%)");
        title(algBl(p) + " PID, setpoint 0.9");
    end
    legend_row([0.3 0.88 0.4 0.1], ["patch" "patch"], [CB CO], [1 1], ["ADC-based" "Timer-based"]);
    save_fig(fB, "Fig_B_jitter_histogram", data_dir, export_figures);

    // ---------------- Figure C: Texec histogram (setpoint 0.9) ----------------
    fC = scf(103); clf(fC); fC.figure_size = [1000 640]; fC.background = color(255, 255, 255);
    CB = rgb("#2a78d6"); CO = rgb("#eb6834"); CK = color(11, 11, 11); CG = color(82, 81, 78);
    archcol = [CB CO];
    a = newaxes(); a.axes_bounds = [0 0 1 0.88]; a.margins = [0.12 0.04 0.06 0.16];
    colC = [color(170, 169, 165) CG];
    for g = 1:2
        sel = find(ALG == algB(g) & REF == "0.9" & REP <> "SUM");
        H = pool_hist(THL, sel);
        if H <> [] then
            ctr = H(:, 1) + 2.5; ctr(H(:, 1) == 0) = 47.5;
            draw_bars(ctr + (g - 1.5) * 2, H(:, 2), 1.8, colC(g), 0);
        end
    end
    style_axes(a); a.data_bounds = [45 0; 125 70];
    xlabel("Execution time of computePID(), Texec (us)"); ylabel("Share of sampling cycles (%)");
    legend_row([0.15 0.88 0.6 0.1], ["patch" "patch"], colC, [1 1], ["Simple PID" "Tustin PID"]);
    save_fig(fC, "Fig_C_execution_time_histogram", data_dir, export_figures);

    // ---------------- Figure D: RMS, IAE, ISE (mean +/- SD, n = 3) ----------------
    fD = scf(104); clf(fD); fD.figure_size = [1500 620]; fD.background = color(255, 255, 255);
    CB = rgb("#2a78d6"); CO = rgb("#eb6834"); CK = color(11, 11, 11); CG = color(82, 81, 78);
    archcol = [CB CO];
    mnames = ["RMS" "IAE" "ISE"]; mtitles = ["RMS Error" "IAE" "ISE"];
    combos_g = ["TUSTIN" "TUSTIN" "SIMPLE" "SIMPLE"]; combos_b = ["PAPER" "TIMER" "PAPER" "TIMER"];
    for p = 1:3
        a = newaxes(); a.axes_bounds = [(p - 1) / 3, 0, 1 / 3, 0.88];
        a.margins = [0.20 0.04 0.10 0.14];
        col = find(idx_names == mnames(p));
        ymin = %inf; ymax = -%inf;
        for r = 1:2
            for c = 1:4
                sel = find(ALG == combos_g(c) & ARCH == combos_b(c) & REF == refs_(r) & REP <> "SUM");
                v = IDX(sel, col); x = (r - 1) + (c - 2.5) * 0.17;
                mu = mean(v); sd = stdev(v);
                ymin = min(ymin, min(v), mu - sd); ymax = max(ymax, max(v), mu + sd);
                bcol = archcol(find(archs_ == combos_b(c)));
                plot([x x], [mu - sd, mu + sd]); e = gce(); e = e.children(1); e.foreground = bcol; e.thickness = 2;
                plot(x, mu); e = gce(); e = e.children(1); e.line_mode = "off"; e.mark_mode = "on";
                e.mark_size = 7; e.mark_foreground = bcol; e.mark_size_unit = "point";
                if combos_g(c) == "TUSTIN" then
                    e.mark_style = 9; e.mark_background = bcol;      // filled circle
                else
                    e.mark_style = 11; e.mark_background = color(255, 255, 255);   // open square
                end
                plot((x + 0.06) * ones(v), v); e = gce(); e = e.children(1); e.line_mode = "off";
                e.mark_mode = "on"; e.mark_style = 0; e.mark_size = 3; e.mark_foreground = CG;
            end
        end
        pad = 0.08 * (ymax - ymin);
        style_axes(a);
        a.data_bounds = [-0.5 ymin - pad; 1.5 ymax + pad];
        a.x_ticks = tlist(["ticks", "locations", "labels"], [0 1], ["Setpoint 0.9" "Setpoint 0.95"]);
        title(mtitles(p));
    end
    legend_row([0.05 0.88 0.9 0.1], ["dot" "dot" "square" "square"], [CB CO CB CO], [1 1 1 1], ..
               ["Tustin & ADC-based" "Tustin & Timer-based" "Simple & ADC-based" "Simple & Timer-based"]);
    save_fig(fD, "Fig_D_performance_indices", data_dir, export_figures);
end

mprintf("\nDone.\n");

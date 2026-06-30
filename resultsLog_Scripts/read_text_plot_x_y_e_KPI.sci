// =============================================================================
// Script Name   : PID_Experimental_Data_Analysis.sce
// Description   : Data analysis and visualization script for evaluating the 
//                 performance of digital PID controllers under varying 
//                 computational forms and sampling architectures.
// Related Paper : "Comparison of Digital PID Computational Forms and Sampling 
//                 Architectures on a Low-Cost Platform: A Case Study in a 
//                 Second-Order RC Circuit"
// Environment   : Scilab 2025.0 (or compatible versions)
//
// INPUT:
//   A comma-separated values (CSV) or TXT file containing experimental data 
//   obtained from the microcontroller. The file must contain exactly 5 columns 
//   without text headers:
//     - Column 1: Time [t] (in seconds)
//     - Column 2: Reference Setpoint [x]
//     - Column 3: System Output / Plant Response [y]
//     - Column 4: Error [e] (where e = x - y)
//     - Column 5: Control Signal / Controller Effort [u]
//
// OUTPUT:
//   1. Graphical Plots:
//      - Figure 1: Transient response (Setpoint, Output, and Error)
//      - Figure 2: Error response and Control effort over time
//      - Figure 3: Clean Setpoint vs Output response
//   2. Console Output (Performance Metrics):
//      - Error Indices: RMS Error, IAE, ISE, ITAE
//      - Time-Domain Specs: Steady-state value, Rise time (tr), 
//        Settling time (ts), and Overshoot (Mp).
//
// USAGE:
//   Modify the 'filename' variable in the USER CONFIGURATION section below 
//   to target the desired experimental dataset before execution.
// =============================================================================

close;
clc;

// --------------------------------------------------------------------
// 1. USER CONFIGURATION
// --------------------------------------------------------------------
// Available datasets (Uncomment the desired case for analysis)

// Case 1: TUSTIN Algorithm & PAPER Architecture (Timer -> ADC -> ISR)
// filename = "PID_TUSTIN_PAPER_ref0.9.txt";
// filename = "PID_TUSTIN_PAPER_ref0.95.txt";

// Case 2: TUSTIN Algorithm & TIMER Architecture (Interrupt-Centric)
// filename = "PID_TUSTIN_TIMER_ref0.9.txt";
// filename = "PID_TUSTIN_TIMER_ref0.95.txt";

// Case 3: SIMPLE Algorithm & PAPER Architecture (Timer -> ADC -> ISR)
// filename = "PID_SIMPLE_PAPER_ref0.9.txt";
// filename = "PID_SIMPLE_PAPER_ref0.95.txt";

// Case 4: SIMPLE Algorithm & TIMER Architecture (Interrupt-Centric)
// filename = "PID_SIMPLE_TIMER_ref0.9.txt";
// filename = "PID_SIMPLE_TIMER_ref0.95.txt";

filename = "PID_SIMPLE_TIMER_ref0.95.txt";   // <-- Active File

delimiter = ","; // Delimiter used in the file ("," or ascii(9) for tab)

// --------------------------------------------------------------------
// 2. LOAD & VALIDATE DATA
// --------------------------------------------------------------------
data = csvRead(filename, delimiter);

// --- Sanity check ---
if size(data, 2) < 5 then
    error("Data format error: expected 5 columns [t x y e u]");
end

disp("Dataset Dimensions:");
disp(size(data));
disp("First 5 rows preview:");
disp(data(1:5,:));

// --------------------------------------------------------------------
// 3. ASSIGN VARIABLES
// --------------------------------------------------------------------
t = data(:,1);   // Time (s)
x = data(:,2);   // Setpoint (Reference)
y = data(:,3);   // Output (System Response)
e = data(:,4);   // Error
u = data(:,5);   // Control signal

// --------------------------------------------------------------------
// 4. SAMPLING INFORMATION
// --------------------------------------------------------------------
Ts = mean(diff(t)); // Average sampling period
T  = t($) - t(1);   // Total experiment duration

// ====================================================================
// FIGURE 1: TRANSIENT RESPONSE (SETPOINT, OUTPUT, ERROR)
// ====================================================================
scf(1); clf();
plot(t, x, 'b-', 'LineWidth', 2);
plot(t, y, 'r.', 'MarkerSize', 1);
plot(t, e, 'g-', 'LineWidth', 2);
ax = gca();
ax.grid = [1 1];
ax.font_size = 3;
xlabel("Time (s)");
ylabel("Amplitude (normalized ADC value)");
// title("Transient Response Analysis");
legend(["Setpoint", "Output", "Error"], "in_upper_right");

// Optional bounds adjustment
// emin = min(e) - 0.05; emax = max(e) + 0.05;
// ax.data_bounds = [min(t) emin ; max(t) emax];

// ====================================================================
// FIGURE 2: ERROR & CONTROL SIGNAL EFFORT
// ====================================================================
scf(2); clf();

// -------- Subplot 1: Error Response --------
subplot(2,1,1);
plot(t, e, 'g-', 'LineWidth', 2);
a = gca();
a.grid = [1 1];
a.font_size = 3;
ylabel("Error e(t)");
title("Error Response");
emin = min(e) - 0.05;
emax = max(e) + 0.05;
a.data_bounds = [min(t) emin ; max(t) emax];

// -------- Subplot 2: Control Effort --------
subplot(2,1,2);
plot(t, u, 'k-', 'LineWidth', 1);
a = gca();
a.grid = [1 1];
a.font_size = 3;
xlabel("Time (s)");
ylabel("Control signal u(t)");
title("Control Effort");
umin = min(u) - 0.05;
umax = max(u) + 0.05;
a.data_bounds = [min(t) umin ; max(t) umax];

// ====================================================================
// PERFORMANCE INDICES CALCULATION
// ====================================================================

// --- Root Mean Square (RMS) Error ---
RMS  = sqrt(mean(e.^2));

// --- Integral Indices ---
IAE  = Ts * sum(abs(e));        // Integral of Absolute Error
ISE  = Ts * sum(e.^2);          // Integral of Squared Error
ITAE = Ts * sum(t .* abs(e));   // Integral of Time-weighted Absolute Error

// --- Display Performance Indices ---
mprintf("\n===== CONTROL PERFORMANCE INDICES =====\n");
mprintf("Sampling time Ts = %.6f s\n", Ts);
mprintf("RMS Error        = %.6f\n", RMS);
mprintf("IAE              = %.6f\n", IAE);
mprintf("ISE              = %.6f\n", ISE);
mprintf("ITAE             = %.6f\n", ITAE);

// ====================================================================
// TIME-DOMAIN SPECIFICATIONS
// ====================================================================

// --- Steady-state value estimation ---
Ntail = min(100, length(y));
y_ss  = mean(y($-Ntail:$));

// --- Rise Time (10% -> 90%) ---
y10 = 0.1 * y_ss;
y90 = 0.9 * y_ss;
idx10 = find(y >= y10, 1);
idx90 = find(y >= y90, 1);
if idx10 <> [] & idx90 <> [] then
    tr = t(idx90) - t(idx10);
else
    tr = %nan;
end

// --- Settling Time (±2% band) ---
band = 0.02 * y_ss;
idx_settle = find(abs(y - y_ss) > band);
if idx_settle <> [] then
    ts = t(max(idx_settle));
else
    ts = 0;
end

// --- Maximum Overshoot ---
ymax = max(y);
Mp   = max(0, (ymax - y_ss) / y_ss * 100);
idx_or = find(y == ymax, 1);
tor = t(idx_or);

// --- Display Time-Domain Results ---
mprintf("\n===== TIME-DOMAIN PERFORMANCE =====\n");
mprintf("Steady-state value y_ss = %.6f\n", y_ss);
mprintf("Rise time tr            = %.4f s\n", tr);
mprintf("Settling time ts        = %.4f s (±2%%)\n", ts);
mprintf("Overshoot Mp            = %.2f %%\n", Mp);
mprintf("Time of overshoot       = %.4f s\n", tor);
mprintf("=======================================\n\n");

// ====================================================================
// FIGURE 3: SETPOINT VS OUTPUT (CLEAN VIEW)
// ====================================================================
scf(3); clf();
plot(t, x, 'b-', 'LineWidth', 2);
plot(t, y, 'r.', 'MarkerSize', 1);
ax = gca();
ax.grid = [1 1];
ax.font_size = 3;
xlabel("Time (s)");
ylabel("Amplitude");
title("Setpoint vs Output");
legend(["Setpoint", "Output"], "in_upper_right");

// ====================================================================
// END OF SCRIPT
// ====================================================================


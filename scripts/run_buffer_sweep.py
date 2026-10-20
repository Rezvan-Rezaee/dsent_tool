#!/usr/bin/env python3
"""
Reusable end-to-end DSENT buffer_characteristics sweep runner. Replaces
run_dsent_sweep.py: adds a --num-vcs sweep dimension, the Phase 5 activity
rates (--write-rate/--read-rate/--flit-toggle-rate), and automatic
register-count sanity checks against Becker (Stanford PhD thesis 2012,
Table 6.1, static management column) for every (depth, num-vcs) point.

Runs the dsent buffer_characteristics subcommand once per (depth, width,
num_vcs) point (isolating any single-config crash rather than aborting the
whole sweep), parses each JSON result, and writes buffer_metrics.json +
run_log.txt.

Pure external orchestration -- does not modify or rebuild DSENT source.

Exit code: 0 if sweep_valid, 1 otherwise.
"""
import argparse
import json
import math
import os
import subprocess
import sys
import tempfile
from datetime import datetime, timezone

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)

# Part F -- assumptions list, embedded verbatim into every run's meta.
ASSUMPTIONS = [
    "Static buffer management: capacity is evenly partitioned across VCs. "
    "Dynamic (linked-list / packet-table) management is not modelled and "
    "costs substantially more control logic.",

    "VC slice depth is assumed a power of two, so index wraparound is free "
    "and the RAM address is a pure concatenation of {vc_id, index}. "
    "Non-power-of-two slices would require a modulo comparator per VC.",

    "Route-computation results and assigned output VC are allocator state, "
    "not buffer state, and are excluded here to avoid double-counting "
    "against DSENT's existing allocator models.",

    "The input-side credit proxy is not modelled. It is a timing "
    "optimisation (a local copy of the output-side credit count) and would "
    "add ceil(log2(depth)) flops per input VC.",

    "Router.cc's own top-level ClockTree's area is excluded from its own "
    "SitePitch (one-pass), slightly underestimating wire length. This no "
    "longer applies to DFFRAM's row-enable path (Phase 4.1 replaced its "
    "BroadcastHTree with a plain buffer chain that has no SitePitch/wire "
    "term at all -- see the next assumption).",

    "Cell areas remain DSENT's analytic values, which are 25-50% below "
    "measured ASAP7 cells (DFF -25%, NAND2/NOR2 -50%, INV -33%). This is "
    "largely common-mode in a ratio but not in an absolute number.",

    "Credit round-trip latency is not modelled. A VC shallower than the "
    "round-trip will credit-stall; that is a throughput cost visible only "
    "in simulation, never in a static power or area figure.",

    "Closure Task Part B: DFFRAM's own inter-cell interconnect is entirely "
    "unmodelled (README bug #4). Every net created via createNet() inside "
    "DFFRAM.cc (CKGate0_Out, EnBufOut_*, NAND2_CKGate1_Outs*, "
    "CKGate1_Outs*, RowBufOut_*, Dec_Out*, DFF_Out*, Mux_Out) is a bare "
    "ElectricalNet with zero distributed cap/res -- only CellMacros.cc's "
    "intra-CELL M1 stub (:322-327, load-independent, baked into each "
    "cell's own cached self-capacitance) is modelled; nothing models the "
    "wire BETWEEN cells. Estimated magnitude (DSENT's own "
    "calculateWireCapacitance on the Local layer, Bulk22LVT.model, same "
    "formula the codebase already uses for BroadcastHTree/RepeatedLink, "
    "just not applied here): a single minimum-size gate input "
    "(NAND2/INV/BUF X1, A_Cap) is ~4.526e-16 F. One local-layer, "
    "one-gate-pitch (120nm) hop is ~2.049e-17 F, ~4.5% of that gate load; "
    "a more realistic several-cell hop (480nm, 4 gate pitches) is "
    "~8.195e-17 F, ~18% of that gate load -- both small and plausibly "
    "negligible for genuinely LOCAL point-to-point nets (the RowBuf chain, "
    "INV_CKGate1_i->DFF.CK). The ROW-ENABLE FANOUT net is a different "
    "story: it must physically span the full DFFRAM array (SitePitch = "
    "sqrt(array_area), which the old BroadcastHTree WAS sized against and "
    "the new EnBuf chain is NOT), hundreds of um at large N/B (e.g. ~495um "
    "at N=512,B=256) -- a Local-layer estimate over that distance would be "
    "~100x+ a single gate's input cap, i.e. NOT negligible, though a real "
    "layout would use a higher (Intermediate/Global) metal layer for a "
    "span that large, not Local. Net effect: replacing the H-tree with a "
    "plain buffer chain (Phase 4.1) correctly fixed a topology mismatch "
    "(NumberLevels tied to log2(N) rather than to real electrical fanout) "
    "but also removed the ONLY wire term the row-enable net had -- for "
    "large N/B this net is now modelled as having literally zero physical "
    "distance, which is itself an underestimate, just a much smaller and "
    "more defensible one than the 9-level H-tree it replaced. Not fixed "
    "here (no new modelling, per instructions) -- reported as a disclosed "
    "limitation.",

    "Final Task: the write/clock-enable path fixes (Phase 4.1-4.3) made "
    "t_write_clock_path 3-10% of a 1ns period at every swept point. The "
    "READ path is now t_critical's binding term at EVERY (depth, width) "
    "point in this sweep -- it was previously masked at small N by the "
    "(now-fixed) write path being even slower. Two read-path components: "
    "t_read_access (CK->DFF internal CK-to-Q->Mux data path->Out) and "
    "t_read_addr_path (RDAddr->Mux select distribution->Mux data path-> "
    "Out); t_critical is the max of both, plus t_write_data_setup/"
    "t_write_clock_path. Multiplexer.cc's select-bit distribution had the "
    "SAME untapered-fanout defect the enable/row-clock chains had: exactly "
    "2 fixed-size INVs per select bit at the top level, broadcasting "
    "unbuffered (plain net-to-net passthrough, zero buffering) through "
    "every recursion level below, to as many as NumberInputs/2 leaf MUX2 "
    "S0 pins for the LSB (measured directly: at N=512 this single "
    "under-sized inverter's own RC delay was 2.364e-10s of a 3.489e-10s "
    "total t_read_addr_path, 67.8%). Fixed the same way as the enable/row "
    "chains: each select bit now gets its own tapered BUF chain "
    "(Multiplexer.cc's SelectBufferStages/SelectBufferTaper parameters), "
    "sized from that bit's own real fanout (countSelectBitFanout(), a "
    "pure structural recursion mirroring the tree's own recursive "
    "construction) -- cut t_read_addr_path at N=512 from 3.489e-10s to "
    "1.487e-10s (57.4%). The MUX TREE'S OWN DATA PATH (In->MUX2 chain-> "
    "Out) does NOT have this defect: every stage there has fanout exactly "
    "1 (drives only the next level's single MUX2 input), so uniform "
    "minimum sizing is structurally correct, not a bug -- left unchanged. "
    "After this fix, e_read is NO LONGER expected to match earlier "
    "sessions' 'e_read must be unchanged' checks -- those checks were "
    "specifically about the DFFRAM clock/enable path never touching the "
    "read path, which was true until this fix touched Multiplexer.cc "
    "itself (part of the read datapath); e_read now correctly varies with "
    "the select-buffer chains' own N-dependent sizing, which is a "
    "correction (the old fixed-2-INV scheme charged the same select-buffer "
    "energy regardless of N, an inaccuracy), not a new defect. The "
    "remaining read-path depth scaling (t_read_addr_path/t_read_access "
    "both still grow with N) is a genuine structural property of an N:1 "
    "mux tree (ceil(log2(N)) series MUX2 delay stages) and was NOT further "
    "reduced -- there is no analogous untapered-fanout defect left to fix "
    "in it.",
]

# Read energy is linear in depth: DSENT's DFFRAM uses a broadcast-select
# binary mux tree, so all N-1 MUX2 cells switch per read (see
# e_read_linear_in_depth below). Write energy is linear in depth too: all N
# entries' D-input inverters see the shared write bus.


def linreg(xs, ys):
    """Manual least-squares linear regression. Returns (slope, intercept, r_squared)."""
    n = len(xs)
    if n < 2:
        return 0.0, (ys[0] if ys else 0.0), 1.0
    mean_x = sum(xs) / n
    mean_y = sum(ys) / n
    sxx = sum((x - mean_x) ** 2 for x in xs)
    sxy = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys))
    syy = sum((y - mean_y) ** 2 for y in ys)
    if sxx == 0.0:
        return 0.0, mean_y, (1.0 if syy == 0.0 else 0.0)
    slope = sxy / sxx
    intercept = mean_y - slope * mean_x
    if syy == 0.0:
        r2 = 1.0
    else:
        ss_res = sum((y - (slope * x + intercept)) ** 2 for x, y in zip(xs, ys))
        r2 = 1.0 - ss_res / syy
    return slope, intercept, r2


def run_git(args):
    r = subprocess.run(["git"] + args, cwd=REPO_ROOT, capture_output=True, text=True)
    return r.stdout.strip()


BUILTIN_DEFAULTS = {
    "dsent_bin": os.path.join(REPO_ROOT, "dsent"),
    "tech": "tech/tech_models/Bulk22LVT.model",
    "frequency": 1e9,
    "depths": "4,8,16,32,64,128,256,512",
    "widths": "32,64,128,256",
    "num_vcs": "1",
    "write_rate": 1.0,
    "read_rate": 1.0,
    "flit_toggle_rate": 0.5,
    "timing_opt": "true",
    "outdir": os.path.join(REPO_ROOT, "sweep_output"),
}


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--dsent-bin", default=BUILTIN_DEFAULTS["dsent_bin"],
                    help="Path to the dsent binary")
    p.add_argument("--tech", default=BUILTIN_DEFAULTS["tech"],
                    help="Tech model path, passed through to --tech")
    p.add_argument("--frequency", type=float, default=BUILTIN_DEFAULTS["frequency"],
                    help="Frequency in Hz")
    p.add_argument("--depths", default=BUILTIN_DEFAULTS["depths"],
                    help="Comma-separated list of --bufs-per-vc depths to sweep")
    p.add_argument("--widths", default=BUILTIN_DEFAULTS["widths"],
                    help="Comma-separated list of --flit-width values to sweep")
    p.add_argument("--num-vcs", default=BUILTIN_DEFAULTS["num_vcs"],
                    help="Comma-separated list of --vcs-per-vn values to sweep "
                         "(new dimension -- V in Becker Table 6.1)")
    p.add_argument("--write-rate", type=float, default=BUILTIN_DEFAULTS["write_rate"],
                    help="Phase 5 WriteRate, passed through to every point (default 1.0)")
    p.add_argument("--read-rate", type=float, default=BUILTIN_DEFAULTS["read_rate"],
                    help="Phase 5 ReadRate, passed through to every point (default 1.0)")
    p.add_argument("--flit-toggle-rate", type=float, default=BUILTIN_DEFAULTS["flit_toggle_rate"],
                    help="Phase 5 FlitToggleRate, passed through to every point (default 0.5)")
    p.add_argument("--timing-opt", default=BUILTIN_DEFAULTS["timing_opt"], choices=["true", "false"],
                    help="Passed through to --timing-opt")
    p.add_argument("--outdir", default=BUILTIN_DEFAULTS["outdir"],
                    help="Output directory for buffer_metrics.json and run_log.txt")
    return p.parse_args()


def run_one_config(dsent_bin, tech, frequency, depth, width, num_vcs,
                    write_rate, read_rate, flit_toggle_rate, timing_opt, repo_root):
    """Run a single dsent invocation. Returns (row_dict_or_None, error_str_or_None, cmd_str)."""
    # Always route dsent's --out through a tempfile, whatever format it turns
    # out to be, and always delete it before returning -- never leave the
    # tool's raw output (CSV or otherwise) on disk.
    fd, tmp_path = tempfile.mkstemp(suffix=".out", prefix="dsent_buf_")
    os.close(fd)
    cmd = [
        dsent_bin, "buffer_characteristics",
        "--tech", tech,
        "--frequency", str(frequency),
        "--num-vns", "1",
        "--vcs-per-vn", str(num_vcs),
        "--bufs-per-vc", str(depth),
        "--flit-width", str(width),
        "--write-rate", str(write_rate),
        "--read-rate", str(read_rate),
        "--flit-toggle-rate", str(flit_toggle_rate),
        "--timing-opt", timing_opt,
        "--out", tmp_path,
    ]
    cmd_str = " ".join(cmd)
    try:
        proc = subprocess.run(cmd, cwd=repo_root, capture_output=True, text=True, timeout=120)
    except subprocess.TimeoutExpired as e:
        if os.path.exists(tmp_path):
            os.remove(tmp_path)
        return None, "TIMEOUT after 120s: %s" % str(e), cmd_str

    if proc.returncode != 0:
        err = (proc.stderr.strip() or proc.stdout.strip() or
               "non-zero exit code %d, no stderr/stdout captured" % proc.returncode)
        if os.path.exists(tmp_path):
            os.remove(tmp_path)
        return None, err, cmd_str

    try:
        with open(tmp_path, "r") as f:
            raw_text = f.read()
        try:
            raw = json.loads(raw_text)
        except json.JSONDecodeError:
            # This binary is known to emit JSON only; if that ever changes to
            # CSV, this is the point a CSV parser would need to be added.
            # Whatever the format, the tempfile is still deleted below -- a
            # .csv never survives on disk either way.
            raise ValueError(
                "dsent output at %s is not valid JSON (first 200 chars: %r); "
                "CSV parsing is not implemented in this script" % (tmp_path, raw_text[:200]))
        if not isinstance(raw, list) or len(raw) != 1:
            raise ValueError("expected a 1-element JSON array, got: %r" % (raw,))
        row = raw[0]
        return row, None, cmd_str
    except Exception as e:
        return None, str(e), cmd_str
    finally:
        if os.path.exists(tmp_path):
            os.remove(tmp_path)


def becker_check(row):
    """Part B: compare the model's counted register fields against Becker's
    Table 6.1 formulas for this row's (idx_bits, num_vcs). Returns a dict."""
    regs = row["registers"]
    return {
        "num_vcs": regs["num_vcs"],
        "idx_bits": regs["idx_bits"],
        "cnt_bits": regs["cnt_bits"],
        "input_ctrl_regs_model": regs["input_ctrl_regs_model"],
        "input_ctrl_regs_becker": regs["input_ctrl_regs_becker"],
        "input_matches_becker": regs["input_ctrl_regs_match_becker"],
        "output_ctrl_regs_model": regs["output_ctrl_regs_model"],
        "output_ctrl_regs_becker": regs["output_ctrl_regs_becker"],
        "output_matches_becker": regs["output_ctrl_regs_match_becker"],
    }


def main():
    args = parse_args()
    depths = [int(x) for x in args.depths.split(",") if x.strip()]
    widths = [int(x) for x in args.widths.split(",") if x.strip()]
    num_vcs_list = [int(x) for x in args.num_vcs.split(",") if x.strip()]

    dsent_bin = args.dsent_bin
    if os.sep not in dsent_bin and "/" not in dsent_bin:
        dsent_bin = os.path.join(REPO_ROOT, dsent_bin)

    os.makedirs(args.outdir, exist_ok=True)
    metrics_path = os.path.join(args.outdir, "buffer_metrics.json")
    log_path = os.path.join(args.outdir, "run_log.txt")

    dsent_git_hash = run_git(["rev-parse", "HEAD"])
    dsent_tree_dirty = len(run_git(["status", "--porcelain"])) > 0
    run_timestamp = datetime.now(timezone.utc).isoformat()

    log_lines = []
    def log(line=""):
        print(line)
        log_lines.append(line)

    log("=" * 78)
    log("DSENT buffer_characteristics sweep -- Phase 5 (traffic-driven activity)")
    log("=" * 78)
    log("dsent_bin        = %s" % dsent_bin)
    log("tech_model       = %s" % args.tech)
    log("frequency_hz     = %g" % args.frequency)
    log("depths           = %s" % depths)
    log("widths           = %s" % widths)
    log("num_vcs          = %s" % num_vcs_list)
    log("write_rate       = %s" % args.write_rate)
    log("read_rate        = %s" % args.read_rate)
    log("flit_toggle_rate = %s" % args.flit_toggle_rate)
    log("timing_opt       = %s" % args.timing_opt)
    log("outdir           = %s" % args.outdir)
    log("dsent_git_hash   = %s" % dsent_git_hash)
    log("dsent_tree_dirty = %s" % dsent_tree_dirty)
    log("run_timestamp    = %s" % run_timestamp)
    log("")
    log("Each (depth, width, num_vcs) point is a separate dsent subprocess")
    log("invocation, so a crash on any single configuration is caught and logged")
    log("into \"failures\" without aborting the rest of the sweep. dsent's --out")
    log("is always routed through a tempfile that is deleted immediately after")
    log("parsing -- no CSV is ever produced or left on disk.")
    log("")
    log("-" * 78)
    log("PER-CONFIGURATION RUNS")
    log("-" * 78)

    points = []
    failures = []
    becker_checks = []
    total_points = len(depths) * len(widths) * len(num_vcs_list)

    for num_vcs in num_vcs_list:
        for width in widths:
            for depth in depths:
                row, err, cmd_str = run_one_config(
                    dsent_bin, args.tech, args.frequency, depth, width, num_vcs,
                    args.write_rate, args.read_rate, args.flit_toggle_rate,
                    args.timing_opt, REPO_ROOT)
                log("$ %s" % cmd_str)
                if err is not None:
                    log("  FAILED depth=%d width=%d num_vcs=%d -- %s" % (depth, width, num_vcs, err))
                    failures.append({"depth": depth, "width": width, "num_vcs": num_vcs, "error": err})
                    continue

                # Part B: register-count sanity check runs for every point,
                # independent of whether timing was met -- area/leakage (and
                # hence register counts) are still computed even when the
                # energy fields are blanked to null.
                bc = becker_check(row)
                bc["depth"] = depth
                bc["width"] = width
                becker_checks.append(bc)

                if not row["timing"]["timing_met"]:
                    err = ("timing not met at target frequency %g Hz -- t_critical=%.6g s, "
                           "slack=%.6g s; energy fields are null for this row"
                           % (args.frequency, row["timing"]["t_critical"], row["timing"]["slack"]))
                    log("  FAILED depth=%d width=%d num_vcs=%d -- %s" % (depth, width, num_vcs, err))
                    failures.append({"depth": depth, "width": width, "num_vcs": num_vcs, "error": err})
                    continue

                derived = row["derived"]
                area = row["area"]
                leak = row["leakage"]
                energy = row["energy"]
                timing = row["timing"]

                actual_depth = depth
                actual_width = row["inputs"]["flit_width"]
                total_bits = derived["total_dff_bits"]

                # Report Area->Active only -- never sum with Metal1Wire.
                area_um2 = area["area_ram"] * 1e12
                area_per_bit_um2 = area_um2 / total_bits if total_bits else float("nan")

                # Energy is already C*Vdd^2 per 0->1 transition -- no 1/2 factor.
                e_read_ram_j = energy["e_read"] - energy["e_read_rdaddr_dff"]
                e_write_ram_j = energy["e_write"] - energy["e_write_rdaddr_dff"]
                e_read_pJ = e_read_ram_j * 1e12
                e_write_pJ = e_write_ram_j * 1e12
                e_read_per_bit_fJ = e_read_pJ / actual_width * 1000.0
                e_write_per_bit_fJ = e_write_pJ / actual_width * 1000.0

                leak_mW = leak["leak_ram"] * 1e3

                t_write_data_setup_ns = timing["t_write_data_setup"] * 1e9
                t_write_clock_path_ns = timing["t_write_clock_path"] * 1e9
                t_read_access_ns = timing["t_read_access"] * 1e9
                t_read_addr_path_ns = timing["t_read_addr_path"] * 1e9
                t_write_critical_ns = max(t_write_data_setup_ns, t_write_clock_path_ns)
                t_read_critical_ns = max(t_read_access_ns, t_read_addr_path_ns)
                t_critical_ns = timing["t_critical"] * 1e9
                slack_ns = timing["slack"] * 1e9

                points.append({
                    "depth": actual_depth,
                    "width": actual_width,
                    "num_vcs": num_vcs,
                    "area_um2": area_um2,
                    "area_per_bit_um2": area_per_bit_um2,
                    "e_read_pJ": e_read_pJ,
                    "e_write_pJ": e_write_pJ,
                    "e_read_per_bit_fJ": e_read_per_bit_fJ,
                    "e_write_per_bit_fJ": e_write_per_bit_fJ,
                    "leak_mW": leak_mW,
                    "registers": row["registers"],
                    "timing": {
                        "t_write_data_setup_ns": t_write_data_setup_ns,
                        "t_write_clock_path_ns": t_write_clock_path_ns,
                        "t_write_ns": t_write_critical_ns,
                        "t_read_access_ns": t_read_access_ns,
                        "t_read_addr_path_ns": t_read_addr_path_ns,
                        "t_read_ns": t_read_critical_ns,
                        "t_critical_ns": t_critical_ns,
                        "slack_ns": slack_ns,
                    },
                })
                log("  OK depth=%d width=%d num_vcs=%d -> area_um2=%.6g e_read_pJ=%.6g e_write_pJ=%.6g "
                    "leak_mW=%.6g t_critical_ns=%.6g"
                    % (actual_depth, actual_width, num_vcs, area_um2, e_read_pJ, e_write_pJ,
                       leak_mW, t_critical_ns))

    log("")
    log("-" * 78)
    log("SANITY CHECKS")
    log("-" * 78)

    by_wv = {}
    for p in points:
        by_wv.setdefault((p["width"], p["num_vcs"]), []).append(p)
    for k in by_wv:
        by_wv[k].sort(key=lambda p: p["depth"])

    # 1. area_linear_in_bits (per width, num_vcs=1 only -- multi-VC area has
    # a real, non-linear-in-bits step from per-VC control logic, see Part C).
    AREA_R2_THRESHOLD = 0.999
    area_r2 = {}
    area_pass = True
    for (w, v), pts in by_wv.items():
        if v != 1:
            continue
        xs = [p["depth"] * p["width"] for p in pts]
        ys = [p["area_um2"] for p in pts]
        _, _, r2 = linreg(xs, ys)
        area_r2[str(w)] = r2
        if r2 < AREA_R2_THRESHOLD:
            area_pass = False
    log("area_linear_in_bits (threshold R^2 >= %.4f per width, num_vcs=1):" % AREA_R2_THRESHOLD)
    for w in sorted(area_r2, key=int):
        log("    width=%-4s R^2=%.6f" % (w, area_r2[w]))
    log("  pass = %s" % area_pass)
    log("")

    # 2. e_write_linear_in_depth (num_vcs=1 only, same reasoning as above)
    WRITE_R2_THRESHOLD = 0.99
    write_slope = {}
    write_r2 = {}
    write_pass = True
    for (w, v), pts in by_wv.items():
        if v != 1:
            continue
        xs = [p["depth"] for p in pts]
        ys = [p["e_write_pJ"] for p in pts]
        slope, _, r2 = linreg(xs, ys)
        write_slope[str(w)] = slope
        write_r2[str(w)] = r2
        if r2 < WRITE_R2_THRESHOLD or slope <= 0:
            write_pass = False
    log("e_write_linear_in_depth (threshold R^2 >= %.4f and slope > 0, per width, num_vcs=1):" % WRITE_R2_THRESHOLD)
    for w in sorted(write_slope, key=int):
        log("    width=%-4s slope=%.6f pJ/entry  R^2=%.6f" % (w, write_slope[w], write_r2[w]))
    log("  pass = %s" % write_pass)
    log("")

    # 3. e_read_linear_in_depth (num_vcs=1 only)
    READ_R2_THRESHOLD = 0.99
    read_slope = {}
    read_r2 = {}
    read_pass = True
    for (w, v), pts in by_wv.items():
        if v != 1:
            continue
        xs = [p["depth"] for p in pts]
        ys = [p["e_read_pJ"] for p in pts]
        slope, _, r2 = linreg(xs, ys)
        read_slope[str(w)] = slope
        read_r2[str(w)] = r2
        if r2 < READ_R2_THRESHOLD or slope <= 0:
            read_pass = False
    log("e_read_linear_in_depth (threshold R^2 >= %.4f and slope > 0, per width, num_vcs=1):" % READ_R2_THRESHOLD)
    for w in sorted(read_slope, key=int):
        log("    width=%-4s slope=%.6f pJ/entry  R^2=%.6f" % (w, read_slope[w], read_r2[w]))
    log("  pass = %s" % read_pass)
    log("")

    # 4. leak_linear_in_bits (num_vcs=1 only)
    LEAK_R2_THRESHOLD = 0.999
    v1_points = [p for p in points if p["num_vcs"] == 1]
    xs = [p["depth"] * p["width"] for p in v1_points]
    ys = [p["leak_mW"] for p in v1_points]
    _, _, leak_r2 = linreg(xs, ys) if v1_points else (0.0, 0.0, 1.0)
    leak_pass = leak_r2 >= LEAK_R2_THRESHOLD
    log("leak_linear_in_bits (threshold R^2 >= %.4f, all num_vcs=1 points combined):" % LEAK_R2_THRESHOLD)
    log("    R^2=%.6f" % leak_r2)
    log("  pass = %s" % leak_pass)
    log("")

    # 5. Part B -- Becker Table 6.1 register-count match, every (depth, num_vcs) point.
    # Both sides are expected to match Becker exactly: OutputVCState's credit
    # counter is cnt_bits = ceil(log2(NumberEntries)) (no +1) -- Busy_DFF/
    # Empty_DFF already carry the two boundary states, matching Table 6.1's
    # own "busy/empty = 2*V" line, so the counter only needs the interior
    # states. See OutputVCState.cc constructModel(). NOTE: a NumberEntries=1
    # slice depth makes cnt_bits=0, which fails construction outright (a
    # zero-input OR gate) -- that shows up as a "failures" entry, not a
    # becker_checks mismatch, since the point never gets to the register
    # audit at all in that case.
    becker_input_pass = all(c["input_matches_becker"] for c in becker_checks) if becker_checks else False
    becker_output_pass = all(c["output_matches_becker"] for c in becker_checks) if becker_checks else False
    becker_mismatches = [c for c in becker_checks
                          if not c["input_matches_becker"] or not c["output_matches_becker"]]
    log("becker_table_6_1_register_counts (both input and output side are expected to match")
    log("Becker exactly -- see OutputVCState.cc constructModel() for why the counter needs no +1):")
    log("  input_matches_becker (all points)  = %s" % becker_input_pass)
    log("  output_matches_becker (all points) = %s" % becker_output_pass)
    log("  %d/%d points checked, %d with any mismatch" % (
        len(becker_checks), total_points, len(becker_mismatches)))
    log("")

    all_present = (len(points) == total_points)
    log("all_points_present = %s (%d/%d points collected, %d failures)"
        % (all_present, len(points), total_points, len(failures)))
    log("")

    sweep_valid = (area_pass and write_pass and read_pass and leak_pass and all_present
                   and becker_input_pass and becker_output_pass)

    sanity_checks = {
        "area_linear_in_bits": {"pass": area_pass, "per_width_R2": area_r2},
        "e_write_linear_in_depth": {"pass": write_pass, "per_width_slope_pJ_per_entry": write_slope},
        "e_read_linear_in_depth": {"pass": read_pass, "per_width_slope_pJ_per_entry": read_slope, "per_width_R2": read_r2},
        "leak_linear_in_bits": {"pass": leak_pass, "R2": leak_r2},
        "becker_table_6_1_input_side": {
            "pass": becker_input_pass,
            "checked_points": len(becker_checks),
        },
        "becker_table_6_1_output_side": {
            "pass": becker_output_pass,
            "checked_points": len(becker_checks),
        },
        "all_points_present": all_present,
    }

    metrics = {
        "meta": {
            "tech_model": args.tech,
            "frequency_hz": args.frequency,
            "dsent_git_hash": dsent_git_hash,
            "dsent_tree_dirty": dsent_tree_dirty,
            "write_rate": args.write_rate,
            "read_rate": args.read_rate,
            "flit_toggle_rate": args.flit_toggle_rate,
            "timing_opt": (args.timing_opt == "true"),
            "run_timestamp": run_timestamp,
            "assumptions": ASSUMPTIONS,
        },
        "points": points,
        "becker_table_6_1_checks": becker_checks,
        "failures": failures,
        "sanity_checks": sanity_checks,
        "sweep_valid": sweep_valid,
    }

    with open(metrics_path, "w") as f:
        json.dump(metrics, f, indent=2)
        f.write("\n")

    log("-" * 78)
    log("FINAL SANITY CHECK SUMMARY")
    log("-" * 78)
    log("  area_linear_in_bits          : %s" % area_pass)
    log("  e_write_linear_in_depth      : %s" % write_pass)
    log("  e_read_linear_in_depth       : %s" % read_pass)
    log("  leak_linear_in_bits          : %s" % leak_pass)
    log("  becker_table_6_1_input_side  : %s" % becker_input_pass)
    log("  becker_table_6_1_output_side : %s" % becker_output_pass)
    log("  all_points_present           : %s" % all_present)
    log("  sweep_valid                  : %s" % sweep_valid)
    log("")
    log("wrote %s (%d points, %d failures)" % (metrics_path, len(points), len(failures)))
    log("-" * 78)

    with open(log_path, "w") as f:
        f.write("\n".join(log_lines) + "\n")

    return 0 if sweep_valid else 1


if __name__ == "__main__":
    sys.exit(main())

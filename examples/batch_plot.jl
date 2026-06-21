# batch_plot.jl
# Command-line plotting tool for kite simulation logs.
# No GUI — select a plot from a text menu.
# Usage: julia batch_plot.jl [path/to/project.yml]

using Pkg
if ! ("MakieControlPlots" ∈ keys(Pkg.project().dependencies))
    Pkg.activate(@__DIR__)
end

if !@isdefined __PRECOMPILE__
    __PRECOMPILE__ = false
end

using KiteControllers, LaTeXStrings, MakieControlPlots, Statistics, YAML
using REPL.TerminalMenus
using KiteModels: reactivate_host_app

include("yaml_utils.jl")

# Paths
const GUI_YAML   = joinpath(@__DIR__, "..", "data", "gui.yaml")
const BATCH_OUTPUT_DIR = joinpath(@__DIR__, "..", "output")

# ---------------------------------------------------------------------------
# Project management helpers
# ---------------------------------------------------------------------------
"""Return sorted list of project names that have a batch-*.arrow file."""
function discover_projects()
    files = filter(f -> startswith(f, "batch-") && endswith(f, ".arrow"),
                   readdir(BATCH_OUTPUT_DIR))
    sort([replace(replace(f, r"^batch-" => ""), r"\.arrow$" => "") for f in files])
end

"""Read currently selected project name (without .yml) from gui.yaml."""
function read_project_name()
    cfg = YAML.load_file(GUI_YAML)
    project = get(get(cfg, "gui", Dict()), "project", "hydra20_600.yml")
    replace(project, r"\.yml$" => "")
end

"""Persist selected project name to gui.yaml."""
function write_project_name(project::String)
    content = read(GUI_YAML, String)
    # Replace the value after "project:" up to the first whitespace/newline/comment
    new_content = replace(content, r"(project:\s+)\S+" => SubstitutionString("\\1" * project * ".yml"))
    write(GUI_YAML, new_content)
end

"""Return the full path to the batch arrow file for a given project name."""
function project_arrow(project::String)
    joinpath(BATCH_OUTPUT_DIR, "batch-" * project * ".arrow")
end

# Mutable reference to the active log file (updated when project is changed)
function _resolve_plot_file(arg::String)
    # Full path given
    dirname(arg) != "" && return arg
    # Already looks like a complete arrow filename
    endswith(arg, ".arrow") && return joinpath(BATCH_OUTPUT_DIR, arg)
    # Try treating as a project name: batch-<arg>.arrow
    candidate = joinpath(BATCH_OUTPUT_DIR, "batch-" * arg * ".arrow")
    isfile(candidate) && return candidate
    # Fall back to using the arg as a bare filename in output/
    joinpath(BATCH_OUTPUT_DIR, arg)
end

const PLOT_FILE = Ref{String}(
    if isempty(ARGS)
        project_arrow(read_project_name())
    else
        let resolved = _resolve_plot_file(ARGS[1])
            # Persist the selected project to gui.yaml so the menu reflects it
            project_name = replace(replace(basename(resolved), r"\.arrow$" => ""), r"^batch-" => "")
            write_project_name(project_name)
            resolved
        end
    end
)

# ---------------------------------------------------------------------------
# Core plot logic (shared with plots.jl via plot_functions.jl)
# ---------------------------------------------------------------------------
include("plot_functions.jl")

# ---------------------------------------------------------------------------
# Load helper
# ---------------------------------------------------------------------------
function load_plot_log()
    load_log(basename(PLOT_FILE[]); path=dirname(PLOT_FILE[]))
end

# ---------------------------------------------------------------------------
# Plot wrappers — load the log, then delegate to the shared implementation
# ---------------------------------------------------------------------------
function plot_main()
    _plot_main(load_plot_log(); ysize=10)
end

function plot_power()
    log = load_plot_log()
    _plot_power(log; ysize=10, energy=log.syslog.e_mech, dt=0.0)
end

function plot_control()
    _plot_control(load_plot_log(); ysize=10)
end

function plot_control_II()
    _plot_control_II(load_plot_log(); ysize=10)
end

function plot_winch_control()
    _plot_winch_control(load_plot_log(); ysize=10)
end

function plot_aerodynamics(plot_lift_drag=false)
    _plot_aerodynamics(load_plot_log(); plot_lift_drag=plot_lift_drag)
end

function plot_elev_az()
    _plot_elev_az(load_plot_log())
end

function plot_elev_az2()
    _plot_elev_az_n(load_plot_log(), 2)
end

function plot_elev_az3()
    _plot_elev_az_n(load_plot_log(), 3)
end

function plot_side_view()
    _plot_side_view(load_plot_log())
end

function plot_side_view2()
    _plot_side_view_n(load_plot_log(), 2)
end

function plot_side_view3()
    _plot_side_view_n(load_plot_log(), 3)
end

function plot_front_view3()
    _plot_front_view_n(load_plot_log(), 3)
end

# ---------------------------------------------------------------------------
# Project selection sub-menu
# ---------------------------------------------------------------------------
function select_project_menu()
    projects = discover_projects()
    if isempty(projects)
        println("No batch-*.arrow files found in $BATCH_OUTPUT_DIR")
        return
    end
    current = read_project_name()
    println("\nCurrent project: $current")
    opts = vcat(projects, ["cancel"])
    menu = RadioMenu(opts, pagesize=8)
    choice = request("\nSelect project: ", menu)
    if choice == -1 || choice == length(opts)
        println("Project selection cancelled.")
        return
    end
    selected = projects[choice]
    write_project_name(selected)
    PLOT_FILE[] = project_arrow(selected)
    println("Project set to: $selected")
    println("Log file: $(PLOT_FILE[])")
    nothing
end

# ---------------------------------------------------------------------------
# Statistics
# ---------------------------------------------------------------------------
function highlight_yaml(content::String)
    RESET  = "\033[0m"
    BOLD   = "\033[1m"
    CYAN   = "\033[36m"
    YELLOW = "\033[33m"
    GREEN  = "\033[32m"
    MAGENTA = "\033[35m"

    buf = IOBuffer()
    for line in split(content, '\n')
        # Section header: "key:" with nothing after colon
        m = match(r"^(\s*)([\w]+)(\s*:\s*)$", line)
        if m !== nothing
            indent, key, colon = m.captures
            print(buf, indent * BOLD * CYAN * key * RESET * colon * "\n")
            continue
        end
        # Key: value  # optional inline comment
        m = match(r"^(\s*)([\w]+)(\s*:\s*)(\"[^\"]*\"|-?[0-9][0-9.]*|[^#\n]*?)(\s*)(#[^\n]*)?\n?$", line)
        if m !== nothing
            indent, key, colon, value, space, comment = m.captures
            value   = something(value,   "")
            space   = something(space,   "")
            comment = something(comment, "")
            # choose value color: string → yellow, number → magenta, empty → default
            val_color = occursin(r"^\s*\"", value) ? YELLOW :
                        occursin(r"^\s*-?[0-9]", value) ? MAGENTA : RESET
            colored = indent * CYAN * key * RESET * colon *
                      val_color * value * RESET * space *
                      (isempty(comment) ? "" : GREEN * comment * RESET)
            print(buf, colored * "\n")
            continue
        end
        print(buf, line * "\n")
    end
    String(take!(buf))
end

function print_statistics()
    project = read_project_name()
    stats_file = joinpath(BATCH_OUTPUT_DIR, "batch-" * project * "_stats.yaml")
    if !isfile(stats_file)
        println("No stats file found: $stats_file")
        return
    end
    println("\033[2J\033[H")  # clear terminal
    print(highlight_yaml(read(stats_file, String)))
    nothing
end

# ---------------------------------------------------------------------------
# Interactive menu (REPL.TerminalMenus RadioMenu)
# ---------------------------------------------------------------------------
const MENU_ITEMS = [
    ("select project",     select_project_menu),
    ("statistics",         print_statistics),
    ("plot_main",          plot_main),
    ("plot_power",         plot_power),
    ("plot_control",       plot_control),
    ("plot_control_II",    plot_control_II),
    ("plot_winch_control", plot_winch_control),
    ("plot_aerodynamics",  () -> plot_aerodynamics(false)),
    ("plot_elev_az",       plot_elev_az),
    ("plot_elev_az2",      plot_elev_az2),
    ("plot_elev_az3",      plot_elev_az3),
    ("plot_side_view",     plot_side_view),
    ("plot_side_view2",    plot_side_view2),
    ("plot_side_view3",    plot_side_view3),
    ("plot_front_view3",   plot_front_view3)
]

const OPTIONS = [item[1] for item in MENU_ITEMS]
push!(OPTIONS, "quit")

function run_menu()
    println("\nLog file: $(PLOT_FILE[])")
    active = true
    while active
        menu   = RadioMenu(OPTIONS, pagesize=8)
        # Derive active project name from the current log file
        log_base = replace(basename(PLOT_FILE[]), r"\.arrow$" => "")
        active_project = replace(log_base, r"^batch-" => "")
        choice = request("\nActive project: \e[1m$active_project\e[0m  Select new project or choose plot to display or `q` to quit: ", menu)
        if choice != -1 && choice != length(OPTIONS)
            name, fn = MENU_ITEMS[choice]
            println("Running $name …")
            try
                fn()
            catch e
                println("Error in $name: $e")
            end
            reactivate_host_app()
        else
            println("Left menu. Press <ctrl><d> to quit Julia!")
            active = false
        end
    end
end

function run_command(cmd::String)
    idx = findfirst(item -> item[1] == cmd, MENU_ITEMS)
    if isnothing(idx)
        println("Unknown command: $cmd")
        println("Available commands: ", join([item[1] for item in MENU_ITEMS], ", "))
        return
    end
    name, fn = MENU_ITEMS[idx]
    println("Running $name …")
    fn()
    wait_for_figures()
    reactivate_host_app()
end

if !__PRECOMPILE__
    if length(ARGS) >= 2
        run_command(ARGS[2])
    else
        run_menu()
    end
end

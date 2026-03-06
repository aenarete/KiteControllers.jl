using Documenter, KiteControllers

makedocs(
    sitename = "KiteControllers.jl",
    authors  = "Uwe Fechner and contributors",
    modules  = [KiteControllers],
    format   = Documenter.HTML(
        prettyurls = get(ENV, "CI", nothing) == "true",
        canonical  = "https://aenarete.github.io/KiteControllers.jl/stable",
    ),
    pages = [
        "Home"      => "index.md",
        "Types"     => "types.md",
        "Functions" => "functions.md",
        "Examples"  => "examples.md",
    ],
    checkdocs = :exports,
    warnonly  = true,
)

deploydocs(
    repo   = "github.com/aenarete/KiteControllers.jl.git",
    target = "build",
    branch = "gh-pages",
    devbranch = "main",
)

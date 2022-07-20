# Use
#
#     DOCUMENTER_DEBUG=true julia --color=yes make.jl local [nonstrict] [fixdoctests]
#
# for local builds.

using Documenter
using TrinamicMotionControl

# Doctest setup
DocMeta.setdocmeta!(
    TrinamicMotionControl,
    :DocTestSetup,
    :(using TrinamicMotionControl);
    recursive=true,
)

makedocs(
    sitename = "TrinamicMotionControl",
    modules = [TrinamicMotionControl],
    format = Documenter.HTML(
        prettyurls = !("local" in ARGS),
        canonical = "https://mppmu.github.io/TrinamicMotionControl.jl/stable/"
    ),
    pages = [
        "Home" => "index.md",
        "API" => "api.md",
        "LICENSE" => "LICENSE.md",
    ],
    doctest = ("fixdoctests" in ARGS) ? :fix : true,
    linkcheck = !("nonstrict" in ARGS),
    strict = !("nonstrict" in ARGS),
)

deploydocs(
    repo = "github.com/mppmu/TrinamicMotionControl.jl.git",
    forcepush = true,
    push_preview = true,
)

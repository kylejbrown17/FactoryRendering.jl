using FactoryRendering
using Documenter

makedocs(;
    modules=[FactoryRendering],
    authors="Kyle Brown <kylejbrown17@gmail.com> and contributors",
    repo="https://github.com/kylejbrown17/FactoryRendering.jl/blob/{commit}{path}#L{line}",
    sitename="FactoryRendering.jl",
    format=Documenter.HTML(;
        prettyurls=get(ENV, "CI", "false") == "true",
        canonical="https://kylejbrown17.github.io/FactoryRendering.jl",
        assets=String[],
    ),
    pages=[
        "Home" => "index.md",
    ],
)

deploydocs(;
    repo="github.com/kylejbrown17/FactoryRendering.jl",
)

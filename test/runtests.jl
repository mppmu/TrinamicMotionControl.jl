# This file is a part of TrinamicMotionControl.jl, licensed under the MIT License (MIT).

import Test

Test.@testset "Package TrinamicMotionControl" begin
    #include("test_aqua.jl")
    include("test_docs.jl")
end # testset

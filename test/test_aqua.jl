# This file is a part of TrinamicMotionControl.jl, licensed under the MIT License (MIT).

import Test
import Aqua
import TrinamicMotionControl

Test.@testset "Aqua tests" begin
    Aqua.test_all(TrinamicMotionControl)
end # testset

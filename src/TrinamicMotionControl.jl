# This file is a part of TrinamicMotionControl.jl, licensed under the MIT License (MIT).

"""
    TrinamicMotionControl

Julia Trinamic Motion Controllers
"""
module TrinamicMotionControl

include("tmcm3110.jl")
include("motor.jl")

end # module

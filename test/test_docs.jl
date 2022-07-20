# This file is a part of TrinamicMotionControl.jl, licensed under the MIT License (MIT).

using Test
using TrinamicMotionControl
import Documenter

Documenter.DocMeta.setdocmeta!(
    TrinamicMotionControl,
    :DocTestSetup,
    :(using TrinamicMotionControl);
    recursive=true,
)
Documenter.doctest(TrinamicMotionControl)

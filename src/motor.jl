# This file is a part of TrinamicMotionControl.jl, licensed under the MIT License (MIT).


mutable struct Motor
    name::String
    id::Int
    axis_length::Value
    motor_position::Value
    encoder_position::Value
    unit::Symbol # :mm or :degree
    calibrated::Bool
    save_to_move::Bool


    Motor(name::String, id::Int, unit::Symbol, save_to_move::Bool) = new(name, id, missing, missing, missing, unit, false, save_to_move)

    function Base.println(m::Motor)
        println()
        println("\t--- $(m.name) ---")
        println("\tmotor position = $(m.motor_position) $(m.unit)")
        println("\tencoder position = $(m.encoder_position) $(m.unit)")
        println("\taxis length = $(m.axis_length) $(m.unit)")
        println("\tcalibrated = $(m.calibrated)")
        println("\tis save to move = $(m.save_to_move)")
        # println()
    end
    function Base.show(m::Motor) println(m) end
    function Base.display(m::Motor) println(m) end
end

"""
    # motor_location
    This is a new struct. Not yet completely integrated...
"""
mutable struct motor_location
    r::Real
    φ::Real
    z::Real

    motor_location(r::Real, φ::Real, z::Real) = new(r, φ, z)
    motor_location() = new(0, 0, 0)
end

function move(loc::motor_location)
    move(r = loc.r, z = loc.z, φ = loc.φ)
end

const accepted_relative_motor_encoder_deviation = 0.05

function convert_unit_to_steps(m::Motor, value::Value)::Union{Int32, Missing}
    if m.unit == :mm
        Int32(round( 25600 * value))
    elseif m.unit == :degree
        Int32(round((51200 / 1.125) * value))
    else
        error("$(m.name) hat not the right unit. Must be ':mm' or ':degree'")
    end
end
function convert_steps_to_unit(m::Motor, steps::Union{Int16, Int32, Int64, Missing})::Value
    if m.unit == :mm
        try
            return Float64(steps/25600.)
        catch err
            @warn err
            return missing
        end
    elseif m.unit == :degree
        try
            return Float64(steps/(51200/1.125))
        catch err
            @warn err
            return missing
        end
    else
        error("$(m.name) hat not the right unit. Must be ':mm' or ':degree'")
    end
end

function stop(m::Motor, motor_controller::TMCM3110)::Nothing
    stop(motor_controller, n_motor=m.id)

    @info "Check $(m.name) velocity:"
    speed = get_axis_parameter( motor_controller, 3, m.id )
    @info "Motor speed = $speed"
    if speed==0
        @info "$(m.name) stoped!"
    else
        @warn "$(m.name) still moving."
        stop(m, motor_controller)
    end
    nothing
end


function update_motor_position!(m::Motor, motor_controller::TMCM3110)::Nothing
    try
        steps = get_axis_parameter(motor_controller, 1, m.id)
        pos = convert_steps_to_unit(m, steps)
        m.motor_position = pos
    catch err
        @warn err
        m.motor_position = missing
    end
    nothing
end
function update_encoder_position!(m::Motor, motor_controller::TMCM3110)::Nothing
    try
        steps = get_axis_parameter(motor_controller, 209, m.id)
        pos = convert_steps_to_unit(m, steps)
        m.encoder_position = pos
    catch err
        @warn err
        m.encoder_position = missing
    end
    nothing
end
function update_axis_length!(m::Motor, motor_controller::TMCM3110)::Nothing
    try
        steps = get_axis_parameter(motor_controller, 196, m.id  )
        length = convert_steps_to_unit(m, steps)
        m.axis_length = length
    catch err
        @warn err
        m.axis_length = missing
    end
    nothing
end

function update_motor!(m::Motor, motor_controller::TMCM3110)::Nothing
    update_motor_position!(m, motor_controller)
    update_encoder_position!(m, motor_controller)
    update_axis_length!(m, motor_controller)
    nothing
end

function calibrate_motor(m::Motor, motor_controller::TMCM3110; force::Bool = false)::Nothing
    if !m.save_to_move && !force
        @warn "$(m.name) is not save to be moved. Skipping Calibration."
        return nothing
    else
        start_calibration = 0
        get_status = 2
        query(motor_controller, 1, 13, start_calibration, m.id, 0)
        @warn "calibrating $(m.name).... wait till it is finished!"
        sleep(0.05)
        calibration_status = query(motor_controller, 1, 13, get_status, m.id, 0)
        while calibration_status != 0
          sleep(1)
          try
              update_motor!(m, motor_controller)
              temp_encoder_position = get_axis_parameter(motor_controller, 209, m.id)
              println("temp_encoder_position: ", round(m.encoder_position, digits=4), " $(m.unit)")
              sleep(0.05)
              calibration_status = query(motor_controller, 1, 13, get_status, m.id, 0)
          catch err
              @warn err
          end
        end
        sleep(0.2)
        @info "Calibration finished:"
        update_motor!(m, motor_controller)
        @info "$(m.name) -> Axis length = $(round(m.axis_length, digits=3))  $(m.unit)"
        nothing
    end
end

function calibrate_motors(ms::Array{Motor}, motor_controller::TMCM3110)::Nothing
    stm = [ m.save_to_move for m in ms ]
    if !(sum(stm) == length(stm))
        @warn "Motors are not save to be moved. Skipping Calibration."
        return nothing
    else
        start_calibration = 0
        get_status = 2

        for m in ms
            query(motor_controller, 1, 13, start_calibration, m.id, 0)
            sleep(0.1)
        end
        sleep(0.05)
        calibration_status = [missing for m in ms]
        for (i,m) in enumerate(ms)
            calibration_status[i] = query(motor_controller, 1, 13, get_status, m.id, 0)
            sleep(0.05)
        end
        while sum(calibration_status) != 0
            for (i,m) in enumerate(ms)
                update_motor_position!(m); sleep(0.05)
                update_encoder_position!(m); sleep(0.05)
                println("$(m.name): Encoder positions: =$(round(m.encoder_position, digits=4)) $(m.unit)")
                calibration_status[i] = query(motor_controller, 1, 13, get_status, m.id, 0)
                sleep(0.05)
            end
            sleep(4)
        end
        @info "Calibration finished"
        for m in ms
            update_motor!(m); sleep(0.05)
            @info "$(m.name) -> Axis length = $(round(m.axis_length, digits=3))  $(m.unit)"
        end
        return nothing
    end
end

function abort_calibration(ms::Array{Motor}, motor_controller::TMCM3110)
    stop_calibration = 1
    reply = Int[]
    for m in ms
        push!(reply, query(motor_controller, 1, 13, stop_calibration, m.id, 0))
        sleep(0.2)
    end
    return reply
end

function is_calibrated(m::Motor)::Nothing
    if abs(m.motor_position - m.encoder_position) <= accepted_relative_motor_encoder_deviation
        m.calibrated = true
    else
        m.calibrated = false
    end
    nothing
end

function move(m::Motor, motor_controller::TMCM3110, position::Value; force::Bool=false)::Nothing
    if !force
        if !(0 <= position <= m.axis_length)
        	@warn "New 'position' is not within axis range: [0, $(round(m.axis_length, digits=6))]. Ignored move command."
        	return nothing
        elseif !m.save_to_move
        	@warn "$(m.name) is not save to move. Ignored move command."
        	return nothing
        elseif !m.calibrated
            @warn "$(m.name) is not calibrated. Ignored move command."
            return nothing
        end
    end
    move(motor_controller, m.id, convert_unit_to_steps(m, position))
    nothing
end

function target_position_flag(m::Motor, motor_controller::TMCM3110)::Bool
    flag = false
    try
        flag = get_axis_parameter(motor_controller, 8, m.id)
    catch err
        @warn err
    end
    return flag
end

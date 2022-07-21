# This file is a part of TrinamicMotionControl.jl, licensed under the MIT License (MIT).


mutable struct Motor
    name::String
    motor_controller::TMCM3110
    id::Int
    axis_type::Symbol #:periodic, :linear
    axis_range::Union{Missing, ClosedInterval}
    manual_range::Union{Missing, ClosedInterval}
    motor_position::Value
    encoder_position::Value
    calibrated::Bool
    safe_to_move::Bool

    function Motor(name::String,
                   motor_controller::TMCM3110,
                   id::Int,
                   axis_type::Symbol,
                   manual_range::ClosedInterval,
                   safe_to_move::Bool)
        m = new(name, motor_controller, id, axis_type, missing, manual_range, missing, missing, false, safe_to_move)
        update_motor!(m)
        m
    end

    function Motor(name::String,
                   motor_controller::TMCM3110,
                   id::Int,
                   axis_type::Symbol,
                   safe_to_move::Bool)
        m = new(name, motor_controller, id, axis_type, missing, missing, missing, missing, false, safe_to_move)
        update_motor!(m)
        m
    end

    function Base.println(m::Motor)
        println()
        println("\t--- $(m.name) ---")
        println("\tMotor Controller = $(m.motor_controller.name):$(m.id)")
        println("\tMotor Position = $(round(m.motor_position, digits = 2))")
        println("\tEncoder Position = $(round(m.encoder_position, digits = 2))")
        println("\tAxis Range = $(ismissing(m.axis_range) ? missing : 0..round(m.axis_range.right, digits = 2))")
        println("\tManual Range = $(m.manual_range)")
        println("\tCalibrated = $(m.calibrated)")
        println("\tIs safe to move = $(m.safe_to_move)")
        println("\tUnits = $(unit(m))")
    end

    function Base.show(m::Motor) println(m) end
    function Base.display(m::Motor) println(m) end
end

const relative_motor_encoder_tol = 0.05
const encoder_factor = 1.024

function unit(m::Motor)
    if m.axis_type == :linear
        return u"mm"
    elseif m.axis_type == :periodic
        return u"°"
    else
        @error "Incompatible units for $(m.name)"
    end
end

function direction(m::Motor)
    calibration_mode = get_axis_parameter(m.motor_controller, 193, m.id)
    if calibration_mode in [65, 2]
        return 1
    elseif calibration_mode in [66]
        return -1
    else
        @error "Calibration Mode not programmed"
    end
end

function convert_unit_to_steps(m::Motor, val::Value)::Union{Int32, Missing}
    dir = direction(m)
    if m.axis_type == :linear
        Int32(dir * round((5000000 / 97.7) * val))
    elseif m.axis_type == :periodic
        Int32(dir * round((640000 / 360) * val))
    else
        @error "Incompatible units for $(m.name)"
    end
end

function convert_steps_to_unit(m::Motor, steps::Union{Int16, Int32, Int64, Missing})::Value
    dir = direction(m)
    if m.axis_type == :linear
        try
            return dir * steps * (97.7 / 5000000)
        catch err
            @warn err
            return missing
        end
    elseif m.axis_type == :periodic
        try
            return dir * steps * (360 / 640000)
        catch err
            @warn err
            return missing
        end
    else
        @error "Incorrect Axis Type"
    end
end

function stop(m::Motor)::Nothing
    stop(m.motor_controller, n_motor=m.id)

    @info "Check $(m.name) velocity:"
    speed = get_axis_parameter( m.motor_controller, 3, m.id )
    @info "Motor speed = $speed"
    if speed==0
        @info "$(m.name) stoped!"
    else
        sleep(0.1)
        @warn "$(m.name) still moving."
        stop(m)
    end
    nothing
end


function update_motor_position!(m::Motor)::Nothing
    try
        steps = get_axis_parameter(m.motor_controller, 1, m.id)
        pos = convert_steps_to_unit(m, steps)
        m.motor_position = pos
    catch err
        @warn err
        m.motor_position = missing
    end
    nothing
end

function update_encoder_position!(m::Motor)::Nothing
    try
        steps = get_axis_parameter(m.motor_controller, 209, m.id)
        pos = encoder_factor * convert_steps_to_unit(m, steps)
        m.encoder_position = pos
    catch err
        @warn err
        m.encoder_position = missing
    end
    nothing
end

function update_axis_range!(m::Motor)::Nothing
    try
        steps = get_axis_parameter(m.motor_controller, 196, m.id  )
        length = convert_steps_to_unit(m, steps)
        m.axis_range = 0..length
    catch err
        @warn err
        m.axis_range = missing
    end
    nothing
end

function update_motor!(m::Motor)::Nothing
    update_motor_position!(m)
    update_encoder_position!(m)
    update_axis_range!(m)
    is_calibrated!(m)
end

function calibrate_motor(m::Motor; force::Bool = false)::Nothing
    if !m.safe_to_move && !force
        @warn "$(m.name) is not safe to be moved. Skipping Calibration."
        return nothing
    else
        if m.axis_type == :periodic
            update_motor_position!(m)
            update_encoder_position!(m)
            if !ismissing(m.motor_position) && !ismissing(m.encoder_position)
                @warn "Move $(m.name) to -10° to ensure proper cable management."
                δ = m.encoder_position - m.motor_position
                move_to(m, -10 - δ, force = true)
            end
        end
        start_calibration = 0
        get_status = 2
        query(m.motor_controller, 13, start_calibration, m.id, 0)
        sleep(0.05)
        calibration_status = query(m.motor_controller, 13, get_status, m.id, 0)
        prog = ProgressUnknown("Calibrating $(m.name)", spinner = true)
        while calibration_status != 0
          sleep(1)
          try
              update_motor_position!(m)
              update_encoder_position!(m)
              pos = round(m.encoder_position, digits = 2)
              next!(prog, spinner = "🐾👣", showvalues = [("Encoder Position", pos * unit(m))])
              sleep(0.05)
              calibration_status = query(m.motor_controller, 13, get_status, m.id, 0)
          catch err
              @warn err
          end
        end
        finish!(prog)
        @info "\nCalibration finished:"
        sleep(0.1)
        update_motor!(m)
        if m.axis_type == :linear
            @info "$(m.name) -> Axis range = $(m.axis_range)"
        elseif m.axis_type == :periodic
            @info "$(m.name) -> Axis range = Periodic\nManual range = $(m.manual_range)"
        end
    end
end

function abort_calibration(m::Motor)
    stop_calibration = 1
    query(m.motor_controller, 13, stop_calibration, m.id, 0)
end

function is_calibrated!(m::Motor)::Nothing #use after motor/encoder positions have been updated
    try
        steps = get_axis_parameter(m.motor_controller, 196, m.id)
        if abs(m.encoder_position - m.motor_position) ≤ relative_motor_encoder_tol && steps != 0
            m.calibrated = true
        else
            m.calibrated = false
        end
    catch err
        @warn err
        m.calibrated = false
    end
    nothing
end

function print_position(m, pos, alt_units)
    print_pos = ismissing(alt_units) ? pos : ustrip(uconvert(alt_units, pos*unit(m)))
    round(print_pos, digits = 2) * (ismissing(alt_units) ? unit(m) : alt_units)
end

#note that is block = false is set, motors will need to be manually updated
function move_to(m::Motor, position::Value; force::Bool = false, alt_units = missing, block = true)::Nothing
    update_motor_position!(m)
    update_encoder_position!(m)
    is_calibrated!(m)
    if !force
        range = ismissing(m.manual_range) ? m.axis_range : m.manual_range
        axorm = ismissing(m.manual_range) ? "axis" : "manual"
        if !(position in range)
        	@warn "New position is not within $axorm range: $range. Ignored move command."
        	return nothing
        elseif !m.safe_to_move
        	@warn "$(m.name) is not safe to move. Ignored move command."
        	return nothing
        elseif !m.calibrated
            @warn "$(m.name) is not calibrated. Ignored move command."
            return nothing
        end
    end
    init_pos = force ? m.encoder_position : m.motor_position
    move_to(m.motor_controller, m.id, convert_unit_to_steps(m,position))
    if block
        prog = Progress(1001, desc = "Moving $(m.name) 🐾 ", barlen = 20)#barglyphs=BarGlyphs(" 🐾   "))
        final_check = 0
        while final_check == 0
          sleep(0.5)
          try
              force ? update_encoder_position!(m) : update_motor_position!(m)
              temp_pos = force ? m.encoder_position : m.motor_position
              sleep(0.05)
              target_reached = get_axis_parameter(m.motor_controller, 8, m.id)
              sleep(0.05)
              vel = get_axis_parameter(m.motor_controller, 3, m.id)
              update!(prog, position == init_pos ? 1000 : Int(round(1000*(temp_pos - init_pos)/(position-init_pos))); showvalues = [("Motor Position", print_position(m, temp_pos, alt_units))])
              if vel == 0 || target_reached == 1
                  final_check = 1
              end
          catch err
              @warn err
          end
        end
        update_motor_position!(m)
        update_encoder_position!(m)
        temp_pos = force ? m.encoder_position : m.motor_position
        update!(prog, position == init_pos ? 1000 : Int(round(1000*(temp_pos - init_pos)/(position-init_pos))); showvalues = [("Motor Position", print_position(m, temp_pos, alt_units))])
        finish!(prog, desc = "$(m.name) is at: $(print_position(m, m.motor_position, alt_units)) | ")
        is_calibrated!(m)
    end
end

move_to(m::Motor, position::Quantity; force::Bool = false) = move_to(m, ustrip(uconvert(unit(m),position)), force = force, alt_units = Unitful.unit(position))

function move(m::Motor, δ::Value; force::Bool = false, alt_units = missing)
    update_motor_position!(m)
    pos = m.motor_position
    move_to(m, pos + δ, force = force, alt_units = alt_units)
end

move(m::Motor, δ::Quantity; force::Bool=false) = move(m, ustrip(uconvert(unit(m),δ)), force = force, alt_units = unit(δ))

function target_position_reached(m::Motor)::Bool
    flag = false
    try
        flag = get_axis_parameter(m.motor_controller, 8, m.id)
    catch err
        @warn err
    end
    return flag
end

# This file is a part of TrinamicMotionControl.jl, licensed under the MIT License (MIT).


struct TMCM3110
    name::String
    ip::IPv4
    port::Int

    function TMCM3110(name::String, ip::IPv4, port::Int)
        device = new(name, ip, port)
        return device
    end

    function Base.show(io::IO, device::TMCM3110)
      for n in fieldnames(typeof(device))
        println(io, "$n: $(getfield(device,n))")
      end
    end
end

function encode_command(device::TMCM3110, m_address::Int, n_command::Int, n_type::Int, n_motor::Int, value::Signed)
  m_address = UInt8( m_address % (1<<8) )
  n_command = UInt8( n_command % (1<<8) )
  n_type    = UInt8( n_type % (1<<8)    )
  n_motor   = UInt8( n_motor % (1<<8)   )
  value     = Int32( value )
  values = [ parse(UInt8, bitstring(value)[ 1+(8*(i-1)) : 8+(8*(i-1)) ] , base = 2) for i in 1:4]

  checksum  = UInt8( (m_address + n_command + n_type + n_motor + sum(values)) % (1<<8) )

  tmcl_bytes = [m_address, n_command, n_type, n_motor, values..., checksum]

  return tmcl_bytes
end

function decode_reply(device::TMCM3110, reply)
  r_address  = UInt8( reply[1] )
  m_address  = UInt8( reply[2] )
  r_status   = UInt8( reply[3] )
  Int32(r_status) != 100 ? (@warn TMCM3110_STATUSCODES[Int(r_status)]) : nothing
  n_command  = UInt8( reply[4] )
  values     = [ UInt8(value) for value in reply[5:8] ]
  r_checksum = UInt8( reply[9] )
  checksum = UInt8( (r_address + m_address + r_status + n_command + sum(values)) % (1<<8) )
  value      = parse( UInt32, "$(bitstring(values[1]))$(bitstring(values[2]))$(bitstring(values[3]))$(bitstring(values[4]))" , base = 2 )
  value      = reinterpret(Int32, value)
  return value
end

function fetch(device::TMCM3110, m_address, n_command, n_type, n_motor, value; timeout=3.0)
    c = -1
    while c == -1
        try
            c = connect(device)
            break
        catch err
          @warn err
        end
        sleep(0.5)
    end
    cmd = encode_command(device, m_address, n_command, n_type, n_motor, value)
    write(c, cmd )
    t0 = time()
    t = 0.
    r=""
    task = @async read(c, 9)
    while t < timeout
        if task.state == :done break end
        t = time()-t0
        sleep(0.01)
    end
    close(c)
    if t >= timeout
        error("Timeout! Device `$(device.name)` did not answer.")
    else
        r = decode_reply(device, task.result)
        return r
    end
end

function get_axis_parameter(device::TMCM3110, n_axisparameter, n_motor)
    if in(n_axisparameter, TMCM3110_AXIS_PARAMETER.keys)
        if in(n_motor, [0,1,2])
            r = ""
            while r == ""
                try
                    r = fetch(device, 1, 6, n_axisparameter, n_motor, 0)
                catch err
                    @warn err 
                    sleep(0.5)
                end
            end
            return r
        else
            err("$n_motor is no a valid motor id. Nothing was done.")
        end
    else
        err("$n_axisparameter is no a valid axis parameter id. Nothing was done.")
    end
end

function set_axis_parameter(device::TMCM3110, n_axisparameter, n_motor, value)
  if in(n_axisparameter, TMCM3110_AXIS_PARAMETER.keys)
      if in(n_motor, [0,1,2])
          r = fetch(device, 1, 5, n_axisparameter, n_motor, value)
          return r
      else
          err("$n_motor is no a valid motor id. Nothing was done.")
      end
  else
      err("$n_axisparameter is no a valid axis parameter id. Nothing was done.")
  end
end

function store_axis_parameter_permanent(device::TMCM3110, n_axisparameter, n_motor, value)
    if in(n_axisparameter, TMCM3110_AXIS_PARAMETER.keys)
        if in(n_motor, [0,1,2])
            set_axis_parameter(device, n_axisparameter, n_motor, value) # first set the parameter to value,
            r = fetch(device, 1, 7, n_axisparameter, n_motor, 0)        # then store it permanent
        else
            err("$n_motor is no a valid motor id. Nothing was done.")
        end
    else
        err("$n_axisparameter is no a valid axis parameter id. Nothing was done.")
    end
end

function list_all_axis_parameters(device::TMCM3110; pars = : )
    for key in sort(collect(keys(TMCM3110_AXIS_PARAMETER)))[pars] # wrong oder
      axis_parameters = Any[0,0,0]
      for i in 1:3
        axis_parameters[i] = get_axis_parameter(device, key, i-1)
      end
      if length(TMCM3110_AXIS_PARAMETER[key]) <= 12
        @info "$key  \t- $(TMCM3110_AXIS_PARAMETER[key]):\t\t\t$(axis_parameters[1])\t\t$(axis_parameters[2])\t\t$(axis_parameters[3])"
      elseif 12 < length(TMCM3110_AXIS_PARAMETER[key]) <= 20
        @info "$key  \t- $(TMCM3110_AXIS_PARAMETER[key]):\t\t$(axis_parameters[1])\t\t$(axis_parameters[2])\t\t$(axis_parameters[3])"
      else
        @info "$key  \t- $(TMCM3110_AXIS_PARAMETER[key]):\t$(axis_parameters[1])\t\t$(axis_parameters[2])\t\t$(axis_parameters[3])"
      end
    end
    return nothing
end

list_motion_axis_parameters(device::TMCM3110) = list_all_axis_parameters(device::TMCM3110; pars = 1:4 )

function move_to(device::TMCM3110, n_motor, value)
    try
        position = convert(Int32, value)
    catch
        err("value must be convertible to Int32")
    end
    if n_motor in [0,1,2]
		r = 0
		t = 0
		while t < 10
	        try
	            r = fetch(device, 1, 4, 0, n_motor, convert(Int,value))  
				break
		    catch err
	             @warn err
	        end
			sleep(1)
			t += 1
		end
		if t == 10
		    error("Motor controller did not answer to move command.") 
	    else
		    return r
		end
    else
        error("$n_motor is no a valid motor id. Must be in [0,1,2]. Nothing was done.")
    end
end

function stop(device::TMCM3110; n_motor=-1)
    if in(n_motor, [0,1,2])
        r = fetch(device, 1, 3, 0, n_motor, 0)
        return r
    else
        r = Int[]
        for m in [0,1,2]
            push!(r, fetch(device, 1, 3, 0, m, 0))
        end
        return r
    end
end

TMCM3110_STATUSCODES = Dict( 100 => "Succesfully executed, no error",
                    101 => "Command loaded into TMCL program EEPROM",
                      1 => "Wrong Checksum",
                      2 => "Invalid command",
                      3 => "Wrong type",
                      4 => "Invalid value",
                      5 => "Configuration EEPROM locked",
                      6 => "Command not available" )

TMCM3110_COMMAND_NUMBERS = Dict(  1 => "ROR",
                         2 => "ROL",
                         3 => "MST",
                         4 => "MVP",
                         5 => "SAP",
                         6 => "GAP",
                         7 => "STAP",
                         8 => "RSAP",
                         9 => "SGP",
                        10 => "GGP",
                        11 => "STGP",
                        12 => "RSGP",
                        13 => "RFS",
                        14 => "SIO",
                        15 => "GIO",
                        19 => "CALC",
                        20 => "COMP",
                        21 => "JC",
                        22 => "JA",
                        23 => "CSUB",
                        24 => "RSUB",
                        25 => "EI",
                        26 => "DI",
                        27 => "WAIT",
                        28 => "STOP",
                        30 => "SCO",
                        31 => "GCO",
                        32 => "CCO",
                        33 => "CALCX",
                        34 => "AAP",
                        35 => "AGP",
                        37 => "VECT",
                        38 => "RETI",
                        39 => "ACO"  )

TMCM3110_AXIS_PARAMETER = Dict(   0 => "target position",
                         1 => "actual position",
                         2 => "target speed",
                         3 => "actual speed",
                         4 => "max positioning speed",
                         5 => "max acceleration",
                         6 => "abs max current",
                         7 => "standby current",
                         8 => "target pos reached",
                         9 => "ref switch status",
                        10 => "right limit switch status",
                        11 => "left limit switch status",
                        12 => "right limit switch disable",
                        13 => "left limit switch disable",
                       130 => "minimum speed",
                       135 => "actual acceleration",
                       138 => "ramp mode",
                       140 => "microstep resolution",
                       141 => "ref switch tolerance",
                       149 => "soft stop flag",
                       153 => "ramp divisor",
                       154 => "pulse divisor",
                       160 => "step interpolation enable",
                       161 => "double step enable",
                       162 => "chopper blank time",
                       163 => "chopper mode",
                       164 => "chopper hysteresis dec",
                       165 => "chopper hysteresis end",
                       166 => "chopper hysteresis start",
                       167 => "chopper off time",
                       168 => "smartEnergy min current",
                       169 => "smartEnergy current downstep",
                       170 => "smartEnergy hysteresis",
                       171 => "smartEnergy current upstep",
                       172 => "smartEnergy hysteresis start",
                       173 => "stallGuard2 filter enable",
                       174 => "stallGuard2 threshold",
                       175 => "slope control high side",
                       176 => "slope control low side",
                       177 => "short protection disable",
                       178 => "short detection timer",
                       179 => "Vsense",
                       180 => "smartEnergy actual current",
                       181 => "stop on stall",
                       182 => "smartEnergy threshold speed",
                       183 => "smartEnergy slow run current",
                       193 => "ref. search mode",
                       194 => "ref. search speed",
                       195 => "ref. switch speed",
                       196 => "distance end switches",
					   201 => "encoder mode",
                       204 => "freewheeling",
                       206 => "actual load value",
                       208 => "TMC262 errorflags",
                       209 => "encoder pos",
                       210 => "encoder prescaler",
                       212 => "encoder max deviation",
                       214 => "power down delay"    )

TMCM3110_INTERRUPT_VECTORS = Dict(  0 => "Timer 0",
                           1 => "Timer 1",
                           2 => "Timer 2",
                           3 => "Target position 0 reached",
                           4 => "Target position 1 reached",
                           5 => "Target position 2 reached",
                          15 => "stallGuard2 axis 0",
                          16 => "stallGuard2 axis 1",
                          17 => "stallGuard2 axis 2",
                          21 => "Deviation axis 0",
                          22 => "Deviation axis 1",
                          23 => "Deviation axis 2",
                          27 => "Left stop switch 0",
                          28 => "Right stop switch 0",
                          29 => "Left stop switch 1",
                          30 => "Right stop switch 1",
                          31 => "Left stop switch 2",
                          32 => "Right stop switch 2",
                          39 => "Input change 0",
                          40 => "Input change 1",
                          41 => "Input change 2",
                          42 => "Input change 3",
                          43 => "Input change 4",
                          44 => "Input change 5",
                          45 => "Input change 6",
                          46 => "Input change 7",
                         255 => "Global interrupts" )
                    
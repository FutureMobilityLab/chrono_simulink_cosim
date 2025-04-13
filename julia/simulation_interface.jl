# This script calls a simple C++ library with Julia.

# Path to the compiled DLL
const lib_path = joinpath(@__DIR__, "..", "build", "Release", "simulation_interface_c_api.dll")

"""
This example shows how to call C++ code from Julia using ccall.
We're interfacing with the SimulationInterface class defined in simulation_interface.cpp.
"""

# Define the same enum values as in the C++ header (Julia is 1-indexed).
# This is used solely for array indexing.
module InterfaceOutput
    const CHASSIS_POS_X = 1
    const CHASSIS_POS_Y = 2
    const CHASSIS_POS_Z = 3
    const CHASSIS_ORIENT_X = 4
    const CHASSIS_ORIENT_Y = 5
    const CHASSIS_ORIENT_Z = 6
    const CHASSIS_VEL_X = 7
    const CHASSIS_VEL_Y = 8
    const CHASSIS_VEL_Z = 9
    const CHASSIS_ANG_VEL_X = 10
    const CHASSIS_ANG_VEL_Y = 11
    const CHASSIS_ANG_VEL_Z = 12
    const CHASSIS_ACC_X = 13
    const CHASSIS_ACC_Y = 14
    const CHASSIS_ACC_Z = 15
    const CHASSIS_ANG_ACC_X = 16
    const CHASSIS_ANG_ACC_Y = 17
    const CHASSIS_ANG_ACC_Z = 18
    const WHEEL_ANG_VEL_FL = 19
    const WHEEL_ANG_VEL_FR = 20
    const WHEEL_ANG_VEL_RL = 21
    const WHEEL_ANG_VEL_RR = 22
    const TIRE_LONG_SLIP_FL = 23
    const TIRE_LONG_SLIP_FR = 24
    const TIRE_LONG_SLIP_RL = 25
    const TIRE_LONG_SLIP_RR = 26
    const TIRE_LAT_SLIP_FL = 27
    const TIRE_LAT_SLIP_FR = 28
    const TIRE_LAT_SLIP_RL = 29
    const TIRE_LAT_SLIP_RR = 30
    const TIRE_FORCE_LONG_FL = 31
    const TIRE_FORCE_LONG_FR = 32
    const TIRE_FORCE_LONG_RL = 33
    const TIRE_FORCE_LONG_RR = 34
    const TIRE_FORCE_LAT_FL = 35
    const TIRE_FORCE_LAT_FR = 36
    const TIRE_FORCE_LAT_RL = 37
    const TIRE_FORCE_LAT_RR = 38
    const TIRE_FORCE_VERT_FL = 39
    const TIRE_FORCE_VERT_FR = 40
    const TIRE_FORCE_VERT_RL = 41
    const TIRE_FORCE_VERT_RR = 42
    const WHEEL_TORQUE_DRIVE_FL = 43
    const WHEEL_TORQUE_DRIVE_FR = 44
    const WHEEL_TORQUE_DRIVE_RL = 45
    const WHEEL_TORQUE_DRIVE_RR = 46
    const WHEEL_TORQUE_BRAKE_FL = 47
    const WHEEL_TORQUE_BRAKE_FR = 48
    const WHEEL_TORQUE_BRAKE_RL = 49
    const WHEEL_TORQUE_BRAKE_RR = 50
    const STEERING_PINION_ANGLE = 51
    const WHEEL_STEER_ANG_FL = 52
    const WHEEL_STEER_ANG_FR = 53
    const WHEEL_STEER_ANG_RL = 54
    const WHEEL_STEER_ANG_RR = 55
    const LENGTH = 52
end

module InterfaceInput
    const STEERING = 1
    const THROTTLE = 2
    const BRAKE = 3
    const LENGTH = 4
end

# Define errors for simulation interface
struct SimulationError <: Exception
    msg::String
end

struct LibraryLoadError <: Exception
    path::String
    msg::String
end

struct InterfaceCreationError <: Exception
    msg::String
end

struct StepError <: Exception
    msg::String
end

# Define a mutable struct to hold a pointer to the SimulationInterface object
mutable struct SimulationInterface
    ptr::Ptr{Cvoid}
    
    # Constructor
    function SimulationInterface(vehicle_model_name::String)
        
        # Create a new SimulationInterface object
        ptr = ccall((:CreateSimulationInterface, lib_path), Ptr{Cvoid}, (Cstring,), vehicle_model_name)

        if ptr == C_NULL
            throw(InterfaceCreationError("Failed to create SimulationInterface with vehicle_model_name: $vehicle_model_name"))
        end
        
        obj = new(ptr)
        
        # Set finalizer to clean up the C++ object when Julia object is garbage collected
        finalizer(free, obj)
        return obj
    end
end

# Function to free the C++ object
function free(obj::SimulationInterface)
    if obj.ptr != C_NULL
        try
            ccall((:DestroySimulationInterface, lib_path), Cvoid, (Ptr{Cvoid},), obj.ptr)
        catch e
            @warn "Error when destroying SimulationInterface: $(e)"
        finally
            obj.ptr = C_NULL
        end
    end
end

# Function to call the step method
function step(obj::SimulationInterface, input::Vector{Float64})
    if obj.ptr == C_NULL
        throw(SimulationError("Cannot call step on invalid SimulationInterface"))
    end

    if length(input) != InterfaceInput.LENGTH - 1
        throw(ArgumentError("Input vector must have exactly $(InterfaceInput.LENGTH - 1) elements"))
    end

    # Create an output array with the correct size from the enum
    output = zeros(Float64, InterfaceOutput.LENGTH - 1)

    # Call the C++ step function with error handling
    try
        ccall((:Step, lib_path), Cvoid, (Ptr{Cvoid}, Ptr{Cdouble}, Ptr{Cdouble}), obj.ptr, input, output)
    catch e
        throw(StepError("Error during simulation step: $e"))
    end

    return output
end

# Example usage
function main()
    try
        println("Creating SimulationInterface object...")
        interface = SimulationInterface("sedan")

        println("\nCalling step with sample inputs...")
        input = [0.0, 1.0, 0.0]  # steering, throttle, brake
        duration = 10.0
        dt = 2e-3
        num_steps = floor(Int64, duration / dt)
        output = zeros(Float64, (num_steps, InterfaceOutput.LENGTH - 1))
        for i = 1:num_steps
            output[i,:] = step(interface, input)
        end
        println("\nDone!")
        println("Final State:")
        println("Output:")
        println("Vehicle State:")
        println("  Position (x,y,z) = ($(output[end, InterfaceOutput.CHASSIS_POS_X]), $(output[end, InterfaceOutput.CHASSIS_POS_Y]), $(output[end, InterfaceOutput.CHASSIS_POS_Z]))")
        println("  Velocity (x,y,z) = ($(output[end, InterfaceOutput.CHASSIS_VEL_X]), $(output[end, InterfaceOutput.CHASSIS_VEL_Y]), $(output[end, InterfaceOutput.CHASSIS_VEL_Z]))")
        
        println("\nTire Forces:")
        println("  Front Left:")
        println("    Longitudinal = $(output[end, InterfaceOutput.TIRE_FORCE_LONG_FL])")
        println("    Lateral = $(output[end, InterfaceOutput.TIRE_FORCE_LAT_FL])")
        println("    Vertical = $(output[end, InterfaceOutput.TIRE_FORCE_VERT_FL])")
        println("    Slip Angle = $(output[end, InterfaceOutput.TIRE_LAT_SLIP_FL])")
        
        println("\nWheel State:")
        println("  Angular Velocities:")
        println("    FL = $(output[end, InterfaceOutput.WHEEL_ANG_VEL_FL])")
        println("    FR = $(output[end, InterfaceOutput.WHEEL_ANG_VEL_FR])")
        println("  Drive Torques:")
        println("    FL = $(output[end, InterfaceOutput.WHEEL_TORQUE_DRIVE_FL])")
        println("    FR = $(output[end, InterfaceOutput.WHEEL_TORQUE_DRIVE_FR])")
        
        println("\nSteering:")
        println("  Pinion Angle = $(output[end, InterfaceOutput.STEERING_PINION_ANGLE])")
    
    catch e
        if isa(e, LibraryLoadError)
            println("Failed to load library: $(e.msg)")
        elseif isa(e, InterfaceCreationError)
            println("Failed to create interface: $(e.msg)")
        elseif isa(e, StepError)
            println("Error during simulation step: $(e.msg)")
        elseif isa(e, ArgumentError)
            println("Invalid argument: $(e.msg)")
        else
            println("Unexpected error: $e")
        end
        rethrow(e)
    end
end

# Check if the library exists
if !isfile(lib_path)
    error_msg = "Library not found at $lib_path. You need to compile the C++ code with the wrapper functions before running this script."
    @error error_msg
    throw(LibraryLoadError(lib_path, error_msg))
else
    # Run the example
    main()
end

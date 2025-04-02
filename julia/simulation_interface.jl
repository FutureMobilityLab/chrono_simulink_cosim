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
    const SUM = 1
    const DIFF = 2
    const LENGTH = 3
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
        input = [1.0, 2.0, 3.0]
        output = step(interface, input)
        println("Output:")
        println("  SUM = $(output[InterfaceOutput.SUM])")
        println("  DIFF = $(output[InterfaceOutput.DIFF])")
        
        println("\nDone!")
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

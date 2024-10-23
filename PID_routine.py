# PID Control Variables
Kp = 4
Ki = 2
Kd = 0
error = 0
last_error = 0
integral = 0
# Define the output limits
output_min = 0
output_max = 65535
# Define the loop time and initial time
loop_time = 0.01

    # PID CONTROL
    current_time = time.monotonic()
    elapsed_time = current_time - last_time # Calculate the elapsed time
    # Read the process variable
    process_variable = amp_mean
    # Calculate the error
    error = current_test - process_variable
    print('Erorr is ' + str(error)) # PRINT for debugging
    # Calculate the integral term
    integral += error * elapsed_time
    # Calculate the derivative term
    derivative = (error - last_error) / elapsed_time
    # Calculate the output
    output = Kp * error + Ki * integral + Kd * derivative
    # Constrain the output to the output limits
    if output < output_min:
        output = output_min
    elif output > output_max:
        output = output_max
    # Set the duty cycle to the output commanded by the PID loop
    duty_cycle = output
    ssr_output.duty_cycle = int(duty_cycle) # Set duty cycle to match value commanded by PID
    print('Duty cycle now ' + str(duty_cycle) + ' / amp_mean now ' + str(amp_mean) + ' / setpoint is ' + str(current_test) + ' / avg current is ' + str(amp_avg) + ' / Test is ' + str(time_percent_completed) + ' % Complete')
    # Store the current time and error for the next loop
    last_time = current_time
    last_error = error
    # Wait for the loop time
    time.sleep(loop_time)

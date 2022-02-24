defmodule Robota.Actions do

  require Logger
  use Bitwise
  alias Circuits.GPIO

  @sensor_pins [cs: 5, clock: 25, address: 24, dataout: 23]
  @ir_pins [dl: 19]
  @motor_pins [rb: 12, rf: 13, lf: 20, lb: 21]
  @pwm_pins [enl: 6, enr: 26]
  @servo_a_pin 27
  @servo_b_pin 22
  @servo_c_pin 18

  @smotor_pins [in1: 2, in2: 3, in3: 4, in4: 17]

  @ref_atoms [:cs, :clock, :address, :dataout]
  @lf_sensor_data %{sensor0: 0, sensor1: 0, sensor2: 0, sensor3: 0, sensor4: 0, sensor5: 0}
  @lf_sensor_map %{0 => :sensor0, 1 => :sensor1, 2 => :sensor2, 3 => :sensor3, 4 => :sensor4, 5 => :sensor5}

  @forward [0, 1, 1, 0]
  @left [0, 1, 0, 1]
  @right [1, 0, 1, 0]
  @backward [1, 0, 0, 1]
  @stop [0, 0, 0, 0]
  @sleft [0, 1, 0, 0]
  @sright [0, 0, 1, 0]

 #@duty_cycles [150, 70, 0]
  @pwm_frequency 50
  @pwm_value 120
  @lim_val 800  

  def main(str) do
    # motor_ref = Enum.map(@motor_pins, fn {_atom, pin_no} -> GPIO.open(pin_no, :output) end)
    # smotor_ref = Enum.map(@smotor_pins, fn {_atom, pin_no} -> GPIO.open(pin_no, :output) end)
    # pwm_ref = Enum.map(@pwm_pins, fn {_atom, pin_no} -> GPIO.open(pin_no, :output) end)
    # Enum.map(pwm_ref,fn {_, ref_no} -> GPIO.write(ref_no, 1) end)
    # sensor_ref = Enum.map(@sensor_pins, fn {atom, pin_no} -> configure_sensor({atom, pin_no}) end)
    # sensor_ref = Enum.map(sensor_ref, fn{_atom, ref_id} -> ref_id end)
    # sensor_ref = Enum.zip(@ref_atoms, sensor_ref)
    # ir_ref = Enum.map(@ir_pins, fn {_atom, pin_no} -> GPIO.open(pin_no, :input, pull_mode: :pullup) end)
    # #move(motor_ref, sensor_ref, 0, 0)   
    # cond do
    #   #str == "move" -> move(motor_ref, sensor_ref, 0, 0)
    #   str == "move" -> move(motor_ref, sensor_ref, 0)
    #   str == "right" -> turn(motor_ref, sensor_ref, "right", 0)
    #   str == "left" -> turn(motor_ref, sensor_ref, "left", 0)
    #   str == "sow" -> sowing(smotor_ref)
    #   str == "weedr" -> weeding("right")
    #   str == "weedl" -> weeding("left")
    #   str == "depositr" -> depo_("right")
    #   str == "depositl" -> depo_("left")
    #   str == "obs" -> get_ir(ir_ref)
    #   true -> nil
    # end
    # Enum.map(pwm_ref,fn {_, ref_no} -> GPIO.write(ref_no, 0) end)
  end
  
  def get_ir(ir_ref) do
    ir_values = Enum.map(ir_ref,fn {_, ref_no} -> GPIO.read(ref_no) end)
    if ir_values == [0] do
     true
    else
     false
    end 
  end

  def turn(motor_ref, sensor_ref, side, state) do
    append_sensor_list = [0,1,2,3,4] ++ [5]
    temp_sensor_list = [5 | append_sensor_list]
    l = append_sensor_list
        |> Enum.with_index
        |> Enum.map(fn {sens_num, sens_idx} ->
              analog_read(sens_num, sensor_ref, Enum.fetch(temp_sensor_list, sens_idx))
              end)
    Enum.each(0..5, fn n -> provide_clock(sensor_ref) end)
    GPIO.write(sensor_ref[:cs], 1)
    ls = Enum.at(l, 4)
    cs = Enum.at(l, 3)
    rs = Enum.at(l, 2)
    rrs = Enum.at(l, 1)
    lls = Enum.at(l, 0)
    
    if state == 0 do
     if side == "right" do
      motor_action(motor_ref, @sright)
      motion_pwm(@pwm_value)
     else
      motor_action(motor_ref, @sleft)
      motion_pwm(@pwm_value)
     end      
     Process.sleep(300)
     turn(motor_ref, sensor_ref, side, 1)
    else
     if cs>@lim_val do
     motor_action(motor_ref, @stop)
     else
      if side == "right" do
        motor_action(motor_ref, @sright)
        motion_pwm(@pwm_value)
      else
        motor_action(motor_ref, @sleft)
        motion_pwm(@pwm_value)
      end
      turn(motor_ref, sensor_ref, side, 1)
     end
    end

 end

#  def turn(motor_ref, sensor_ref, side, state) do
#     append_sensor_list = [0,1,2,3,4] ++ [5]
#     temp_sensor_list = [5 | append_sensor_list]
#     l = append_sensor_list
#         |> Enum.with_index
#         |> Enum.map(fn {sens_num, sens_idx} ->
#               analog_read(sens_num, sensor_ref, Enum.fetch(temp_sensor_list, sens_idx))
#               end)
#     Enum.each(0..5, fn n -> provide_clock(sensor_ref) end)
#     ls = Enum.at(l, 4)
#     cs = Enum.at(l, 3)
#     rs = Enum.at(l, 2)
#     rrs = Enum.at(l, 1)
#     lls = Enum.at(l, 0)
     
#     if cs >@lim_val && state == 0 do
#      if side == "right" do
#        Pigpiox.Pwm.gpio_pwm(20, @pwm_value - 10)
#     Pigpiox.Pwm.gpio_pwm(13, 0)
#      else
#         Pigpiox.Pwm.gpio_pwm(20, 0)
#     Pigpiox.Pwm.gpio_pwm(13, @pwm_value - 10)
#      end      
#      turn(motor_ref, sensor_ref, side, 0)
#     else
#      if cs >@lim_val do
#      motor_action(motor_ref, @stop)
#      motion_pwm(0)
#      else
#       if side == "right" do
#          Pigpiox.Pwm.gpio_pwm(20, @pwm_value - 10)
#     Pigpiox.Pwm.gpio_pwm(13, 0)
#       else
#         Pigpiox.Pwm.gpio_pwm(20, 0)
#     Pigpiox.Pwm.gpio_pwm(13, @pwm_value - 10)
        
#       end
#       turn(motor_ref, sensor_ref, side, 1)
#      end
#     end

#  end

 def move(motor_ref, sensor_ref, state) do
    append_sensor_list = [0,1,2,3,4] ++ [5]
    temp_sensor_list = [5 | append_sensor_list]
    l = append_sensor_list
        |> Enum.with_index
        |> Enum.map(fn {sens_num, sens_idx} ->
              analog_read(sens_num, sensor_ref, Enum.fetch(temp_sensor_list, sens_idx))
              end)
    Enum.each(0..5, fn n -> provide_clock(sensor_ref) end)
    GPIO.write(sensor_ref[:cs], 1)
    ls = Enum.at(l, 4)
    cs = Enum.at(l, 3)
    rs = Enum.at(l, 2)
    rrs = Enum.at(l, 1) 
    lls = Enum.at(l, 0)
    
    if (((lls>@lim_val)&&(ls>@lim_val)&&(cs>@lim_val)) ||  ((rrs>@lim_val)&&(rs>@lim_val)&&(cs>@lim_val))) && state == 0 do
      motor_action(motor_ref, @forward)
      motion_pwm(@pwm_value)
      move(motor_ref, sensor_ref, 0)
    else
    cond do
    ((lls>@lim_val)&&(ls>@lim_val)&&(cs>@lim_val)) ||  ((rrs>@lim_val)&&(rs>@lim_val)&&(cs>@lim_val)) -> motor_action(motor_ref, @stop)             
    cs<@lim_val -> if (ls>@lim_val || lls>@lim_val) do
		motor_action(motor_ref, @sright)
	        motion_pwm(@pwm_value)
                move(motor_ref, sensor_ref, 1)
              else
		motor_action(motor_ref, @sleft)
	        motion_pwm(@pwm_value)
                move(motor_ref, sensor_ref, 1)
              end
    true ->
       motor_action(motor_ref, @forward)
       motion_pwm(@pwm_value)
       move(motor_ref, sensor_ref, 1) 
    end
    end
 end

#  def move(motor_ref, sensor_ref, state, pError) do
#     append_sensor_list = [0,1,2,3,4] ++ [5]
#     temp_sensor_list = [5 | append_sensor_list]
#     l = append_sensor_list
#         |> Enum.with_index
#         |> Enum.map(fn {sens_num, sens_idx} ->
#               analog_read(sens_num, sensor_ref, Enum.fetch(temp_sensor_list, sens_idx))
#               end)
#     Enum.each(0..5, fn n -> provide_clock(sensor_ref) end)
#     ls = Enum.at(l, 4)
#     cs = Enum.at(l, 3)
#     rs = Enum.at(l, 2)
#     rrs = Enum.at(l, 1) 
#     lls = Enum.at(l, 0)
    
#     if (((lls>@lim_val)&&(ls>@lim_val)&&(cs>@lim_val)) ||  ((rrs>@lim_val)&&(rs>@lim_val)&&(cs>@lim_val))) && state == 0 do
#       #if (ls+cs+rs+rrs+lls)/5 >= 550 && state == 0 do
#       motor_action(motor_ref, @forward)
#       motion_pwm(@pwm_value)
#       move(motor_ref, sensor_ref, 0, pError)
#     else
#     cond do
#     #(ls+cs+rs+rrs+lls)/5 >= 550 ->  
#     ((lls>@lim_val)&&(ls>@lim_val)&&(cs>@lim_val)) ||  ((rrs>@lim_val)&&(rs>@lim_val)&&(cs>@lim_val)) -> 
# # || ((ls>@lim_val)&&(lls>@lim_val)) || ((rs>@lim_val)&&(rrs>@lim_val)) ->
#     motion_pwm(0) 
#     motor_action(motor_ref, @stop)             
#     true ->
#        error = getError(lls, ls, cs, rs, rrs)
#        {pError, pid} = calculatePID(error, pError)
#        #IO.inspect(pid)
#        motorPIDcontrol(pid, motor_ref)
#        move(motor_ref, sensor_ref, 1, pError)
#     end
#     end
#  end
 
#   def getError(lls, ls, cs, rs, rrs) do
#     cond do
# 	lls < @lim_val && ls < @lim_val && cs < @lim_val && rs < @lim_val && rrs > @lim_val -> 4
# 	lls < @lim_val && ls < @lim_val && cs < @lim_val && rs > @lim_val && rrs > @lim_val -> 3
# 	lls < @lim_val && ls < @lim_val && cs < @lim_val && rs > @lim_val && rrs < @lim_val -> 2
# 	lls < @lim_val && ls < @lim_val && cs > @lim_val && rs > @lim_val && rrs < @lim_val -> 1
# 	lls < @lim_val && ls > @lim_val && cs > @lim_val && rs < @lim_val && rrs < @lim_val -> -1
# 	lls < @lim_val && ls > @lim_val && cs < @lim_val && rs < @lim_val && rrs < @lim_val -> -2
# 	lls > @lim_val && ls > @lim_val && cs < @lim_val && rs < @lim_val && rrs < @lim_val -> -3
# 	lls > @lim_val && ls < @lim_val && cs < @lim_val && rs < @lim_val && rrs < @lim_val -> -4
# 	true -> 0
#     end
#   end

#   def calculatePID(error, pError) do
#     p = error
#     d = error-pError
#     pError = error
#     pidvalue = (17*p) + (17*d)
#     {pError, pidvalue}
#  end

#   def motorPIDcontrol(pid, motor_ref) do
#     leftMotorSpeed = @pwm_value - pid
#     rightMotorSpeed = @pwm_value + pid
#     Pigpiox.Pwm.gpio_pwm(20, leftMotorSpeed)
#     Pigpiox.Pwm.gpio_pwm(13, rightMotorSpeed)
#   end

  def test_servo_a(angle) do
    val = trunc(((2.5 + 10.0 * angle / 150) / 100 ) * 255)
    Pigpiox.Pwm.set_pwm_frequency(@servo_a_pin, @pwm_frequency)
    Pigpiox.Pwm.gpio_pwm(@servo_a_pin, val)
  end

  def test_servo_b(angle) do
    val = trunc(((2.5 + 10.0 * angle / 150) / 100 ) * 255)
    Pigpiox.Pwm.set_pwm_frequency(@servo_b_pin, @pwm_frequency)
    Pigpiox.Pwm.gpio_pwm(@servo_b_pin, val)
  end
  
  def test_servo_c(angle) do
    val = trunc(((2.5 + 10.0 * angle / 150) / 100 ) * 255)
    Pigpiox.Pwm.set_pwm_frequency(@servo_c_pin, @pwm_frequency)
    Pigpiox.Pwm.gpio_pwm(@servo_c_pin, val)
  end

def sowing(smotor_ref) do
  IO.puts("smotor connecting")
  smotor_action(smotor_ref, [1, 0, 0, 1])
  Process.sleep(5)
  smotor_action(smotor_ref, [1, 0, 0, 0])
  Process.sleep(5)
  smotor_action(smotor_ref, [1, 1, 0, 0])
  Process.sleep(5)
  smotor_action(smotor_ref, [0, 1, 0, 0])
  Process.sleep(5)
  smotor_action(smotor_ref, [0, 1, 1, 0])
  Process.sleep(5)
  smotor_action(smotor_ref, [0, 0, 1, 0])
  Process.sleep(5)
  smotor_action(smotor_ref, [0, 0, 1, 1])
  Process.sleep(5)
  smotor_action(smotor_ref, [0, 0, 0, 1])
  Process.sleep(5)
  sowing(smotor_ref)
  IO.puts("smotor connected")
end

def weeding(dir) do
  test_servo_a(40)
  Process.sleep(700)
  test_servo_c(20)
  Process.sleep(700)
  if dir == "right" do
   test_servo_b(0)
  else
   test_servo_b(180)
  end
  Process.sleep(1000)
  test_servo_a(20)
  Process.sleep(700)
  test_servo_c(85)
  Process.sleep(700)
  test_servo_a(40)
  Process.sleep(700)
  test_servo_b(90)
end

  def depo_(dir) do
    if dir == "left" do
      test_servo_b(180)
    else
      test_servo_b(0)
     end
     Process.sleep(1000)
     test_servo_c(20)
     Process.sleep(1000)
     test_servo_b(90)
   end
   
  defp configure_sensor({atom, pin_no}) do
   if (atom == :dataout) do
      GPIO.open(pin_no, :input, pull_mode: :pullup)
    else
      GPIO.open(pin_no, :output)
    end
  end

  defp analog_read(sens_num, sensor_ref, {_, sensor_atom_num}) do

    GPIO.write(sensor_ref[:cs], 0)
    %{^sensor_atom_num => sensor_atom} = @lf_sensor_map
    Enum.reduce(0..9, @lf_sensor_data, fn n, acc ->
        read_data(n, acc, sens_num, sensor_ref, sensor_atom_num)
        |> clock_signal(n, sensor_ref)
      end)[sensor_atom]
  end

  defp provide_clock(sensor_ref) do
    GPIO.write(sensor_ref[:clock], 1)
    GPIO.write(sensor_ref[:clock], 0)
  end

  defp read_data(n, acc, sens_num, sensor_ref, sensor_atom_num) do
    if (n < 4) do

      if (((sens_num) >>> (3 - n)) &&& 0x01) == 1 do
        GPIO.write(sensor_ref[:address], 1)
      else
        GPIO.write(sensor_ref[:address], 0)
      end
      Process.sleep(1)
    end

    %{^sensor_atom_num => sensor_atom} = @lf_sensor_map
    if (n <= 9) do
      Map.update!(acc, sensor_atom, fn sensor_atom -> ( sensor_atom <<< 1 ||| GPIO.read(sensor_ref[:dataout]) ) end)
    end
  end

  defp clock_signal(acc, n, sensor_ref) do
    GPIO.write(sensor_ref[:clock], 1)
    GPIO.write(sensor_ref[:clock], 0)
    acc
  end

  defp motor_action(motor_ref,motion) do
    motor_ref |> Enum.zip(motion) |> Enum.each(fn {{_, ref_no}, value} -> GPIO.write(ref_no, value) end)
  end

  defp smotor_action(smotor_ref,motion) do
    smotor_ref |> Enum.zip(motion) |> Enum.each(fn {{_, ref_no}, value} -> GPIO.write(ref_no, value) end)
  end

  defp motion_pwm(value) do
    pwm(value)
  end

  defp pwm(duty) do
    Enum.each(@pwm_pins, fn {_atom, pin_no} -> Pigpiox.Pwm.gpio_pwm(pin_no, duty) end)
  end

end


# Team id - 2095
# File name - phx_socket_client.ex
#theme - Functional Weeder
#functions - connect_server, wait_until_connected, send_robot_status, get_goals,
#            get_start_pos, init_server_variables, stop_process, get_updated_goals,
#            update_goals, get_b_pos, send_for_eval

defmodule Robota.PhoenixSocketClient do

  alias PhoenixClient.{Socket, Channel, Message}

  @doc """
  Connect to the Phoenix Server URL (defined in config.exs) via socket.
  Once ensured that socket is connected, join the channel on the server with topic "robot:status".
  Get the channel's PID in return after joining it.

  NOTE:
  The socket will automatically attempt to connect when it starts.
  If the socket becomes disconnected, it will attempt to reconnect automatically.
  Please note that start_link is not synchronous,
  so you must wait for the socket to become connected before attempting to join a channel.
  Reference to above note: https://github.com/mobileoverlord/phoenix_client#usage

  You may refer: https://github.com/mobileoverlord/phoenix_client/issues/29#issuecomment-660518498
  """

  # connects to the server and joins the channel 

  def connect_server do

    ###########################
    ## complete this funcion ##
    ###########################
    Agent.start_link(fn -> "running" end, name: :rh)
    url = Application.get_env(:robota, :phoenix_server_url)
    socket_opts = [
      url: url
    ]
    {:ok, socket} = PhoenixClient.Socket.start_link(socket_opts)
    wait_until_connected(socket)
    {:ok, _response, channel} = PhoenixClient.Channel.join(socket, "robot:status")
    [channel, channel]
    ###########################
  end

  defp wait_until_connected(socket) do
  if !PhoenixClient.Socket.connected?(socket) do
    Process.sleep(10)
    wait_until_connected(socket)
  end

  end

  @doc """
  Send Toy Robot's current status i.e. location (x, y) and facing
  to the channel's PID with topic "robot:status" on Phoenix Server with the event named "new_msg".

  The message to be sent should be a Map strictly of this format:
  %{"client": < "robot_A" or "robot_B" >,  "x": < x_coordinate >, "y": < y_coordinate >, "face": < facing_direction > }

  In return from Phoenix server, receive the boolean value < true OR false > indicating the obstacle's presence
  in this format: {:ok, < true OR false >}.
  Create a tuple of this format: '{:obstacle_presence, < true or false >}' as a return of this function.
  """
  def send_robot_status(channel, %Robota.Position{x: x, y: y, facing: facing} = robot) do

    ###########################
    ## complete this funcion ##
    ###########################
    message = %{"client": "robot_A", "x": x, "y": y, "face": facing}
    {:ok, rs} = PhoenixClient.Channel.push(channel, "new_msg", message)
    if rs == 1 do
      if Agent.get(:rh, fn l -> l end) == "stopped" do
        Agent.update(:rh, fn l -> "running" end)
        send_for_eval(8, channel, "nil")
      end
      send_for_eval(1, channel, %{"x": x, "y": y, "face": facing})
      #obs = Robota.Actions.main("obs")
      obs = false
      if obs == true do
        send_for_eval(2, channel, %{"x": x, "y": y, "face": facing})
      end
      {:obstacle_presence, obs}
    else
    if Agent.get(:rh, fn l -> l end) == "running" do
      Agent.update(:rh, fn l -> "stopped" end)
      send_for_eval(7, channel, "nil")
    end
    Process.sleep(1000)
    send_robot_status(channel, robot)
    end
  end

  ######################################################
  ## You may create extra helper functions as needed. ##
  ######################################################

  # gets all the goals to be reached

  def get_goals(channel) do
    {:ok, g} = PhoenixClient.Channel.push(channel, "get_goals", [])
    g
  end

  # gets the starting positions from the server

  def get_start_pos(channel) do
    {:ok, g} = PhoenixClient.Channel.push(channel, "get_start_pos", "robotA")
    if g == [] do
      Process.sleep(1000)
      get_start_pos(channel)
    else
      g
    end
  end

  # initializes the server variables like goals, updated goals, etc.

  def init_server_variables(channel, gl) do
  {:ok, _s} = PhoenixClient.Channel.push(channel, "init", gl)
  end

  # stops the process after all the tasks are done

  def stop_process(channel) do
    {:ok, _s} = PhoenixClient.Channel.push(channel, "stop_process", ["A"])
    send_for_eval(9, channel, "nil")
  end

  # gets the updated goals if the other robot have complted a task

  def get_updated_goals(channel) do
    {:ok, gs} = PhoenixClient.Channel.push(channel, "get_upd_goals", "robotA")
    gs
  end

  # updates the goals if the robot completes a task

  def update_goals(channel, gl) do
    {:ok, _gs} = PhoenixClient.Channel.push(channel, "upd_goals", [gl, "robotA"])
  end

  #gets the current position of robot A to prevent clashing

  def get_b_pos(channel) do
    {:ok, pos} = PhoenixClient.Channel.push(channel, "get_pos", "robotB")
    pos
  end

  #sends messages for evaluation

  def send_for_eval(id, channel, data) do
   {:ok, _s} = PhoenixClient.Channel.push(channel, "event_msg", %{"event_id" => id, "sender" => "A", "value" => data})
  end

end

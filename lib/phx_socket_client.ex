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

  def get_goals(channel) do
    {:ok, g} = PhoenixClient.Channel.push(channel, "get_goals", [])
    g
  end

  def get_start_pos(channel) do
    {:ok, g} = PhoenixClient.Channel.push(channel, "get_start_pos", "robotA")
    if g == [] do
      Process.sleep(1000)
      get_start_pos(channel)
    else
      g
    end
  end

  def init_server_variables(channel, gl) do
  {:ok, _s} = PhoenixClient.Channel.push(channel, "init", gl)
  end

  def stop_process(channel) do
    {:ok, _s} = PhoenixClient.Channel.push(channel, "stop_process", ["A"])
    send_for_eval(9, channel, "nil")
  end

  def get_updated_goals(channel) do
    {:ok, gs} = PhoenixClient.Channel.push(channel, "get_upd_goals", "robotA")
    gs
  end

  def update_goals(channel, gl) do
    {:ok, _gs} = PhoenixClient.Channel.push(channel, "upd_goals", [gl, "robotA"])
  end

  def get_b_pos(channel) do
    {:ok, pos} = PhoenixClient.Channel.push(channel, "get_pos", "robotB")
    pos
  end

  def send_for_eval(id, channel, data) do
   {:ok, _s} = PhoenixClient.Channel.push(channel, "event_msg", %{"event_id" => id, "sender" => "A", "value" => data})
  end

end

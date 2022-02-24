defmodule RobotaTest do
  use ExUnit.Case
  doctest Robota

  test "greets the world" do
    assert Robota.hello() == :world
  end
end

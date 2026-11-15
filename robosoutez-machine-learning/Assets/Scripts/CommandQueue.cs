using Godot;
using System;
using System.Collections.Generic;

public partial class CommandQueue : Node
{
    private Queue<Action> _commandQueue = new Queue<Action>();
    private Action _currentCommand = null;
    private bool _isExecuting = false;
    private bool _commandComplete = false;

    // Reference to the robot through its interface
    private IRobot _robot;

    public override void _Process(double delta)
    {
        if (_isExecuting && _currentCommand != null && _commandComplete)
        {
            _commandComplete = false;
            _currentCommand = null;
            ProcessNextCommand();
        }
    }

    public bool IsExecuting()
    {
        return _isExecuting;
    }

    // Initialize with robot reference
    public void Initialize(IRobot robot)
    {
        _robot = robot;
    }

    // Add a single command to the queue
    public void EnqueueCommand(Action command)
    {
        _commandQueue.Enqueue(command);
    }

    // Add multiple commands at once
    public void EnqueueCommands(params Action[] commands)
    {
        foreach (var command in commands)
        {
            _commandQueue.Enqueue(command);
        }
    }

    // Start executing the queue
    public void StartExecution()
    {
        if (_commandQueue.Count > 0)
        {
            _isExecuting = true;
            ProcessNextCommand();
        }
        else
        {
            GD.Print("No commands in queue!");
        }
    }

    // Stop execution and clear queue
    public void StopExecution()
    {
        _isExecuting = false;
        _currentCommand = null;
        _commandQueue.Clear();
        GD.Print("Execution stopped!");
    }

    // Call this from your robot functions when they complete
    public void OnCommandComplete()
    {
        _commandComplete = true;
    }

    private void ProcessNextCommand()
    {
        if (_commandQueue.Count > 0)
        {
            _currentCommand = _commandQueue.Dequeue();
            _currentCommand.Invoke();
        }
        else
        {
            _isExecuting = false;
            GD.Print("All commands completed!");
        }
    }

    // Clear all commands without stopping execution
    public void ClearQueue()
    {
        _commandQueue.Clear();
    }

    // Get remaining commands count
    public int GetRemainingCommands()
    {
        return _commandQueue.Count;
    }

    // Get robot reference (useful for commands)
    public IRobot GetRobot()
    {
        return _robot;
    }
}

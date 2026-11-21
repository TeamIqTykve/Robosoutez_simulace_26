using Godot;
using RoboSoutezMachineLearning.Global;
using System;
using System.ComponentModel;
using System.Drawing;
using System.Reflection.Metadata;
using System.Security.Cryptography.X509Certificates;
using static System.Reflection.Metadata.BlobBuilder;

public partial class Robot : StaticBody3D, IRobot
{
	private CommandQueue _commandQueue;
	[Export] private HingeJoint3D _handjoint;
	private RayCast3D _colorSensor;
	private RayCast3D _wallSensor;
	private const float WHEEL_BASE = 0.1f; // m
	private const float MAX_SPEED = 0.17f;
	private const float MAX_HAND_SPEED = (float)Math.PI/2; // rad/sec
	private int _currentHandPosition = 0;
	private double _timeLeft = 0;
	private bool isNegative;
	public double _simulationtimeLeft = 90;
	int targetColor;
	int lineCount;
	int previousColor;
	private ROBOT_STATE _robotState = ROBOT_STATE.DOING_NOTHING;

	private LoadSave _loadSave;

	private enum ROBOT_STATE { 
		DOING_NOTHING,
		MOVING_FORWARD,
		MOVING_BACKWARD,
		TURNING,
		ROTATING_HAND,
		MOVING_UNTILL_LINE,
		MOVING_UNTILL_WALL,
		TURNING_AND_ROTATING
	}

	public static Robot Instance { get; private set; }

	public double LeftWheelSpeed { get; set; } // m/s
	public double RightWheelSpeed { get; set; } // m/s
	public float HandSpeed { get; set; } // rad/s
	public bool ButtonPressed { get; set; }

	public override void _Ready()
	{
		Instance = this;
		_colorSensor = this.GetNode<RayCast3D>("ColorSensor");
		_wallSensor = this.GetNode<RayCast3D>("WallSensor");

		_commandQueue = GetNode<CommandQueue>("CommandQueue");
		_commandQueue.Initialize(this);

		_loadSave = new LoadSave();
		this.AddChild(_loadSave);

		LoadCommandSequence();
	}

	private int CurrentColor()
	{
		if (_colorSensor.IsColliding())
		{
			var c = _colorSensor.GetCollider();
			if (c is StaticBody3D)
			{
				switch ((c as StaticBody3D).Name)
				{
					case "BlackLine":
						return 1;
					case "YellowLine":
						return 2;
					default:
						return 0;
			   }
			}
		}
		return 0;
	}

	private bool TouchingWall()
	{
		return _wallSensor.IsColliding();
	}

	private void LoadBlockPositions()
	{
		_loadSave.NodePaths.Clear();
		var blocks = this.GetTree().Root.GetNode("Simulation/Blocks").GetChildren();
		foreach (var block in blocks)
		{
			_loadSave.NodePaths.Add(block.GetPath());
		}
		_loadSave.NodePaths.Add(this.GetTree().Root.GetNode("Simulation/RobotPivot").GetPath());
		_loadSave.NodePaths.Add(this.GetTree().Root.GetNode("Simulation/RobotPivot/Robot").GetPath());
		_loadSave.NodePaths.Add(this.GetTree().Root.GetNode("Simulation/RobotPivot/HandR").GetPath());

		_loadSave.LoadTransforms("seq1.txt");
	}

	private void SaveBlockPositions()
	{
		_loadSave.NodePaths.Clear();
		var blocks = this.GetTree().Root.GetNode("Simulation/Blocks").GetChildren();
		foreach (var block in blocks)
		{
			_loadSave.NodePaths.Add(block.GetPath());
		}
		_loadSave.NodePaths.Add(this.GetTree().Root.GetNode("Simulation/RobotPivot").GetPath());
		_loadSave.NodePaths.Add(this.GetTree().Root.GetNode("Simulation/RobotPivot/Robot").GetPath());
		_loadSave.NodePaths.Add(this.GetTree().Root.GetNode("Simulation/RobotPivot/HandR").GetPath());
		_loadSave.NodePaths.Add(this.GetTree().Root.GetNode("Simulation/RobotPivot/HingeJoint3D").GetPath());
		_loadSave.SaveTransforms("seq1.txt");
	}

	public override void _PhysicsProcess(double delta)
	{
		//GD.Print("Position:", GlobalPosition);
		//if (Input.IsActionPressed("LoadBlocks"))
		//{
		//    LoadBlockPositions();
		//}

		if (Input.IsActionPressed("SaveBlocks"))
		{
			SaveBlockPositions();
		}

		double v = (RightWheelSpeed + LeftWheelSpeed) / 2;
		double om = (LeftWheelSpeed - RightWheelSpeed) / WHEEL_BASE;

		Vector3 fwd = GlobalTransform.Basis.X;
		Vector3 fwdXZ = new Vector3(fwd.X, 0, fwd.Z).Normalized();

		Vector3 V = fwdXZ * (float)v;
		LinearVelocity = V;t
		AngularVelocity = new Vector3(AngularVelocity.X, (float)-om, AngularVelocity.Z);

		switch (_robotState)
		{
			case ROBOT_STATE.MOVING_FORWARD:
			case ROBOT_STATE.MOVING_BACKWARD:
			case ROBOT_STATE.TURNING:
			case ROBOT_STATE.ROTATING_HAND:
			case ROBOT_STATE.TURNING_AND_ROTATING:
				_timeLeft -= delta;
				if (_timeLeft <= 0)
				{
					LeftWheelSpeed = RightWheelSpeed = 0;
					_handjoint.Set("motor/target_velocity", 0);
					_commandQueue.OnCommandComplete();
				}
				break;
			case ROBOT_STATE.MOVING_UNTILL_LINE:
				if (CurrentColor() == targetColor && previousColor != targetColor)
				{
					lineCount--;
				}
				if (lineCount == 0)
				{
					LeftWheelSpeed = RightWheelSpeed = 0;
					_commandQueue.OnCommandComplete();
				}
				previousColor = CurrentColor();
				break;
			case ROBOT_STATE.MOVING_UNTILL_WALL:
				if (TouchingWall())
				{
					LeftWheelSpeed = RightWheelSpeed = 0;
					_commandQueue.OnCommandComplete();
				}
				break;
			case ROBOT_STATE.DOING_NOTHING:
				break;
		}

		if (IsExecuting())
		{
			_simulationtimeLeft -= delta;
			//GD.Print(_simulationtimeLeft);
		}
	}

	public bool IsAlive()
	{
		return (_simulationtimeLeft <= 0);
	}
	public void MoveForward(double time)
	{
		LeftWheelSpeed = RightWheelSpeed = MAX_SPEED;
		_timeLeft = time;
		GD.Print("Moving Forward");
		_robotState = ROBOT_STATE.MOVING_FORWARD;
	}

	public void MoveBackward(double time)
	{
		LeftWheelSpeed = RightWheelSpeed = -MAX_SPEED;
		_timeLeft = time;
		GD.Print("Moving Backward");
		_robotState = ROBOT_STATE.MOVING_BACKWARD;
	}

	public void MoveUntillWall()
	{
		LeftWheelSpeed = RightWheelSpeed = MAX_SPEED;
		GD.Print("Moving untill collision");
		_robotState = ROBOT_STATE.MOVING_UNTILL_WALL;
	}

	public void RotateHand(int where)
	{
		if (_currentHandPosition == where)
			_commandQueue.OnCommandComplete();
		float alpha = 0;
		alpha = (where - _currentHandPosition) * (float)Math.PI / 2;
		_timeLeft = Math.Abs(alpha / MAX_HAND_SPEED);
		_currentHandPosition = where;

		if (alpha < 0)
			_handjoint.Set("motor/target_velocity", -MAX_HAND_SPEED);
		else
			_handjoint.Set("motor/target_velocity", MAX_HAND_SPEED);
		

		GD.Print($"Rotating hand to {where}");
		_robotState = ROBOT_STATE.ROTATING_HAND;
	}
	public void Turn(double angle)
	{
		if (angle == 0)
			_commandQueue.OnCommandComplete();

		float turnSpeed = (2*MAX_SPEED) / WHEEL_BASE;
		_timeLeft = Math.Abs(angle / turnSpeed);

		if (angle < 0)
		{
			LeftWheelSpeed = -MAX_SPEED;
			RightWheelSpeed = MAX_SPEED;
		}
		if (angle > 0) 
		{ 
				LeftWheelSpeed = MAX_SPEED;
				RightWheelSpeed = -MAX_SPEED;
		}

		GD.Print($"Turning by {angle}");
		_robotState = ROBOT_STATE.TURNING;
	}
	public void MoveUntillLine(int count, /*int color,*/ bool forward)
	{
		targetColor = 1;
		lineCount = count;
		GD.Print($"Moving untill {count} lines crossed");
		if (forward)
			LeftWheelSpeed = RightWheelSpeed = MAX_SPEED;
		else
			LeftWheelSpeed = RightWheelSpeed = -MAX_SPEED;

		GD.Print($"Moving untill {count} lines crossed");
		_robotState = ROBOT_STATE.MOVING_UNTILL_LINE;
	}
	public void TurnAndRotate(double angle, int where)
	{
		if (angle == 0)
			_commandQueue.OnCommandComplete();
		if (_currentHandPosition == where)
			_commandQueue.OnCommandComplete();

		float alpha = (where - _currentHandPosition) * (float)Math.PI / 2;
		_timeLeft = Math.Abs(alpha / MAX_HAND_SPEED);

		double speed = MAX_HAND_SPEED * WHEEL_BASE * 0.52;
		if (alpha < 0)
			_handjoint.Set("motor/target_velocity", -MAX_HAND_SPEED);
		else
			_handjoint.Set("motor/target_velocity", MAX_HAND_SPEED);

		if (angle < 0)
		{
			LeftWheelSpeed = -speed;
			RightWheelSpeed = speed;
		}
		if (angle > 0)
		{
			LeftWheelSpeed = speed;
			RightWheelSpeed = -speed;
		}
		_currentHandPosition = where;

		GD.Print($"Turning by {angle} while rotating hand to {where}");
		_robotState = ROBOT_STATE.TURNING_AND_ROTATING;
	}
	private void LoadCommandSequence()
	{
		LoadBlockPositions();

		_commandQueue.EnqueueCommands
		(
			//() => TurnAndRotate(Math.PI / 2, 1),
			//() => MoveBackward(1),
			//() => MoveForward(1),
			//() => TurnAndRotate(-Math.PI / 2, 0),
			//() => MoveBackward(1),
			//() => MoveForward(1.5),
			//() => Turn(-Math.PI / 2),
			//() => MoveForward(4),
			//() => MoveBackward(5)

			() => MoveForward(0.7),
			() => Turn(-Math.PI / 2),
			() => MoveUntillLine(1, true),
			() => Turn(Math.PI / 2),
			() => MoveBackward(1),
			() => MoveForward(1),
			() => Turn(Math.PI / 2),
			() => RotateHand(-1),
			() => MoveForward(0.7)
		);

		_commandQueue.StartExecution();
	}

	public bool IsExecuting()
	{
		return _commandQueue.IsExecuting();
	}
}

using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;
using Godot;

public partial class RobotSim : Node3D
{
	[Export] public Node3D RobotPivot { get; set; }
	[Export] public StaticBody3D HandL { get; set; }
	[Export] public StaticBody3D HandR { get; set; }
	[Export] public StaticBody3D Magazine { get; set; }

	// Parametry simulace (lze upravovat i v Inspektoru)
	[Export] public float BoardMoveScale = 0.001f; // Převod rychlosti motoru na m/s v simulaci
	[Export] public float ArmMoveSpeedDegPerSec = 180f;
	[Export] public float MagazineMoveSpeedPerSec = 0.08f;
	[Export] public float DefaultForwardVel = 500f;
	[Export] public float DefaultRotateAngle = 90f;
	[Export] public float DefaultObstacleDistance = 150f;

	// Stavy přesně podle main.py
	public int IsCW { get; private set; } = 1;
	public int IsCCW { get; private set; } = 0;
	public int IsLeft { get; private set; } = 0;
	public int IsRight { get; private set; } = 1;

	// Cílové hodnoty pro plynulý pohyb v 3D
	private float _targetHandYawDeg = -90f; // Odpovídá IsCW = 1
	private float _targetMagazineZ = 0.032f; // Odpovídá IsRight = 1

	// Aktuální pohybové chování robota (pro reálné řízení i přehrávání sekvence)
	private enum DriveMode { Idle, Forward, ReverseUnload, Rotate, UntilObstacle }
	private DriveMode _driveMode = DriveMode.Idle;
	private float _currentSpeed = 0f;
	private float _targetSpeed = 0f;
	private float _remainingDuration = 0f;
	private float _remainingAngleDeg = 0f;
	private float _rotateSign = 1f;
	private float _obstacleDistanceMm = 150f;
	private float _simulatedObstacleDistMm = 1000f; // Volitelně lze napojit na RayCast3D

	// Záznam kroků ("kód" uživatele / záznam z klávesnice)
	public sealed class SimStep
	{
		public string FunctionName { get; set; }
		public double Param1 { get; set; }
		public double Param2 { get; set; }

		public string ToPythonCall()
		{
			string p1 = Param1.ToString(CultureInfo.InvariantCulture);
			string p2 = Param2.ToString(CultureInfo.InvariantCulture);

			return FunctionName switch
			{
				"run_forward" => $"run_forward({p1})  # trvání: {p2} s",
				"rotate" => $"rotate({p1})",
				"run_until_obstacle" => $"run_until_obstacle({p1})",
				"arm_rotate_block_cw" => "arm_rotate_block_cw()",
				"arm_rotate_block_ccw" => "arm_rotate_block_ccw()",
				"arm_rotate_neutral" => "arm_rotate_neutral()",
				"mag_set_left" => "mag_set_left()",
				"mag_set_right" => "mag_set_right()",
				"unload_blocks" => $"unload_blocks()  # doba vykládky: {p1} s",
				"stop" => "stop()",
				_ => $"# {FunctionName}({p1}, {p2})"
			};
		}
	}

	private readonly List<SimStep> _recordedSteps = new();
	private readonly List<SimStep> _playbackQueue = new();
	private bool _isPlayingSequence = false;
	private float _stepWaitTimer = 0f;

	public override void _Ready()
	{
		// Inicializace výchozí polohy dle výchozího stavu IsCW=1, IsRight=1
		ApplyInstantPoseFromFlags();
		GD.Print("=== Simulace robota připravena ===");
		PrintControlsHelp();
	}

	public override void _UnhandledInput(InputEvent @event)
{
	if (_isPlayingSequence) return;

	if (@event is InputEventKey keyEvent && keyEvent.Pressed && !keyEvent.Echo)
	{
		switch (keyEvent.Keycode)
		{
			case Key.A:
				TriggerRotate(-DefaultRotateAngle);
				break;
			case Key.D:
				TriggerRotate(DefaultRotateAngle);
				break;
			case Key.J:
				TriggerArmRotateBlockCcw(); // Doleva (IsCCW = 1 -> +90°)
				break;
			case Key.L:
				TriggerArmRotateBlockCw();  // Doprava (IsCW = 1 -> -90°)
				break;
			case Key.K:
				TriggerArmRotateNeutral();  // Rovně (0°)
				break;
			case Key.Z:
				TriggerMagSetLeft();        // Magazín vlevo (-0.032)
				break;
			case Key.X:
				TriggerMagSetRight();       // Magazín vpravo (+0.032)
				break;
			case Key.U:
				TriggerUnloadBlocks(3.0);
				break;
			case Key.O:
				TriggerRunUntilObstacle(DefaultObstacleDistance);
				break;
			case Key.Space:
				TriggerStop();
				break;
			case Key.P:
				PrintRecordedCode();
				break;
			case Key.Enter:
				StartPlayback(_recordedSteps);
				break;
			case Key.C:
				_recordedSteps.Clear();
				GD.Print("[SIM] Záznam kroků vymazán.");
				break;
		}
	}
}

	// --- Funkce odpovídající main.py ---

	public void TriggerStop(bool record = true)
	{
		_driveMode = DriveMode.Idle;
		_targetSpeed = 0f;
		_currentSpeed = 0f;
		_remainingDuration = 0f;
		LogAndRecord("stop", 0, 0, record);
	}

	public void TriggerRotate(double angleDeg, bool record = true)
	{
		_driveMode = DriveMode.Rotate;
		_remainingAngleDeg = MathF.Abs((float)angleDeg);
		_rotateSign = angleDeg >= 0 ? -1f : 1f; // V Godotu kladné Y otáčí doleva (CCW)
		_currentSpeed = 0f;
		_targetSpeed = 0f;
		LogAndRecord("rotate", angleDeg, 0, record);
	}

	public void TriggerRunForward(double vel = 500, double durationSec = 3.0, bool record = true)
	{
		_driveMode = DriveMode.Forward;
		_targetSpeed = (float)vel;
		_remainingDuration = (float)durationSec;
		LogAndRecord("run_forward", vel, durationSec, record);
	}

	public void TriggerRunUntilObstacle(double distanceMm = 150, bool record = true)
	{
		_driveMode = DriveMode.UntilObstacle;
		_obstacleDistanceMm = (float)distanceMm;
		_targetSpeed = 150f;
		LogAndRecord("run_until_obstacle", distanceMm, 0, record);
	}

	public void TriggerArmRotateBlockCw(bool record = true)
	{
		// Dle main.py: ruce doprava -> IsCW = 1, IsCCW = 0 (-90° kolem Y)
		IsCW = 1;
		IsCCW = 0;
		_targetHandYawDeg = -90f;
		LogAndRecord("arm_rotate_block_cw", 0, 0, record);
	}

	public void TriggerArmRotateBlockCcw(bool record = true)
	{
		// Dle main.py: ruce doleva -> IsCCW = 1, IsCW = 0 (+90° kolem Y)
		IsCCW = 1;
		IsCW = 0;
		_targetHandYawDeg = 90f;
		LogAndRecord("arm_rotate_block_ccw", 0, 0, record);
	}

	public void TriggerArmRotateNeutral(bool record = true)
	{
		// Dle main.py: ruce rovně -> IsCW = 0, IsCCW = 0 (0° kolem Y)
		IsCW = 0;
		IsCCW = 0;
		_targetHandYawDeg = 0f;
		LogAndRecord("arm_rotate_neutral", 0, 0, record);
	}

	public void TriggerMagSetLeft(bool record = true)
	{
		// Dle main.py mag_set_left(): nastaví IsCW = 1, IsCCW = 0, IsLeft = 1, IsRight = 0
		IsCW = 1;
		IsCCW = 0;
		IsLeft = 1;
		IsRight = 0;

		_targetHandYawDeg = -90f;
		_targetMagazineZ = -0.032f;
		LogAndRecord("mag_set_left", 0, 0, record);
	}

	public void TriggerMagSetRight(bool record = true)
	{
		// Dle main.py mag_set_right(): nastaví IsCW = 0, IsCCW = 1, IsLeft = 0, IsRight = 1
		IsCW = 0;
		IsCCW = 1;
		IsLeft = 0;
		IsRight = 1;

		_targetHandYawDeg = 90f;
		_targetMagazineZ = 0.032f;
		LogAndRecord("mag_set_right", 0, 0, record);
	}

	public void TriggerUnloadBlocks(double unloadDurationSec = 3.0, bool record = true)
	{
		// Dle main.py unload_blocks(): nejprve ruce do neutrálu, poté couvání
		TriggerArmRotateNeutral(record: false);
		_driveMode = DriveMode.ReverseUnload;
		_targetSpeed = -400f;
		_remainingDuration = (float)unloadDurationSec;
		LogAndRecord("unload_blocks", unloadDurationSec, 0, record);
	}

	// --- Pohyb uzlů v simulaci ---

	private void UpdateRobotMovement(float dt)
	{
		if (RobotPivot == null) return;

		if (_driveMode == DriveMode.Rotate)
		{
			float stepDeg = ArmMoveSpeedDegPerSec * dt;
			if (stepDeg >= _remainingAngleDeg)
			{
				RobotPivot.RotateY(Mathf.DegToRad(_remainingAngleDeg * _rotateSign));
				_remainingAngleDeg = 0f;
				_driveMode = DriveMode.Idle;
			}
			else
			{
				RobotPivot.RotateY(Mathf.DegToRad(stepDeg * _rotateSign));
				_remainingAngleDeg -= stepDeg;
			}
			return;
		}

		if (_driveMode == DriveMode.UntilObstacle)
		{
			if (_simulatedObstacleDistMm <= _obstacleDistanceMm)
			{
				TriggerStop(record: false);
				return;
			}
			_currentSpeed = 150f;
		}
		else
		{
			// Plynulý náběh podobný run_forward / unload_blocks v main.py
			float accel = 600f * dt;
			if (_currentSpeed < _targetSpeed)
				_currentSpeed = MathF.Min(_targetSpeed, _currentSpeed + accel);
			else if (_currentSpeed > _targetSpeed)
				_currentSpeed = MathF.Max(_targetSpeed, _currentSpeed - accel);

			if (_remainingDuration > 0f)
			{
				_remainingDuration -= dt;
				if (_remainingDuration <= 0f && !_isPlayingSequence)
				{
					_targetSpeed = 0f;
				}
			}
		}

		// Posun robota po hrací desce (lokální osa -Z jako dopředný směr)
		float worldSpeed = _currentSpeed * BoardMoveScale;
		Vector3 forwardDir = -RobotPivot.GlobalTransform.Basis.Z.Normalized();
		RobotPivot.GlobalPosition += forwardDir * worldSpeed * dt;
	}

	private void UpdateHandsAndMagazine(float dt)
	{
		// Obě ruce se pohybují stejně kolem osy Y (-90° až +90°)
		float targetRad = Mathf.DegToRad(_targetHandYawDeg);
		float maxStepRad = Mathf.DegToRad(ArmMoveSpeedDegPerSec) * dt;

		bool handsReachedTarget = true;

		if (HandL != null)
		{
			float nextY = Mathf.MoveToward(HandL.Rotation.Y, targetRad, maxStepRad);
			HandL.Rotation = new Vector3(HandL.Rotation.X, nextY, HandL.Rotation.Z);
			if (MathF.Abs(nextY - targetRad) > 0.001f) handsReachedTarget = false;
		}

		if (HandR != null)
		{
			float nextY = Mathf.MoveToward(HandR.Rotation.Y, targetRad, maxStepRad);
			HandR.Rotation = new Vector3(HandR.Rotation.X, nextY, HandR.Rotation.Z);
			if (MathF.Abs(nextY - targetRad) > 0.001f) handsReachedTarget = false;
		}

		// Zásobník (magazine) se začne pohybovat až po dojetí rukou na místo (stejný motor)
		if (Magazine != null && handsReachedTarget)
		{
			float clampedTargetZ = Mathf.Clamp(_targetMagazineZ, -0.032f, 0.032f);
			float nextZ = Mathf.MoveToward(Magazine.Position.Z, clampedTargetZ, MagazineMoveSpeedPerSec * dt);
			Magazine.Position = new Vector3(Magazine.Position.X, Magazine.Position.Y, nextZ);
		}
	}

	private void ApplyInstantPoseFromFlags()
	{
		if (IsCW == 1) _targetHandYawDeg = -90f;
		else if (IsCCW == 1) _targetHandYawDeg = 90f;
		else _targetHandYawDeg = 0f;

		if (IsLeft == 1) _targetMagazineZ = -0.032f;
		else if (IsRight == 1) _targetMagazineZ = 0.032f;

		if (HandL != null) HandL.Rotation = new Vector3(HandL.Rotation.X, Mathf.DegToRad(_targetHandYawDeg), HandL.Rotation.Z);
		if (HandR != null) HandR.Rotation = new Vector3(HandR.Rotation.X, Mathf.DegToRad(_targetHandYawDeg), HandR.Rotation.Z);
		if (Magazine != null) Magazine.Position = new Vector3(Magazine.Position.X, Magazine.Position.Y, _targetMagazineZ);
	}

	// --- Záznam, úprava a přehrávání sekvence ---

	private void LogAndRecord(string funcName, double p1, double p2, bool record)
	{
		var step = new SimStep { FunctionName = funcName, Param1 = p1, Param2 = p2 };
		GD.Print($"[MAIN.PY FUNKCE] {step.ToPythonCall()}  | Stavy: IsCW={IsCW}, IsCCW={IsCCW}, IsLeft={IsLeft}, IsRight={IsRight}");

		if (record)
		{
			_recordedSteps.Add(step);
		}
	}

	public void PrintRecordedCode()
	{
		var sb = new StringBuilder();
		sb.AppendLine("=== Vygenerovaný seznam kroků (lze upravit hodnoty a přehrát) ===");
		for (int i = 0; i < _recordedSteps.Count; i++)
		{
			sb.AppendLine($"{i + 1}. {_recordedSteps[i].ToPythonCall()}");
		}
		GD.Print(sb.ToString());
	}

	/// <summary>
	/// Umožňuje vložit / upravit vlastní seznam kroků přímo v kódu a spustit simulaci.
	/// </summary>
	public void RunCustomSequence(IEnumerable<SimStep> customSteps)
	{
		StartPlayback(customSteps);
	}

	private void StartPlayback(IEnumerable<SimStep> steps)
	{
		_playbackQueue.Clear();
		_playbackQueue.AddRange(steps);
		_isPlayingSequence = _playbackQueue.Count > 0;
		_stepWaitTimer = 0f;
		GD.Print($"[SIM] Spouštím přehrávání sekvence ({_playbackQueue.Count} kroků)...");
	}

	private void UpdateSequencePlayback(float dt)
	{
		if (_stepWaitTimer > 0f)
		{
			_stepWaitTimer -= dt;
			return;
		}

		if (_playbackQueue.Count == 0)
		{
			_isPlayingSequence = false;
			TriggerStop(record: false);
			GD.Print("[SIM] Přehrávání sekvence dokončeno.");
			return;
		}

		SimStep step = _playbackQueue[0];
		_playbackQueue.RemoveAt(0);

		switch (step.FunctionName)
		{
			case "run_forward":
				TriggerRunForward(step.Param1, step.Param2 > 0 ? step.Param2 : 3.0, record: false);
				_stepWaitTimer = step.Param2 > 0 ? (float)step.Param2 : 3.0f;
				break;

			case "rotate":
				TriggerRotate(step.Param1, record: false);
				_stepWaitTimer = 1.0f;
				break;

			case "run_until_obstacle":
				TriggerRunUntilObstacle(step.Param1, record: false);
				_stepWaitTimer = 2.0f;
				break;

			case "arm_rotate_block_cw":
				TriggerArmRotateBlockCw(record: false);
				_stepWaitTimer = 0.6f;
				break;

			case "arm_rotate_block_ccw":
				TriggerArmRotateBlockCcw(record: false);
				_stepWaitTimer = 0.6f;
				break;

			case "arm_rotate_neutral":
				TriggerArmRotateNeutral(record: false);
				_stepWaitTimer = 0.6f;
				break;

			case "mag_set_left":
				TriggerMagSetLeft(record: false);
				_stepWaitTimer = 0.8f;
				break;

			case "mag_set_right":
				TriggerMagSetRight(record: false);
				_stepWaitTimer = 0.8f;
				break;

			case "unload_blocks":
				TriggerUnloadBlocks(step.Param1 > 0 ? step.Param1 : 3.0, record: false);
				_stepWaitTimer = step.Param1 > 0 ? (float)step.Param1 : 3.0f;
				break;

			case "stop":
			default:
				TriggerStop(record: false);
				_stepWaitTimer = 0.2f;
				break;
		}
	}

	private void PrintControlsHelp()
	{
		GD.Print(
			"Ovládání: W/S = vpřed/vzad, Q/E = rotate(-90/90), J/L/K = ruce (CCW/CW/neutrál), " +
			"Z/X = magazín (vlevo/vpravo), U = unload_blocks, O = run_until_obstacle, Space = stop | " +
            "P = vypsat kroky, Enter = přehrát kroky, C = smazat záznam"
		);
	}
}

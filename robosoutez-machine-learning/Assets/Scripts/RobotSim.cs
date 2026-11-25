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
	[Export] public float BoardMoveScale = 0.001f;
	[Export] public float TurnSpeedDegPerSec = 180f;
	[Export] public float ArmMoveSpeedDegPerSec = 180f;
	[Export] public float MagazineMoveSpeedPerSec = 0.08f;
	[Export] public float DefaultForwardVel = 500f;
	[Export] public float DefaultObstacleDistance = 150f;

	/// <summary>Kolizní vrstva (bitová maska) mantinelů hrací desky. Robot přes ni neprojede.</summary>
	[Export(PropertyHint.Layers3DPhysics)] public uint BoardWallMask = 8;
	/// <summary>Kolik iterací půlení intervalu se použije, když pohyb narazí do mantinelu.</summary>
	[Export] public int WallSearchIterations = 8;
	/// <summary>Kolizní vrstva domino kostek.</summary>
	[Export(PropertyHint.Layers3DPhysics)] public uint DominoMask = 2;
	/// <summary>Kostka blíž k mantinelu (nebo k jiné zaklíněné kostce) než tato vzdálenost [m] je považována za zaklíněnou.</summary>
	[Export] public float PinnedMargin = 0.004f;
	/// <summary>Kolizní vrstva dílů robota (tělo, ruce, zásobník).</summary>
	[Export(PropertyHint.Layers3DPhysics)] public uint RobotMask = 4;
	/// <summary>Kostku, která se dostala do robota, vždy vytlačit ve směru pohybu robota (fyzika by ji jinak mohla protlačit skrz tenké díly).</summary>
	[Export] public bool ForcePushDominoes = true;

	// Stavy dle main.py
	public int IsCW { get; private set; } = 1;
	public int IsCCW { get; private set; } = 0;
	public int IsLeft { get; private set; } = 0;
	public int IsRight { get; private set; } = 1;

	private float _targetHandYawDeg = -90f;
	private float _targetMagazineZ = 0.032f;

	private enum DriveMode { Idle, Forward, ReverseUnload, Rotate, UntilObstacle }
	private DriveMode _driveMode = DriveMode.Idle;

	private float _currentSpeed = 0f;
	private float _targetSpeed = 0f;
	private float _remainingDuration = 0f;
	private float _remainingAngleDeg = 0f;
	private float _rotateSign = 1f;
	private float _obstacleDistanceMm = 150f;
	private float _simulatedObstacleDistMm = 1000f;

	// Sledování plynulého otáčení při držení A / D
	private float _heldRotateDeg = 0f;
	private float _heldRotateSign = 0f;

	// Kolizní tvary robota (tělo, ruce, zásobník) pro kontrolu průniku s mantinely
	private sealed class RobotShape
	{
		public CollisionShape3D Node;
		public Shape3D QueryShape;
	}
	private readonly List<RobotShape> _robotShapes = new();
	private readonly PhysicsShapeQueryParameters3D _wallQuery = new();
	private bool _warnedStartOverlap = false;

	// Domino kostky – robot nesmí kostku opřenou o mantinel "rozmáčknout" (fyzika by ji vystřelila ven)
	private sealed class DominoInfo
	{
		public RigidBody3D Body;
		public CollisionShape3D Node;
		public BoxShape3D InflatedShape;
		public BoxShape3D PushShape;
	}
	private readonly List<DominoInfo> _dominoes = new();
	private readonly HashSet<Rid> _pinnedDominoes = new();
	private readonly Godot.Collections.Array<Rid> _nonPinnedDominoRids = new();
	private readonly PhysicsShapeQueryParameters3D _squeezeQuery = new();
	private const float SqueezeTolerance = 0.00001f;

	// Polohy dílů robota na začátku fyzikálního kroku – z nich se počítá, kam se díl posunul
	private readonly Dictionary<CollisionObject3D, Transform3D> _robotBodyPrevTransforms = new();
	private readonly List<CollisionObject3D> _robotBodies = new();
	private readonly PhysicsShapeQueryParameters3D _pushQuery = new();

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
		ApplyInstantPoseFromFlags();
		CollectRobotShapes(RobotPivot ?? this);
		_wallQuery.CollisionMask = BoardWallMask;
		_wallQuery.CollideWithBodies = true;
		_wallQuery.CollideWithAreas = false;
		_wallQuery.Margin = 0f;
		CollectDominoes(GetTree().Root);
		_squeezeQuery.CollisionMask = DominoMask;
		_squeezeQuery.Margin = 0f;
		GD.Print("Ovládání: W/S = vpřed/vzad (okamžité zastavení), A/D = plynulé otáčení podle doby držení, J/L/K = ruce (CCW/CW/neutrál), Z/X = magazín (vlevo/vpravo), U = unload_blocks, O = run_until_obstacle, Space = stop | P = vypsat, Enter = přehrát, C = vymazat");
	}

	public override void _UnhandledInput(InputEvent @event)
	{
		if (_isPlayingSequence) return;

		if (@event is InputEventKey keyEvent && keyEvent.Pressed && !keyEvent.Echo)
		{
			switch (keyEvent.Keycode)
			{
				case Key.J: TriggerArmRotateBlockCcw(); break;
				case Key.L: TriggerArmRotateBlockCw(); break;
				case Key.K: TriggerArmRotateNeutral(); break;
				case Key.Z: TriggerMagSetLeft(); break;
				case Key.X: TriggerMagSetRight(); break;
				case Key.U: TriggerUnloadBlocks(3.0); break;
				case Key.O: TriggerRunUntilObstacle(DefaultObstacleDistance); break;
				case Key.Space: TriggerStop(); break;
				case Key.P: PrintRecordedCode(); break;
				case Key.Enter: StartPlayback(_recordedSteps); break;
				case Key.C:
					_recordedSteps.Clear();
					GD.Print("[SIM] Záznam kroků vymazán.");
					break;
			}
		}
	}

	public override void _PhysicsProcess(double delta)
	{
		float dt = (float)delta;
		UpdatePinnedDominoes();
		foreach (CollisionObject3D b in _robotBodies) _robotBodyPrevTransforms[b] = b.GlobalTransform;

		if (!_isPlayingSequence)
		{
			HandleManualHoldMovement(dt);
		}
		else
		{
			UpdateSequencePlayback(dt);
		}

		UpdateStaticBodyRobotMovement(dt);
		UpdateHandsAndMagazine(dt);
		ForcePushDominoesOutOfRobot(dt);
	}

	private void HandleManualHoldMovement(float dt)
	{
		// 1. Jízda vpřed / vzad – okamžitý rozjezd i okamžité zastavení po puštění klávesy
		bool wPressed = Input.IsKeyPressed(Key.W);
		bool sPressed = Input.IsKeyPressed(Key.S);

		if (wPressed && !sPressed)
		{
			_driveMode = DriveMode.Forward;
			_targetSpeed = 250F;
			_currentSpeed = 250f;
			_remainingDuration = 0f;
		}
		else if (sPressed && !wPressed)
		{
			_driveMode = DriveMode.ReverseUnload;
			_targetSpeed = -250f;
			_currentSpeed = -250f;
			_remainingDuration = 0f;
		}
		else if (_driveMode == DriveMode.Forward || _driveMode == DriveMode.ReverseUnload)
		{
			_targetSpeed = 0f;
			_currentSpeed = 0f;
			_remainingDuration = 0f;
			_driveMode = DriveMode.Idle;
		}

		// 2. Otáčení po dobu držení A (doleva) / D (doprava)
		bool aPressed = Input.IsKeyPressed(Key.A);
		bool dPressed = Input.IsKeyPressed(Key.D);

		if (aPressed ^ dPressed)
		{
			float sign = dPressed ? -1f : 1f; // V Godotu kladné Y otáčí doleva (A), záporné doprava (D)
			float stepDeg = TurnSpeedDegPerSec * dt;

			stepDeg *= TryRotatePivot(stepDeg * sign);

			if (_heldRotateSign != sign && MathF.Abs(_heldRotateDeg) > 0.01f)
			{
				FlushHeldRotationRecord();
			}

			_heldRotateSign = sign;
			// Do rotate(angle) v main.py ukládáme kladný úhel pro otočení doprava (D) a záporný pro doleva (A)
			_heldRotateDeg += stepDeg * (dPressed ? 1f : -1f);
			_driveMode = DriveMode.Idle;
		}
		else if (MathF.Abs(_heldRotateDeg) > 0.01f)
		{
			FlushHeldRotationRecord();
		}
	}

	private void FlushHeldRotationRecord()
	{
		double roundedAngle = Math.Round(_heldRotateDeg, 1);
		if (Math.Abs(roundedAngle) >= 0.1)
		{
			LogAndRecord("rotate", roundedAngle, 0, record: true);
		}
		_heldRotateDeg = 0f;
		_heldRotateSign = 0f;
	}

	private void UpdateStaticBodyRobotMovement(float dt)
	{
		Node3D targetNode = RobotPivot ?? this;

		if (_driveMode == DriveMode.Rotate)
		{
			float stepDeg = TurnSpeedDegPerSec * dt;
			float appliedDeg = MathF.Min(stepDeg, _remainingAngleDeg);
			float done = TryRotatePivot(appliedDeg * _rotateSign);
			_remainingAngleDeg -= appliedDeg;

			// Zablokováno mantinelem – otáčení ukončíme
			if (done < 1f || _remainingAngleDeg <= 0.0001f)
			{
				_remainingAngleDeg = 0f;
				_driveMode = DriveMode.Idle;
			}
			return;
		}

		if (_driveMode == DriveMode.UntilObstacle)
		{
			_simulatedObstacleDistMm = MeasureObstacleDistanceMm(targetNode);
			if (_simulatedObstacleDistMm <= _obstacleDistanceMm)
			{
				TriggerStop(record: false);
				return;
			}
			_currentSpeed = 150f;
		}
		else if (_isPlayingSequence)
		{
			_currentSpeed = _targetSpeed;

			if (_remainingDuration > 0f)
			{
				_remainingDuration -= dt;
				if (_remainingDuration <= 0f)
				{
					_remainingDuration = 0f;
					_targetSpeed = 0f;
					_currentSpeed = 0f;
					_driveMode = DriveMode.Idle;
				}
			}
		}

		if (MathF.Abs(_currentSpeed) <= 0.0001f) return;

		Vector3 forwardDir = targetNode.GlobalTransform.Basis.X.Normalized();
		Vector3 displacement = forwardDir * (_currentSpeed * BoardMoveScale) * dt;

		Vector3 start = targetNode.GlobalPosition;
		float travelled = MoveBlockedByWalls(t => targetNode.GlobalPosition = start + displacement * t);

		// Robot narazil do mantinelu – zastavíme jízdu (ruční W/S se znovu rozjede při držení klávesy)
		if (travelled < 1f && _driveMode != DriveMode.Forward && _driveMode != DriveMode.ReverseUnload)
		{
			TriggerStop(record: false);
		}
	}

	// --- Kolize robota s mantinely hrací desky ---

	private void CollectRobotShapes(Node root)
	{
		_robotShapes.Clear();
		_robotBodies.Clear();
		var stack = new Stack<Node>();
		stack.Push(root);
		while (stack.Count > 0)
		{
			Node n = stack.Pop();
			foreach (Node child in n.GetChildren()) stack.Push(child);

			if (n is not CollisionShape3D cs || cs.Shape == null || cs.Disabled) continue;
			if (cs.GetParent() is not CollisionObject3D) continue;

			// Dotazy do fyziky nepodporují škálované tvary → škálu "zapečeme" do kopie tvaru
			Vector3 scale = cs.GlobalTransform.Basis.Scale.Abs();
			Shape3D query = cs.Shape;
			if (cs.Shape is BoxShape3D box)
			{
				query = new BoxShape3D { Size = box.Size * scale };
			}
			else if (!scale.IsEqualApprox(Vector3.One))
			{
				GD.PushWarning($"[SIM] Tvar {cs.GetPath()} je škálovaný a není BoxShape3D – kontrola mantinelů může být nepřesná.");
			}
			_robotShapes.Add(new RobotShape { Node = cs, QueryShape = query });
			var owner = (CollisionObject3D)cs.GetParent();
			if (!_robotBodies.Contains(owner)) _robotBodies.Add(owner);
		}
		GD.Print($"[SIM] Kontrola mantinelů: {_robotShapes.Count} kolizních tvarů robota.");
	}

	private bool RobotOverlapsWalls() => _robotShapes.Exists(ShapeOverlapsWalls);

	private bool ShapeOverlapsWalls(RobotShape rs)
	{
		if (!rs.Node.IsInsideTree() || rs.Node.Disabled) return false;
		Transform3D gt = rs.Node.GlobalTransform;
		_wallQuery.Shape = rs.QueryShape;
		_wallQuery.Transform = new Transform3D(gt.Basis.Orthonormalized(), gt.Origin);
		return GetWorld3D().DirectSpaceState.IntersectShape(_wallQuery, 1).Count > 0;
	}

	/// <summary>
	/// Provede pohyb (apply(t), t∈[0,1]) tak, aby robot neprošel mantinelem.
	/// Vrací, jaký podíl pohybu se skutečně provedl.
	/// </summary>
	private float MoveBlockedByWalls(Action<float> apply)
	{
		if (_robotShapes.Count == 0) { apply(1f); return 1f; }

		apply(0f);
		// Pokud robot už v mantinelu stojí (např. špatná startovní pozice), pustíme ho ven
		bool startInWall = RobotOverlapsWalls();
		if (startInWall && !_warnedStartOverlap)
		{
			GD.PushWarning("[SIM] Robot je v kolizi s mantinelem – pohyb nebude blokován, dokud se nedostane ven.");
			_warnedStartOverlap = true;
		}
		float startSqueeze = SqueezeDepth();

		bool Allowed() =>
			(startInWall || !RobotOverlapsWalls()) &&
			SqueezeDepth() <= startSqueeze + SqueezeTolerance;

		apply(1f);
		if (Allowed()) return 1f;

		// Půlení intervalu – najdeme nejdelší možný posun bez průniku
		float lo = 0f, hi = 1f;
		for (int i = 0; i < WallSearchIterations; i++)
		{
			float mid = 0.5f * (lo + hi);
			apply(mid);
			if (Allowed()) lo = mid; else hi = mid;
		}
		apply(lo);
		return lo;
	}

	private void CollectDominoes(Node root)
	{
		_dominoes.Clear();
		var stack = new Stack<Node>();
		stack.Push(root);
		while (stack.Count > 0)
		{
			Node n = stack.Pop();
			foreach (Node child in n.GetChildren()) stack.Push(child);
			if (n is not RigidBody3D body || (body.CollisionLayer & DominoMask) == 0) continue;

			foreach (Node child in body.GetChildren())
			{
				if (child is CollisionShape3D cs && cs.Shape is BoxShape3D box)
				{
					Vector3 size = box.Size * cs.GlobalTransform.Basis.Scale.Abs();
					_dominoes.Add(new DominoInfo
					{
						Body = body,
						Node = cs,
						InflatedShape = new BoxShape3D { Size = size + Vector3.One * (2f * PinnedMargin) },
						// Mírně nižší, aby dotaz nezachytil podlahu
						PushShape = new BoxShape3D { Size = new Vector3(size.X, MathF.Max(0.001f, size.Y - 0.004f), size.Z) }
					});
				}
			}
		}
		GD.Print($"[SIM] Nalezeno {_dominoes.Count} domino kostek.");
	}

	/// <summary>Najde kostky opřené o mantinel – přímo, nebo přes řetěz dalších kostek.</summary>
	private void UpdatePinnedDominoes()
	{
		_pinnedDominoes.Clear();
		_nonPinnedDominoRids.Clear();
		if (_dominoes.Count == 0) return;

		var space = GetWorld3D().DirectSpaceState;
		var q = new PhysicsShapeQueryParameters3D { Margin = 0f, CollisionMask = BoardWallMask };

		foreach (DominoInfo d in _dominoes)
		{
			if (!d.Node.IsInsideTree()) continue;
			Transform3D gt = d.Node.GlobalTransform;
			q.Shape = d.InflatedShape;
			q.Transform = new Transform3D(gt.Basis.Orthonormalized(), gt.Origin);
			if (space.IntersectShape(q, 1).Count > 0) _pinnedDominoes.Add(d.Body.GetRid());
		}

		// Řetězení: kostka dotýkající se zaklíněné kostky je také zaklíněná
		q.CollisionMask = DominoMask;
		bool changed = _pinnedDominoes.Count > 0;
		while (changed)
		{
			changed = false;
			foreach (DominoInfo d in _dominoes)
			{
				Rid rid = d.Body.GetRid();
				if (_pinnedDominoes.Contains(rid) || !d.Node.IsInsideTree()) continue;
				Transform3D gt = d.Node.GlobalTransform;
				q.Shape = d.InflatedShape;
				q.Transform = new Transform3D(gt.Basis.Orthonormalized(), gt.Origin);
				q.Exclude = new Godot.Collections.Array<Rid> { rid };
				foreach (var hit in space.IntersectShape(q, 8))
				{
					if (_pinnedDominoes.Contains((Rid)hit["rid"]))
					{
						_pinnedDominoes.Add(rid);
						changed = true;
						break;
					}
				}
			}
		}

		foreach (DominoInfo d in _dominoes)
		{
			Rid rid = d.Body.GetRid();
			if (!_pinnedDominoes.Contains(rid)) _nonPinnedDominoRids.Add(rid);
		}
		_squeezeQuery.Exclude = _nonPinnedDominoRids;
	}

	private bool DominoOverlapsMask(DominoInfo d, Vector3 position, uint mask)
	{
		Transform3D gt = d.Node.GlobalTransform;
		_pushQuery.Shape = d.PushShape;
		_pushQuery.Transform = new Transform3D(gt.Basis.Orthonormalized(), position + (gt.Origin - d.Body.GlobalPosition));
		_pushQuery.CollisionMask = mask;
		_pushQuery.Margin = 0f;
		return GetWorld3D().DirectSpaceState.IntersectShape(_pushQuery, 1).Count > 0;
	}

	/// <summary>
	/// Kostka, která po pohybu robota zůstala uvnitř robota, se posune ve směru pohybu dílu,
	/// který ji zasáhl, právě tak daleko, aby z robota vyjela. Do mantinelu se nikdy nevytlačí.
	/// </summary>
	private void ForcePushDominoesOutOfRobot(float dt)
	{
		if (!ForcePushDominoes || _dominoes.Count == 0 || dt <= 0f) return;
		var space = GetWorld3D().DirectSpaceState;

		foreach (DominoInfo d in _dominoes)
		{
			if (!d.Node.IsInsideTree()) continue;
			Vector3 pos = d.Body.GlobalPosition;
			Transform3D gt = d.Node.GlobalTransform;
			_pushQuery.Shape = d.PushShape;
			_pushQuery.Transform = new Transform3D(gt.Basis.Orthonormalized(), gt.Origin);
			_pushQuery.CollisionMask = RobotMask;
			_pushQuery.Margin = 0f;
			var hits = space.IntersectShape(_pushQuery, 4);
			if (hits.Count == 0) continue;

			// Pohyb dílu robota v místě kostky během tohoto kroku (největší ze zasažených dílů)
			Vector3 motion = Vector3.Zero;
			foreach (var hit in hits)
			{
				if (hit["collider"].AsGodotObject() is not CollisionObject3D body) continue;
				if (!_robotBodyPrevTransforms.TryGetValue(body, out Transform3D prev)) continue;
				Vector3 prevPoint = prev * (body.GlobalTransform.AffineInverse() * gt.Origin);
				Vector3 m = gt.Origin - prevPoint;
				m.Y = 0f;
				if (m.LengthSquared() > motion.LengthSquared()) motion = m;
			}
			if (motion.LengthSquared() < 1e-10f) continue; // robot stojí – kolizi vyřeší fyzika

			Vector3 dir = motion.Normalized();
			// Dostatečná vzdálenost, aby kostka vyjela i z hlubokého průniku
			float maxDist = motion.Length() + d.PushShape.Size.Length();
			if (DominoOverlapsMask(d, pos + dir * maxDist, RobotMask)) continue;

			float lo = 0f, hi = maxDist;
			for (int i = 0; i < 12; i++)
			{
				float mid = 0.5f * (lo + hi);
				if (DominoOverlapsMask(d, pos + dir * mid, RobotMask)) lo = mid; else hi = mid;
			}
			Vector3 target = pos + dir * (hi + 0.0005f);

			// Nikdy ne do mantinelu – zaklínění řeší blokace pohybu robota
			if (DominoOverlapsMask(d, target, BoardWallMask)) continue;

			d.Body.GlobalPosition = target;
			Vector3 v = d.Body.LinearVelocity;
			float along = v.Dot(dir);
			float robotSpeed = motion.Length() / dt;
			if (along < robotSpeed) d.Body.LinearVelocity = v + dir * (robotSpeed - along);
		}
	}

	/// <summary>Součet průniků robota do zaklíněných kostek [m].</summary>
	private float SqueezeDepth()
	{
		if (_pinnedDominoes.Count == 0) return 0f;
		var space = GetWorld3D().DirectSpaceState;
		float total = 0f;
		foreach (RobotShape rs in _robotShapes)
		{
			if (!rs.Node.IsInsideTree() || rs.Node.Disabled) continue;
			Transform3D gt = rs.Node.GlobalTransform;
			_squeezeQuery.Shape = rs.QueryShape;
			_squeezeQuery.Transform = new Transform3D(gt.Basis.Orthonormalized(), gt.Origin);
			var pts = space.CollideShape(_squeezeQuery, 8);
			float maxDepth = 0f;
			for (int i = 0; i + 1 < pts.Count; i += 2)
				maxDepth = MathF.Max(maxDepth, (pts[i] - pts[i + 1]).Length());
			total += maxDepth;
		}
		return total;
	}

	private float TryRotatePivot(float deg)
	{
		Node3D targetNode = RobotPivot ?? this;
		Basis startBasis = targetNode.GlobalBasis;
		return MoveBlockedByWalls(t =>
			targetNode.GlobalBasis = new Basis(Vector3.Up, Mathf.DegToRad(deg * t)) * startBasis);
	}

	private float MeasureObstacleDistanceMm(Node3D targetNode)
	{
		Vector3 forward = targetNode.GlobalTransform.Basis.X.Normalized();
		Vector3 from = targetNode.GlobalPosition;
		var ray = PhysicsRayQueryParameters3D.Create(from, from + forward * 3f, BoardWallMask);
		var hit = GetWorld3D().DirectSpaceState.IntersectRay(ray);
		if (hit.Count == 0) return 1000f;
		// Vzdálenost od přední hrany robota by závisela na senzoru; měříme od středu otáčení
		return ((Vector3)hit["position"] - from).Length() / BoardMoveScale;
	}

	// --- Funkce odpovídající main.py ---

	public void TriggerStop(bool record = true)
	{
		if (MathF.Abs(_heldRotateDeg) > 0.01f)
		{
			FlushHeldRotationRecord();
		}

		_driveMode = DriveMode.Idle;
		_targetSpeed = 0f;
		_currentSpeed = 0f;
		_remainingDuration = 0f;
		_remainingAngleDeg = 0f;
		LogAndRecord("stop", 0, 0, record);
	}

	public void TriggerRotate(double angleDeg, bool record = true)
	{
		_driveMode = DriveMode.Rotate;
		_remainingAngleDeg = MathF.Abs((float)angleDeg);
		_rotateSign = angleDeg >= 0 ? -1f : 1f;
		_currentSpeed = 0f;
		_targetSpeed = 0f;
		LogAndRecord("rotate", angleDeg, 0, record);
	}

	public void TriggerRunForward(double vel = 500, double durationSec = 3.0, bool record = true)
	{
		_driveMode = DriveMode.Forward;
		_targetSpeed = (float)vel;
		_currentSpeed = (float)vel;
		_remainingDuration = (float)durationSec;
		LogAndRecord("run_forward", vel, durationSec, record);
	}

	public void TriggerRunUntilObstacle(double distanceMm = 150, bool record = true)
	{
		_driveMode = DriveMode.UntilObstacle;
		_obstacleDistanceMm = (float)distanceMm;
		_targetSpeed = 150f;
		_currentSpeed = 150f;
		LogAndRecord("run_until_obstacle", distanceMm, 0, record);
	}

	public void TriggerArmRotateBlockCw(bool record = true)
	{
		IsCW = 1; IsCCW = 0;
		_targetHandYawDeg = -90f;
		LogAndRecord("arm_rotate_block_cw", 0, 0, record);
	}

	public void TriggerArmRotateBlockCcw(bool record = true)
	{
		IsCCW = 1; IsCW = 0;
		_targetHandYawDeg = 90f;
		LogAndRecord("arm_rotate_block_ccw", 0, 0, record);
	}

	public void TriggerArmRotateNeutral(bool record = true)
	{
		IsCW = 0; IsCCW = 0;
		_targetHandYawDeg = 0f;
		LogAndRecord("arm_rotate_neutral", 0, 0, record);
	}

	public void TriggerMagSetLeft(bool record = true)
	{
		IsCW = 1; IsCCW = 0; IsLeft = 1; IsRight = 0;
		_targetHandYawDeg = -90f;
		_targetMagazineZ = -0.032f;
		LogAndRecord("mag_set_left", 0, 0, record);
	}

	public void TriggerMagSetRight(bool record = true)
	{
		IsCW = 0; IsCCW = 1; IsLeft = 0; IsRight = 1;
		_targetHandYawDeg = 90f;
		_targetMagazineZ = 0.032f;
		LogAndRecord("mag_set_right", 0, 0, record);
	}

	public void TriggerUnloadBlocks(double unloadDurationSec = 3.0, bool record = true)
	{
		TriggerArmRotateNeutral(record: false);
		_driveMode = DriveMode.ReverseUnload;
		_targetSpeed = -400f;
		_currentSpeed = -400f;
		_remainingDuration = (float)unloadDurationSec;
		LogAndRecord("unload_blocks", unloadDurationSec, 0, record);
	}

	private void UpdateHandsAndMagazine(float dt)
	{
		float targetRad = Mathf.DegToRad(_targetHandYawDeg);
		float maxStepRad = Mathf.DegToRad(ArmMoveSpeedDegPerSec) * dt;
		bool handsReachedTarget = true;

		foreach (StaticBody3D hand in new[] { HandL, HandR })
		{
			if (hand == null) continue;
			float startY = hand.Rotation.Y;
			float nextY = Mathf.MoveToward(startY, targetRad, maxStepRad);
			MoveBlockedByWalls(t => hand.Rotation = new Vector3(hand.Rotation.X, Mathf.Lerp(startY, nextY, t), hand.Rotation.Z));
			if (MathF.Abs(hand.Rotation.Y - targetRad) > 0.001f) handsReachedTarget = false;
		}

		if (Magazine != null && handsReachedTarget)
		{
			float clampedTargetZ = Mathf.Clamp(_targetMagazineZ, -0.032f, 0.032f);
			float startZ = Magazine.Position.Z;
			float nextZ = Mathf.MoveToward(startZ, clampedTargetZ, MagazineMoveSpeedPerSec * dt);
			MoveBlockedByWalls(t => Magazine.Position = new Vector3(Magazine.Position.X, Magazine.Position.Y, Mathf.Lerp(startZ, nextZ, t)));
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

	private void LogAndRecord(string funcName, double p1, double p2, bool record)
	{
		var step = new SimStep { FunctionName = funcName, Param1 = p1, Param2 = p2 };
		GD.Print($"[MAIN.PY FUNKCE] {step.ToPythonCall()}  | Stavy: IsCW={IsCW}, IsCCW={IsCCW}, IsLeft={IsLeft}, IsRight={IsRight}");
		if (record) _recordedSteps.Add(step);
	}

	public void PrintRecordedCode()
	{
		var sb = new StringBuilder();
		sb.AppendLine("=== Vygenerovaný seznam kroků ===");
		for (int i = 0; i < _recordedSteps.Count; i++)
			sb.AppendLine($"{i + 1}. {_recordedSteps[i].ToPythonCall()}");
		GD.Print(sb.ToString());
	}

	public void RunCustomSequence(IEnumerable<SimStep> customSteps) => StartPlayback(customSteps);

	private void StartPlayback(IEnumerable<SimStep> steps)
	{
		if (MathF.Abs(_heldRotateDeg) > 0.01f)
		{
			FlushHeldRotationRecord();
		}

		_playbackQueue.Clear();
		_playbackQueue.AddRange(steps);
		_isPlayingSequence = _playbackQueue.Count > 0;
		_stepWaitTimer = 0f;
		GD.Print($"[SIM] Spouštím přehrávání sekvence ({_playbackQueue.Count} kroků)...");
	}

	private void UpdateSequencePlayback(float dt)
	{
		if (_stepWaitTimer > 0f) { _stepWaitTimer -= dt; return; }
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
				_stepWaitTimer = MathF.Abs((float)step.Param1) / MathF.Max(1f, TurnSpeedDegPerSec);
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
			default:
				TriggerStop(record: false);
				_stepWaitTimer = 0.2f;
				break;
		}
	}
}

using Godot;
using RoboSoutezMachineLearning.Global;
using System;
using System.Collections.Generic;
using System.Text;
using System.Text.Json;

public partial class McpServer : Node
{
    private const int Port = 9000;

    private TcpServer _tcpServer;
    private readonly Dictionary<long, WebSocketPeer> _peers = new();
    private long _lastPeerId = 0;

    private GameField _gameField;
    private IRobot _robot;

    public override void _Ready()
    {
        // Adjust the path to your Player node as needed
        _robot = GetParent().GetNode<Robot>("Node/Robot");
        _gameField = GetParent().GetNode<GameField>("GameField");

        _tcpServer = new TcpServer();
        var err = _tcpServer.Listen(Port);
        if (err == Error.Ok)
        {
            GD.Print($"TCP server started on port {Port}.");
        }
        else
        {
            GD.PushError($"Unable to start TCP server on port {Port}: {err}");
            SetProcess(false);
        }
    }

    public override void _Process(double delta)
    {
        //var gs = _gameField.GenerateTextImage();


        // Accept new connections and wrap them in WebSocketPeer
        while (_tcpServer.IsConnectionAvailable())
        {
            _lastPeerId += 1;

            var stream = _tcpServer.TakeConnection();
            GD.Print($"+ Peer {_lastPeerId} connected (TCP).");

            var ws = new WebSocketPeer();
            var wsErr = ws.AcceptStream(stream);
            if (wsErr != Error.Ok)
            {
                GD.PushError($"Failed to accept WebSocket stream: {wsErr}");
                stream.DisconnectFromHost();
                continue;
            }

            _peers[_lastPeerId] = ws;
            GD.Print($"+ Peer {_lastPeerId} upgraded to WebSocket.");
        }

        // Iterate over connected peers
        var peerIds = new List<long>(_peers.Keys);
        foreach (var peerId in peerIds)
        {
            var peer = _peers[peerId];
            // Let the WebSocketPeer process its internal state
            peer.Poll();

            var state = peer.GetReadyState();

            if (state == WebSocketPeer.State.Open)
            {
                // Read all pending packets
                while (peer.GetAvailablePacketCount() > 0)
                {
                    byte[] packet = peer.GetPacket();

                    if (peer.WasStringPacket())
                    {
                        string text = Encoding.UTF8.GetString(packet);
                        GD.Print($"< Got text data from peer {peerId}: {text}");

                        HandleRawMessage(peer, text);
                    }
                    else
                    {
                        GD.Print($"< Got binary data from peer {peerId}: {packet.Length} bytes (ignored)");
                        // If you want, you could handle binary here.
                    }
                }
            }
            else if (state == WebSocketPeer.State.Closed)
            {
                _peers.Remove(peerId);
                int code = peer.GetCloseCode();
                string reason = peer.GetCloseReason();
                GD.Print($"- Peer {peerId} closed with code: {code}, reason: {reason}. Clean: {code != -1}");
            }
        }
    }

    private void HandleRawMessage(WebSocketPeer peer, string text)
    {
        try
        {
            var options = new JsonSerializerOptions
            {
                PropertyNameCaseInsensitive = true
            };

            var cmd = JsonSerializer.Deserialize<GameCommand>(text, options);
            if (cmd != null)
            {
                var response = HandleCommand(peer, cmd);
                SendJson(peer, response);
            }
            else
            {
                SendJson(peer, new { status = "ERR", error = "Empty or invalid command" });
            }
        }
        catch (Exception e)
        {
            GD.PushError($"Failed to parse command: {e.Message}");
            SendJson(peer, new { status = "ERR", error = "Invalid JSON", detail = e.Message });
        }
    }
    private int ConvertWhere(string where) 
    {
        if (where == "left")
            return -1;
        if (where == "straight")
            return 0;
        if (where == "right")
            return 1;

        return 0;
    }
    private int ConvertColor(string color)
    {
        if (color == "black")
            return 1;
        if (color == "yellow")
            return 2;
        return 0;
    }

    private CommandResult HandleCommand(WebSocketPeer peer, GameCommand cmd)
    {
        switch (cmd.type)
        {
            case "moveforward":
                _robot.MoveForward(cmd.seconds);
                break;
            case "movebackward":
                _robot.MoveBackward(cmd.seconds);
                break;
            case "rotatehand":
                _robot.RotateHand(ConvertWhere(cmd.where));
                break;
            case "turn":
                _robot.Turn(cmd.radians);
                break;
            case "moveuntillline":
                _robot.MoveUntillLine(cmd.count, cmd.forward);
                break;
            case "turnandrotate":
                _robot.TurnAndRotate(cmd.radians, ConvertWhere(cmd.where));
                break;
            case "gamestatus":
                var gs = _gameField.GenerateTextImage();
                return new CommandResult() { status = "OK", screen = gs, gameover = _robot.IsAlive()};
            default:
                break;
        }

        while (_robot.IsExecuting())
            System.Threading.Thread.Sleep(100);

        return new CommandResult() { status = "OK" };
    }

    private void SendJson(WebSocketPeer peer, object obj)
    {
        string json = JsonSerializer.Serialize(obj);
        // For WebSocket text frames we can just send the string:
        peer.SendText(json);
    }
}

public class GameCommand
{
    public string type { get; set; }
    public string where { get; set; }
    public float seconds { get; set; }
    public float radians { get; set; }
    public int count { get; set; }
    public string color { get; set; }
    public bool forward { get; set; }
}

public class CommandResult
{ 
    public string status {  get; set; }
    public string screen {  get; set; }
    public bool gameover {  get; set; }
}
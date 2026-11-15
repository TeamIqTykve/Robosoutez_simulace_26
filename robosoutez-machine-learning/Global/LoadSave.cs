using Godot;
using System.Collections.Generic;

namespace RoboSoutezMachineLearning.Global
{
    internal partial class LoadSave : Node
    {

        public List<NodePath> NodePaths = new();

        /// <summary>
        /// Saves transforms of all nodes in NodePaths to JSON file.
        /// </summary>
        public void SaveTransforms(string filePath)
        {
            var data = new Dictionary<string, object>();

            foreach (var path in NodePaths)
            {
                if (GetTree().Root.GetNodeOrNull<Node3D>(path) is Node3D node)
                {
                    data[node.Name] = new Dictionary<string, object>
                {
                    { "position", new float[] { node.GlobalPosition.X, node.GlobalPosition.Y, node.GlobalPosition.Z } },
                    { "rotation", new float[] { node.GlobalRotation.X, node.GlobalRotation.Y, node.GlobalRotation.Z } }
                };
                }
            }

            string json = System.Text.Json.JsonSerializer.Serialize(data);
            //string json = Json.Stringify(data, indent: "  ");
            using var file = FileAccess.Open(filePath, FileAccess.ModeFlags.Write);
            file.StoreString(json);
            GD.Print($"Transforms saved to: {filePath}");
        }

        /// <summary>
        /// Loads transforms from JSON file and applies them to nodes.
        /// </summary>
        public void LoadTransforms(string filePath)
        {
            if (!FileAccess.FileExists(filePath))
            {
                GD.PrintErr($"File not found: {filePath}");
                return;
            }

            using var file = FileAccess.Open(filePath, FileAccess.ModeFlags.Read);
            string json = file.GetAsText();

            var dict = System.Text.Json.JsonSerializer.Deserialize<Dictionary<string, object>>(json);
            //var result = Json.ParseString(json);

            foreach (var path in NodePaths)
            {
                if (GetTree().Root.GetNodeOrNull<Node3D>(path) is Node3D node && dict.ContainsKey(node.Name))
                {
                    var pr =  System.Text.Json.JsonSerializer.Deserialize<PR>(dict[node.Name].ToString());


                    node.GlobalPosition = new Vector3(
                        (float)pr.position[0], (float)pr.position[1], (float)pr.position[2]
                    );

                    node.GlobalRotation = new Vector3(
                        (float)pr.rotation[0], (float)pr.rotation[1], (float)pr.rotation[2]
                    );
                }
            }

            GD.Print($"Transforms loaded from: {filePath}");
        }
    }

    internal class PR
    {
        public double[] position { get; set; }
        public double[] rotation { get; set; }
    }
}


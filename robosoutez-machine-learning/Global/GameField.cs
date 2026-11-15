using Godot;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RoboSoutezMachineLearning.Global
{
    internal partial class GameField : Node3D
    {
        [Export] public Camera3D camera;
        [Export] public int textWidth = 5;
        [Export] public int textHeight = 5;
        [Export] public float maxRayDistance = 100f;

        // Map object types/names to characters
        private Dictionary<string, char> objectCharMap = new Dictionary<string, char>()
    {
        { "BlackLine", 'B' },
        { "YellowLine", 'Y' },
        { "SBlock", 'S' },
        { "SBlock2", 'S' },
        { "SBlock3", 'S' },
        { "SBlock4", 'S' },
        { "LineBlock", 'L' },
        { "LineBlock2", 'L' },
        { "LineBlock3", 'L' },
        { "LineBlock4", 'L' },
        { "ZBlock", 'Z' },
        { "ZBlock2", 'Z' },
        { "ZBlock3", 'Z' },
        { "ZBlock4", 'Z' },
        { "LBlock", 'J' },
        { "LBlock2", 'J' },
        { "LBlock3", 'J' },
        { "LBlock4", 'J' },
        { "TBlock", 'T' },
        { "TBlock2", 'T' },
        { "TBlock3", 'T' },
        { "TBlock4", 'T' },
        { "Robot", 'R' },
        { "RobotHand", 'H' },
        { "Ground", '_' },
        { "Wall", 'W' }
    };

        public override void _Ready()
        {
            if (camera == null)
            {
                camera = GetViewport().GetCamera3D();
            }
        }

        /// <summary>
        /// Generates a text-based representation of what the camera sees
        /// </summary>
        public string GenerateTextImage()
        {
            if (camera == null)
            {
                GD.PrintErr("Camera not assigned!");
                return "";
            }

            StringBuilder result = new StringBuilder();
            PhysicsDirectSpaceState3D spaceState = GetWorld3D().DirectSpaceState;

            var screenSize = camera.GetWindow().Size;

            for (int y = 25 * textHeight; y < (screenSize.Y - 29 * textHeight); y += textHeight)
            {
                for (int x = 65 * textWidth; x < (screenSize.X - 64 * textWidth); x += textWidth)
                {
                    float screenX = x;
                    float screenY = y;

                    // Get ray from camera through this screen position
                    Vector3 rayOrigin = camera.ProjectRayOrigin(new Vector2(screenX, screenY));
                    Vector3 rayDirection = camera.ProjectRayNormal(new Vector2(screenX, screenY));
                    Vector3 rayEnd = rayOrigin + rayDirection * maxRayDistance;

                    // Perform raycast
                    var query = PhysicsRayQueryParameters3D.Create(rayOrigin, rayEnd);
                    query.CollideWithAreas = false;
                    query.CollideWithBodies = true;

                    var result3D = spaceState.IntersectRay(query);

                    if (result3D.Count > 0)
                    {
                        // Hit something
                        Node3D hitObject = result3D["collider"].As<Node3D>();
                        char character = GetCharacterForObject(hitObject);
                        result.Append(character);
                    }
                    else
                    {
                        // No collision
                        result.Append('_');
                    }
                }
                result.Append('\n');
            }

            return result.ToString();
        }

        /// <summary>
        /// Maps a hit object to a character representation
        /// </summary>
        private char GetCharacterForObject(Node3D obj)
        {
            if (obj == null) return '?';

            // Try exact name match first
            if (objectCharMap.ContainsKey(obj.Name))
            {
                return objectCharMap[obj.Name];
            }

            // Try matching by groups/tags
            foreach (var group in obj.GetGroups())
            {
                string groupName = group.ToString();
                if (objectCharMap.ContainsKey(groupName))
                {
                    return objectCharMap[groupName];
                }
            }

            // Try matching by class name
            string className = obj.GetClass();
            if (objectCharMap.ContainsKey(className))
            {
                return objectCharMap[className];
            }

            // Try matching by partial name
            foreach (var kvp in objectCharMap)
            {
                if (obj.Name.ToString().Contains(kvp.Key, StringComparison.OrdinalIgnoreCase))
                {
                    return kvp.Value;
                }
            }

            // Default: use first letter of object name
            string name = obj.Name.ToString();
            return name.Length > 0 ? char.ToUpper(name[0]) : '?';
        }

        /// <summary>
        /// Alternative version that also returns depth information
        /// </summary>
        public string GenerateTextImageWithDepth()
        {
            if (camera == null)
            {
                GD.PrintErr("Camera not assigned!");
                return "";
            }

            StringBuilder result = new StringBuilder();
            PhysicsDirectSpaceState3D spaceState = GetWorld3D().DirectSpaceState;

            // Depth characters from closest to farthest
            string depthChars = "@%#*+=-:. ";

            for (int y = 0; y < textHeight; y++)
            {
                for (int x = 0; x < textWidth; x++)
                {
                    float screenX = (float)x / textWidth;
                    float screenY = (float)y / textHeight;

                    Vector3 rayOrigin = camera.ProjectRayOrigin(new Vector2(screenX, screenY));
                    Vector3 rayDirection = camera.ProjectRayNormal(new Vector2(screenX, screenY));
                    Vector3 rayEnd = rayOrigin + rayDirection * maxRayDistance;

                    var query = PhysicsRayQueryParameters3D.Create(rayOrigin, rayEnd);
                    query.CollideWithAreas = false;
                    query.CollideWithBodies = true;

                    var result3D = spaceState.IntersectRay(query);

                    if (result3D.Count > 0)
                    {
                        Vector3 hitPosition = result3D["position"].AsVector3();
                        float distance = rayOrigin.DistanceTo(hitPosition);

                        // Map distance to depth character
                        int depthIndex = Mathf.Clamp(
                            (int)((distance / maxRayDistance) * depthChars.Length),
                            0,
                            depthChars.Length - 1
                        );

                        result.Append(depthChars[depthIndex]);
                    }
                    else
                    {
                        result.Append('_');
                    }
                }
                result.Append('\n');
            }

            return result.ToString();
        }

        /// <summary>
        /// Generates a colored text representation (returns object type per pixel)
        /// Useful for creating a legend or more detailed analysis
        /// </summary>
        public Dictionary<Vector2I, string> GenerateObjectMap()
        {
            var objectMap = new Dictionary<Vector2I, string>();

            if (camera == null) return objectMap;

            PhysicsDirectSpaceState3D spaceState = GetWorld3D().DirectSpaceState;

            var screenSize = camera.GetWindow().Size;

            for (int y = 3; y < (screenSize.Y - 10); y+= textHeight)
            {
                for (int x = 38; x < (screenSize.X - 34); x+= textWidth)
                {
                    float screenX = x;
                    float screenY = y;

                    Vector3 rayOrigin = camera.ProjectRayOrigin(new Vector2(screenX, screenY));
                    Vector3 rayDirection = camera.ProjectRayNormal(new Vector2(screenX, screenY));
                    Vector3 rayEnd = rayOrigin + rayDirection * maxRayDistance;

                    var query = PhysicsRayQueryParameters3D.Create(rayOrigin, rayEnd);
                    var result3D = spaceState.IntersectRay(query);

                    if (result3D.Count > 0)
                    {
                        Node3D hitObject = result3D["collider"].As<Node3D>();
                        objectMap[new Vector2I(x, y)] = hitObject.Name;
                    }
                    else
                    {
                        objectMap[new Vector2I(x, y)] = "empty";
                    }
                }
            }

            return objectMap;
        }
    }
}

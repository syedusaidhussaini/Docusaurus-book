# Chapter 5: Unity for Human-Robot Interaction

## Learning Objectives

After completing this chapter, you will be able to:
- Set up Unity for robot visualization and HRI
- Create 3D models for humanoid robots
- Implement ROS 2 to Unity communication
- Design intuitive human-robot interaction interfaces

## 5.1 Introduction to Unity for Robotics

Unity is a powerful 3D development platform that provides real-time rendering capabilities, making it ideal for human-robot interaction (HRI) interfaces and visualization. In robotics, Unity serves as a high-fidelity visualization layer that complements physics simulation in Gazebo.

### Key Features for Robotics:
- Real-time 3D rendering
- Physics simulation (for visualization)
- Cross-platform deployment
- Extensive asset ecosystem
- Scripting with C#

## 5.2 Setting Up Unity for Robotics

First, create a new Unity project and install the ROS# package for ROS 2 communication:

### Unity Project Structure:
```
simulation/unity/
├── Assets/
│   ├── Scenes/
│   │   ├── RobotVisualization.unity
│   │   └── HRI_Interface.unity
│   ├── Scripts/
│   │   ├── ROSCommunication/
│   │   ├── RobotVisualization/
│   │   └── HRI/
│   ├── Models/
│   │   ├── RobotParts/
│   │   └── Environments/
│   ├── Materials/
│   ├── Prefabs/
│   └── Plugins/
├── Packages/
└── ProjectSettings/
```

## 5.3 Creating Robot Models in Unity

Create a basic humanoid robot hierarchy in Unity:

```csharp
// RobotController.cs
using UnityEngine;
using System.Collections.Generic;

public class RobotController : MonoBehaviour
{
    [Header("Robot Configuration")]
    public List<Transform> jointTransforms = new List<Transform>();
    public List<string> jointNames = new List<string>();

    [Header("Joint Limits")]
    public float minJointAngle = -90f;
    public float maxJointAngle = 90f;

    [Header("Visualization")]
    public bool showJointAxes = true;

    // Store initial joint positions
    private List<Quaternion> initialRotations = new List<Quaternion>();

    void Start()
    {
        // Store initial rotations for reference
        foreach(Transform joint in jointTransforms)
        {
            initialRotations.Add(joint.localRotation);
        }
    }

    // Update joint positions from ROS data
    public void UpdateJointPositions(List<float> jointPositions)
    {
        if (jointPositions.Count != jointTransforms.Count)
        {
            Debug.LogWarning("Joint position count mismatch!");
            return;
        }

        for (int i = 0; i < jointTransforms.Count; i++)
        {
            // Apply joint position as rotation (simplified)
            float angle = Mathf.Clamp(jointPositions[i], minJointAngle, maxJointAngle);
            jointTransforms[i].localRotation = initialRotations[i] *
                Quaternion.Euler(0, 0, angle);
        }
    }

    void OnDrawGizmos()
    {
        if (showJointAxes && jointTransforms != null)
        {
            foreach (Transform joint in jointTransforms)
            {
                Gizmos.color = Color.red;
                Gizmos.DrawWireSphere(joint.position, 0.02f);
            }
        }
    }
}
```

## 5.4 ROS 2 to Unity Communication

Implement ROS communication in Unity using the ROS# library:

```csharp
// ROSCommunicationManager.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Std;
using System.Collections.Generic;

public class ROSCommunicationManager : MonoBehaviour
{
    [Header("ROS Configuration")]
    public string rosIP = "127.0.0.1";
    public int rosPort = 10000;

    [Header("Robot Topics")]
    public string jointStateTopic = "/joint_states";
    public string imuTopic = "/imu/data";

    private ROSConnection ros;
    private RobotController robotController;

    void Start()
    {
        // Connect to ROS
        ros = ROSConnection.GetOrCreateInstance();
        ros.Initialize(rosIP, rosPort);

        // Subscribe to topics
        ros.Subscribe<Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor.JointStateMsg>(
            jointStateTopic, OnJointStateReceived);

        ros.Subscribe<Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor.ImuMsg>(
            imuTopic, OnImuReceived);

        robotController = FindObjectOfType<RobotController>();
    }

    void OnJointStateReceived(Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor.JointStateMsg jointState)
    {
        if (robotController != null)
        {
            // Convert ROS joint state to Unity format
            List<float> jointPositions = new List<float>();
            foreach (double pos in jointState.position)
            {
                jointPositions.Add((float)pos);
            }

            robotController.UpdateJointPositions(jointPositions);
        }
    }

    void OnImuReceived(Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor.ImuMsg imuMsg)
    {
        // Process IMU data for visualization
        ProcessImuData(imuMsg);
    }

    void ProcessImuData(Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor.ImuMsg imuMsg)
    {
        // Extract orientation and display it
        Quaternion orientation = new Quaternion(
            (float)imuMsg.orientation.x,
            (float)imuMsg.orientation.y,
            (float)imuMsg.orientation.z,
            (float)imuMsg.orientation.w
        );

        // Apply to robot body or display in UI
        // Implementation depends on your specific needs
    }

    // Publish commands to ROS
    public void SendJointCommand(string jointName, float position)
    {
        var jointCmd = new Unity.Robotics.ROSTCPConnector.MessageTypes.Std.Float64Msg();
        jointCmd.data = position;

        ros.Send(jointName + "_command", jointCmd);
    }
}
```

## 5.5 Creating HRI Interface Components

Design user interfaces for human-robot interaction:

```csharp
// HRIController.cs
using UnityEngine;
using UnityEngine.UI;
using System.Collections;
using System.Collections.Generic;

public class HRIController : MonoBehaviour
{
    [Header("UI References")]
    public Slider[] jointSliders;
    public Button[] presetButtons;
    public Text robotStatusText;
    public Text jointValuesText;

    [Header("Robot Presets")]
    public PresetPose[] robotPresets;

    private ROSCommunicationManager rosComm;
    private RobotController robotCtrl;

    [System.Serializable]
    public class PresetPose
    {
        public string name;
        public float[] jointPositions;
    }

    void Start()
    {
        rosComm = FindObjectOfType<ROSCommunicationManager>();
        robotCtrl = FindObjectOfType<RobotController>();

        // Initialize UI
        InitializeUI();
    }

    void InitializeUI()
    {
        // Setup joint sliders
        for (int i = 0; i < jointSliders.Length; i++)
        {
            int index = i; // Closure variable
            jointSliders[i].onValueChanged.AddListener((value) =>
                OnJointSliderChanged(index, value));
        }

        // Setup preset buttons
        for (int i = 0; i < presetButtons.Length; i++)
        {
            int index = i;
            presetButtons[i].onClick.AddListener(() => ApplyPresetPose(index));
        }
    }

    void OnJointSliderChanged(int jointIndex, float value)
    {
        // Update joint visualization
        if (robotCtrl != null && robotCtrl.jointTransforms.Count > jointIndex)
        {
            // Send command to ROS
            if (rosComm != null)
            {
                rosComm.SendJointCommand(
                    robotCtrl.jointNames[jointIndex],
                    value
                );
            }

            // Update text display
            if (jointValuesText != null)
            {
                jointValuesText.text = $"Joint {jointIndex}: {value:F2}";
            }
        }
    }

    public void ApplyPresetPose(int presetIndex)
    {
        if (presetIndex < robotPresets.Length)
        {
            PresetPose preset = robotPresets[presetIndex];

            // Apply each joint position
            for (int i = 0; i < Mathf.Min(preset.jointPositions.Length, jointSliders.Length); i++)
            {
                jointSliders[i].value = preset.jointPositions[i];
            }

            // Update robot status
            if (robotStatusText != null)
            {
                robotStatusText.text = $"Applied preset: {preset.name}";
            }
        }
    }

    // Emergency stop functionality
    public void EmergencyStop()
    {
        // Send emergency stop command to ROS
        if (rosComm != null)
        {
            var stopMsg = new Unity.Robotics.ROSTCPConnector.MessageTypes.Std.BoolMsg();
            stopMsg.data = true;
            rosComm.ros.Send("/emergency_stop", stopMsg);
        }

        if (robotStatusText != null)
        {
            robotStatusText.text = "EMERGENCY STOP ACTIVATED";
        }
    }
}
```

## 5.6 Visualization Components

Create visualization components for robot state and sensor data:

```csharp
// RobotStateVisualizer.cs
using UnityEngine;
using UnityEngine.UI;
using System.Collections.Generic;

public class RobotStateVisualizer : MonoBehaviour
{
    [Header("Visualization Elements")]
    public GameObject robotModel;
    public Text statusText;
    public Image[] jointStatusIndicators;
    public Slider[] jointLoadSliders;
    public Text[] jointAngleTexts;

    [Header("Color Coding")]
    public Color normalColor = Color.green;
    public Color warningColor = Color.yellow;
    public Color errorColor = Color.red;

    // Robot state data
    private List<float> currentJointPositions = new List<float>();
    private List<float> currentJointVelocities = new List<float>();
    private List<float> currentJointEfforts = new List<float>();
    private bool[] jointStatus = new bool[12]; // Assuming 12 joints

    void Update()
    {
        UpdateVisualization();
    }

    public void UpdateRobotState(
        List<float> positions,
        List<float> velocities,
        List<float> efforts,
        bool[] status)
    {
        currentJointPositions = positions;
        currentJointVelocities = velocities;
        currentJointEfforts = efforts;
        jointStatus = status;
    }

    void UpdateVisualization()
    {
        // Update joint status indicators
        for (int i = 0; i < jointStatusIndicators.Length && i < jointStatus.Length; i++)
        {
            if (i < jointStatus.Length)
            {
                jointStatusIndicators[i].color = jointStatus[i] ? normalColor : errorColor;
            }
        }

        // Update joint load sliders
        for (int i = 0; i < jointLoadSliders.Length && i < currentJointEfforts.Count; i++)
        {
            if (i < currentJointEfforts.Count)
            {
                // Normalize effort value (assuming max effort of 100)
                float normalizedEffort = Mathf.Abs(currentJointEfforts[i]) / 100f;
                jointLoadSliders[i].value = normalizedEffort;
            }
        }

        // Update joint angle displays
        for (int i = 0; i < jointAngleTexts.Length && i < currentJointPositions.Count; i++)
        {
            if (i < currentJointPositions.Count)
            {
                jointAngleTexts[i].text = $"{currentJointPositions[i]:F2}°";
            }
        }

        // Update overall status
        UpdateRobotStatus();
    }

    void UpdateRobotStatus()
    {
        int activeJoints = 0;
        int totalJoints = jointStatus.Length;

        for (int i = 0; i < jointStatus.Length; i++)
        {
            if (jointStatus[i]) activeJoints++;
        }

        float statusPercentage = (float)activeJoints / totalJoints * 100;

        if (statusText != null)
        {
            statusText.text = $"Robot Status: {activeJoints}/{totalJoints} joints active ({statusPercentage:F1}%)";

            // Color code based on status
            if (statusPercentage >= 95f)
                statusText.color = normalColor;
            else if (statusPercentage >= 80f)
                statusText.color = warningColor;
            else
                statusText.color = errorColor;
        }
    }
}
```

## 5.7 Scene Setup and Configuration

Create a Unity scene that integrates all components:

1. **Main Camera**: Set up for robot visualization
2. **Robot Model**: Import and configure the robot hierarchy
3. **ROS Communication Manager**: Handle ROS connections
4. **Robot Controller**: Manage robot state and visualization
5. **HRI Controller**: Handle user interface interactions
6. **Robot State Visualizer**: Display robot status information

### Example Scene Hierarchy:
```
Main Camera
Directional Light
RobotModel
├── BaseLink
├── Torso
├── LeftArm
│   ├── Shoulder
│   ├── Elbow
│   └── Wrist
├── RightArm
├── LeftLeg
└── RightLeg
ROSCommunicationManager (empty GameObject)
HRIController (empty GameObject)
RobotStateVisualizer (empty GameObject)
Canvas (UI elements)
├── JointSlidersPanel
├── PresetButtonsPanel
├── RobotStatusPanel
└── EmergencyStopButton
```

## Exercises

1. Create a Unity scene with a humanoid robot model and basic joint control
2. Implement ROS communication to visualize real-time robot state
3. Design an intuitive HRI interface with joint sliders and preset positions
4. Add safety features like emergency stop and joint limit visualization

## Summary

This chapter covered Unity integration for human-robot interaction, including robot visualization, ROS communication, and intuitive interface design. You've learned how to create effective visualization and interaction systems for humanoid robots.
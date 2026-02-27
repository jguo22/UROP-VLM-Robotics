# CSV Trajectory Controller for Unity Robot Simulation

This guide explains how to use the CSV trajectory system to control robot arm motion using exported trajectory data.

## Overview

The CSV Trajectory Controller allows you to:
- Load robot trajectories from CSV files
- Play back recorded motion paths
- Control playback speed and looping
- Integrate with the existing Unified Robot Controller system

## Components

### 1. CSVTrajectoryController.cs
Main controller that handles CSV file loading, parsing, and playback.

### 2. UnifiedRobotController.cs (Modified)
Enhanced to include CSV trajectory mode and integration methods.

### 3. CSVTrajectoryExample.cs
Example script showing how to use the system with a GUI.

## CSV File Format

The system expects CSV files in the format exported by your robot simulation:

```csv
Timestamp,base_PosX,base_PosY,base_PosZ,base_RotX,base_RotY,base_RotZ,base_RotW,shoulder_PosX,shoulder_PosY,shoulder_PosZ,shoulder_RotX,shoulder_RotY,shoulder_RotZ,shoulder_RotW,elbow_PosX,elbow_PosY,elbow_PosZ,elbow_RotX,elbow_RotY,elbow_RotZ,elbow_RotW,wrist1_PosX,wrist1_PosY,wrist1_PosZ,wrist1_RotX,wrist1_RotY,wrist1_RotZ,wrist1_RotW,wrist2_PosX,wrist2_PosY,wrist2_PosZ,wrist2_RotX,wrist2_RotY,wrist2_RotZ,wrist2_RotW,wrist3_PosX,wrist3_PosY,wrist3_PosZ,wrist3_RotX,wrist3_RotY,wrist3_RotZ,wrist3_RotW
0.000,0.000,0.719,0.000,0.000,-1.000,0.000,-0.001,0.136,0.719,0.000,-0.001,-0.837,0.547,-0.001,0.017,0.890,-0.389,-0.001,-0.577,0.817,-0.001,0.018,0.759,-0.759,-0.001,-0.326,0.945,0.000,0.111,0.759,-0.759,-0.442,-0.289,0.836,0.152,0.111,0.685,-0.817,-0.454,-0.366,0.805,0.109,0.500,0.644,-0.500,-0.001,0.000,-0.001,1.000,-0.500,0.640,-0.500,0.000,0.000,0.000,1.000
```

### Required Columns:
- **Timestamp**: Time in seconds for each frame
- **Joint Position Data**: PosX, PosY, PosZ, RotX, RotY, RotZ, RotW for each joint
- **Supported Joints**: base, shoulder, elbow, wrist1, wrist2, wrist3

## 📁 File Organization

### Exports Folder (Auto-Generated)
```
📂 ur5_simulation/Exports/
└── 📄 RobotJointCoordinates_YYYYMMDD_HHMMSS.csv
```
Contains trajectories automatically recorded by the robot system.

### Imports Folder (Custom Trajectories)
```
📂 ur5_simulation/Imports/
├── 📄 Your_Custom_Trajectory.csv
├── 📄 SampleTrajectory_Test.csv (sample included)
└── 📄 README.md (detailed usage guide)
```
Place your custom trajectory files here for testing and playback.

### How Unity Finds Files:
- **Automatically scans both folders** when you press F1
- **Shows folder prefixes**: `[Exports]` and `[Imports]` in the dropdown
- **Loads from correct location** based on selection

## Setup Instructions

### 1. Add Components to Your Scene

1. Ensure your robot has the required components:
   - `UnifiedRobotController`
   - `CSVTrajectoryController`
   - `RobotArmSetup`
   - `Controller` (for manual control)

2. Add the `CSVTrajectoryExample` script to see the demo GUI.

### 2. Configure CSV Files

**Option A: Using TextAssets (Recommended for built applications)**
1. Place your CSV files in `Assets/Exports/`
2. Drag CSV files to the `csvFiles` list in the Inspector

**Option B: Using File Paths**
1. Set the `csvFilePath` in the `CSVTrajectoryController` component
2. Or use the custom file path field in the demo GUI

### 3. Control Modes

The system integrates with the Unified Robot Controller's control modes:

1. **Manual** (Press 1): Keyboard joint control
2. **IK** (Press 2): End-effector inverse kinematics
3. **Pick and Place** (Press 3): Automated pick and place
4. **Programmatic** (Press 4): Script-based control
5. **CSV Trajectory** (Press 5): CSV file playback **← NEW**

## Usage Examples

### Basic Playback

```csharp
using UnityEngine;

public class SimpleCSVExample : MonoBehaviour
{
    public UnifiedRobotController robotController;

    void Start()
    {
        // Load trajectory from file path
        robotController.LoadCSVTrajectory("C:/Path/To/Your/Trajectory.csv");

        // Start playback
        robotController.StartCSVPlayback();
    }

    void Update()
    {
        // Control playback with keyboard
        if (Input.GetKeyDown(KeyCode.Space))
        {
            robotController.PauseCSVPlayback();
        }

        if (Input.GetKeyDown(KeyCode.S))
        {
            robotController.StopCSVPlayback();
        }
    }
}
```

### Advanced Control

```csharp
using UnityEngine;

public class AdvancedCSVExample : MonoBehaviour
{
    public UnifiedRobotController robotController;
    public TextAsset[] trajectoryFiles;

    void Start()
    {
        // Load trajectory from TextAsset
        robotController.LoadCSVTrajectory(trajectoryFiles[0]);

        // Set playback speed to half speed
        robotController.SetCSVPlaybackSpeed(0.5f);

        // Start playback
        robotController.StartCSVPlayback();
    }

    void Update()
    {
        // Check playback status
        if (robotController.IsCSVPlaying())
        {
            float progress = robotController.GetCSVProgress();
            Debug.Log($"Playback progress: {progress:P1}");
        }

        // Speed control
        if (Input.GetKeyDown(KeyCode.Plus))
        {
            robotController.SetCSVPlaybackSpeed(
                robotController.GetComponent<CSVTrajectoryController>().playbackSpeed * 1.2f);
        }
    }
}
```

## Keyboard Controls

When in CSV Trajectory mode (press 5):

- **Space**: Play/Pause playback
- **S**: Stop playback
- **R**: Reload current trajectory
- **+/-**: Increase/decrease playback speed
- **L**: Toggle trajectory looping
- **Left/Right Arrow**: Step through frames (for debugging)

## GUI Controls

The demo GUI (press F1 to toggle) provides:

- **File Selection**: Choose from available CSV files
- **Custom Path**: Load from any file path
- **Playback Controls**: Play, pause, stop buttons
- **Speed Control**: Adjust playback speed
- **Progress Display**: Visual progress bar and frame counter
- **Loop Toggle**: Enable/disable trajectory looping

## API Reference

### CSVTrajectoryController Methods

```csharp
// Loading
bool LoadTrajectory()                    // Load from configured path/TextAsset
void LoadTrajectoryFromPath(string path) // Load from file path
void LoadTrajectoryFromTextAsset(TextAsset asset) // Load from TextAsset

// Playback Control
void StartPlayback()                     // Start trajectory playback
void PausePlayback()                     // Pause/resume playback
void StopPlayback()                      // Stop playback
void JumpToFrame(int frameIndex)         // Jump to specific frame
void JumpToTime(float time)              // Jump to specific time
void SetPlaybackSpeed(float speed)       // Set playback speed multiplier

// Status
bool IsPlaying()                         // Check if currently playing
float GetProgress()                      // Get playback progress (0-1)

// Properties
public bool loopTrajectory               // Enable/disable looping
public float playbackSpeed               // Playback speed multiplier
public int totalFrames                   // Total number of frames
public float totalDuration               // Total trajectory duration
```

### UnifiedRobotController CSV Methods

```csharp
// Loading
void LoadCSVTrajectory(string filePath)        // Load from file path
void LoadCSVTrajectory(TextAsset textAsset)    // Load from TextAsset

// Playback Control
void StartCSVPlayback()                        // Start CSV playback
void StopCSVPlayback()                         // Stop CSV playback
void PauseCSVPlayback()                        // Pause/resume CSV playback
void SetCSVPlaybackSpeed(float speed)          // Set playback speed

// Status
bool IsCSVPlaying()                           // Check if CSV is playing
float GetCSVProgress()                        // Get CSV progress (0-1)
```

## Troubleshooting

### Common Issues

1. **"CSV file must have at least header and one data row"**
   - Check that your CSV file has the correct format
   - Ensure the file is not empty

2. **"Timestamp column not found"**
   - Verify the CSV has a "Timestamp" column
   - Check column header spelling

3. **Robot not moving during playback**
   - Ensure you're in CSV Trajectory mode (press 5)
   - Check that the trajectory loaded successfully
   - Verify joint names match your robot configuration

4. **Playback speed too fast/slow**
   - Adjust playback speed using +/- keys or SetPlaybackSpeed()
   - Check timestamp intervals in your CSV file

### Debug Information

Enable debug logging to see detailed information:
- Trajectory loading status
- Frame-by-frame playback information
- Joint angle conversion details

## Performance Considerations

- **File Size**: Large CSV files may take time to load
- **Frame Rate**: Very high-frequency trajectories may impact performance
- **Memory Usage**: Large trajectories are stored in memory during playback

## Integration with Existing Systems

The CSV trajectory system integrates seamlessly with:
- **UnifiedRobotController**: Access via control mode 5
- **RobotArmSetup**: Uses existing joint configuration
- **PickAndPlaceController**: Can be combined with trajectory playback
- **FKRobot**: Uses existing forward kinematics for validation

## Future Enhancements

Potential improvements:
- Real-time trajectory recording
- Trajectory blending and interpolation
- Path optimization and smoothing
- Multi-robot coordination
- Trajectory validation and collision checking

---

For questions or issues, check the Unity console for detailed error messages and debug information.


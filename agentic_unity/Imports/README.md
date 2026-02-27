# 📁 Imports Folder - Custom Trajectory Files

This folder is for your **custom trajectory files** that you want to import and test with the UR5 robot.

## 🚀 How to Use

1. **Place your CSV files** in this `Imports/` folder
2. **Run Unity** and load your scene
3. **Press `5`** to enter CSV Trajectory mode
4. **Press `F1`** to show the trajectory control panel
5. **Select your file** from the dropdown (marked with [Imports])
6. **Click "Load Selected Trajectory"**
7. **Click "Play"** to see your robot follow the trajectory

## 📋 CSV File Format

Your CSV files must follow this exact format:

### Required Columns (Header Row):
```csv
Timestamp,base_PosX,base_PosY,base_PosZ,base_RotX,base_RotY,base_RotZ,base_RotW,shoulder_PosX,shoulder_PosY,shoulder_PosZ,shoulder_RotX,shoulder_RotY,shoulder_RotZ,shoulder_RotW,elbow_PosX,elbow_PosY,elbow_PosZ,elbow_RotX,elbow_RotY,elbow_RotZ,elbow_RotW,wrist1_PosX,wrist1_PosY,wrist1_PosZ,wrist1_RotX,wrist1_RotY,wrist1_RotZ,wrist1_RotW,wrist2_PosX,wrist2_PosY,wrist2_PosZ,wrist2_RotX,wrist2_RotY,wrist2_RotZ,wrist2_RotW,wrist3_PosX,wrist3_PosY,wrist3_PosZ,wrist3_RotX,wrist3_RotY,wrist3_RotZ,wrist3_RotW,block_PosX,block_PosY,block_PosZ,block_RotX,block_RotY,block_RotZ,block_RotW,platform_PosX,platform_PosY,platform_PosZ,platform_RotX,platform_RotY,platform_RotZ,platform_RotW
```

### Data Format:
- **Timestamp**: Time in seconds (e.g., 0.000, 0.100, 0.200, etc.)
- **Position**: X, Y, Z coordinates in meters
- **Rotation**: X, Y, Z, W quaternion components
- **Joints**: base, shoulder, elbow, wrist1, wrist2, wrist3
- **Objects**: block and platform (optional, can be left as default values)

### Sample Data Row:
```csv
0.000,0.000,0.719,0.000,0.000,0.000,0.000,1.000,0.136,0.719,0.000,0.000,0.000,0.000,1.000,0.425,0.719,0.000,0.000,0.000,0.000,1.000,0.717,0.719,0.000,0.000,0.000,0.000,1.000,0.717,0.719,-0.392,0.000,0.000,0.000,1.000,0.717,0.719,-0.784,0.000,0.000,0.000,1.000,0.500,0.100,0.000,0.000,0.000,0.000,1.000,0.000,0.000,0.500,0.000,0.000,0.000,1.000
```

## 🎯 Creating Your Own Trajectories

### Method 1: Record from Robot
1. **Run the robot** with recording enabled
2. **Move the robot** through your desired path
3. **The system automatically saves** to `Exports/` folder
4. **Copy the file** to `Imports/` for testing

### Method 2: Create Manually
1. **Use the sample file** `SampleTrajectory_Test.csv` as a template
2. **Modify the joint positions** for each timestamp
3. **Ensure smooth transitions** between frames
4. **Save as CSV** in this folder

### Method 3: Generate Programmatically
1. **Write a script** to generate trajectory points
2. **Calculate joint positions** for each point
3. **Export to CSV format** with proper headers
4. **Save in this folder**

## 📊 Sample Trajectory Included

**`SampleTrajectory_Test.csv`** - A basic test trajectory that:
- ✅ Moves the robot arm in a smooth arc
- ✅ Contains 16 frames (1.6 seconds total)
- ✅ Uses realistic joint movements
- ✅ Follows the exact CSV format required

## 🔧 Tips for Creating Good Trajectories

### Timing:
- **Frame Rate**: 10-30 frames per second works well
- **Total Duration**: 2-10 seconds for testing
- **Consistent Intervals**: Keep timestamp intervals regular

### Joint Movements:
- **Smooth Transitions**: Avoid sudden jumps between frames
- **Realistic Limits**: Don't exceed joint angle limits
- **Natural Motion**: Follow realistic robot movement patterns

### Testing:
- **Start Simple**: Begin with basic movements
- **Test Gradually**: Add complexity incrementally
- **Verify Playback**: Use Unity's trajectory controls to test

## 🚨 Important Notes

### File Format Requirements:
- ✅ **Must have exact column headers** (copy from sample)
- ✅ **First column must be "Timestamp"**
- ✅ **Data must be comma-separated**
- ✅ **No empty rows**
- ✅ **Numbers should have proper decimal places**

### Unity Integration:
- ✅ **Files are automatically detected** when you press F1
- ✅ **Shows as [Imports] filename** in dropdown
- ✅ **Loads from correct folder** automatically
- ✅ **Timestamp-based playback** respects your timing

### Troubleshooting:
- ❌ **File not showing?** Check CSV format and headers
- ❌ **Robot not moving?** Verify joint data is valid
- ❌ **Timing issues?** Check timestamp intervals are consistent

## 🎮 Quick Test

1. **Keep the sample file** `SampleTrajectory_Test.csv` in this folder
2. **Run Unity** and load your scene
3. **Press `5` then `F1`**
4. **Select `[Imports] SampleTrajectory_Test.csv`**
5. **Click "Load Selected Trajectory"**
6. **Click "Play"** and watch your robot move!

**Happy trajectory creation! 🤖✨**

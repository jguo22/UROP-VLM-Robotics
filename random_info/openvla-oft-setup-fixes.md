# OpenVLA-OFT Setup Fixes

A summary of all fixes required to set up OpenVLA-OFT on an HPC cluster.

## Environment
- Python 3.10.18
- PyTorch 2.2.0+cu121
- System CUDA: 13.1

---

## 1. Flash-Attention Installation

**Problem:** `[Errno 18] Invalid cross-device link` — pip couldn't build flash-attn due to cross-filesystem cache issues on the HPC cluster.

**Fix:** Install the pre-built wheel directly:
```bash
pip install https://github.com/Dao-AILab/flash-attention/releases/download/v2.5.5/flash_attn-2.5.5+cu122torch2.2cxx11abiFALSE-cp310-cp310-linux_x86_64.whl
```

---

## 2. CUDA Version Mismatch

**Problem:** System CUDA (13.1) didn't match PyTorch's CUDA (12.1), causing compilation errors.

**Fix:** Load a compatible CUDA module:
```bash
module unload cuda
module load cuda/12.4.0
```

**Verify:**
```bash
nvcc --version
python -c "import torch; print(torch.version.cuda)"
```

---

## 3. NumPy Version Conflict

**Problem:** `A module that was compiled using NumPy 1.x cannot be run in NumPy 2.x` — flash-attn and other packages require NumPy 1.x.

**Fix:** Downgrade NumPy:
```bash
pip install "numpy<2.0.0"
```

---

## 4. OpenCV Conflict

**Problem:** `opencv-python 4.13.0.90 requires numpy>=2` — conflicts with the NumPy 1.x requirement.

**Fix:** Downgrade OpenCV:
```bash
pip install "opencv-python<4.10"
```

---

## 5. OpenGL/MuJoCo Rendering

**Problem:** `AttributeError: 'NoneType' object has no attribute 'glGetError'` — PyOpenGL can't find a working OpenGL backend for headless rendering on the cluster.

**Fix (Option A - EGL for GPU rendering):**
```bash
export MUJOCO_GL=egl
export PYOPENGL_PLATFORM=egl
```

**Fix (Option B - OSMesa for software rendering):**
```bash
conda install -c conda-forge mesalib
export MUJOCO_GL=osmesa
export PYOPENGL_PLATFORM=osmesa
```

**Make it permanent** by adding to `~/.bashrc`:
```bash
echo 'export MUJOCO_GL=egl' >> ~/.bashrc
echo 'export PYOPENGL_PLATFORM=egl' >> ~/.bashrc
source ~/.bashrc
```

---

## Quick Setup Script

```bash
#!/bin/bash

# Load CUDA
module unload cuda
module load cuda/12.4.0

# Set rendering backend
export MUJOCO_GL=egl
export PYOPENGL_PLATFORM=egl

# Install dependencies with fixes
pip install https://github.com/Dao-AILab/flash-attention/releases/download/v2.5.5/flash_attn-2.5.5+cu122torch2.2cxx11abiFALSE-cp310-cp310-linux_x86_64.whl
pip install "numpy<2.0.0"
pip install "opencv-python<4.10"
```

---

## Useful Commands

```bash
# Check PyTorch CUDA version
python -c "import torch; print(torch.version.cuda)"

# Check if CUDA is available
python -c "import torch; print(torch.cuda.is_available())"

# Check NumPy version
python -c "import numpy; print(numpy.__version__)"

# Check for OpenGL libraries
ldconfig -p | grep -i osmesa
ldconfig -p | grep -i egl
```

# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

OpenVLA is a Vision-Language-Action model for robotic manipulation built on top of Prismatic VLMs. The codebase supports training VLAs from scratch, fine-tuning pretrained models (via LoRA or full fine-tuning), and deploying models for robot control.

## Development Commands

### Installation

```bash
# Create virtual environment with Python 3.11
python3.11 -m venv venv

# Activate virtual environment
source venv/bin/activate 

# Upgrade pip
pip install --upgrade pip

# Install PyTorch 
pip install torch==2.2.0 torchvision==0.17.0 torchaudio==2.2.0

# Install package in editable mode
pip install -e .
```

### Linting and Code Quality

```bash
# Check code style without modifying files
make check

# Auto-format code (also happens via pre-commit)
make autoformat

# Clean temporary files
make clean
```

### Training and Fine-Tuning

#### LoRA Fine-Tuning (Recommended for most users)
```bash
torchrun --standalone --nnodes 1 --nproc-per-node <NUM_GPUS> vla-scripts/finetune.py \
  --vla_path "openvla/openvla-7b" \
  --data_root_dir <DATASETS_DIR> \
  --dataset_name <DATASET_NAME> \
  --run_root_dir <LOGS_DIR> \
  --adapter_tmp_dir <ADAPTER_DIR> \
  --lora_rank 32 \
  --batch_size 16 \
  --learning_rate 5e-4
```

#### Full Fine-Tuning (Requires 8 GPUs)
```bash
torchrun --standalone --nnodes 1 --nproc-per-node 8 vla-scripts/train.py \
  --pretrained_checkpoint <CHECKPOINT_PATH> \
  --vla.type <VLA_CONFIG_ID> \
  --data_root_dir <DATASETS_DIR> \
  --run_root_dir <LOGS_DIR>
```

### Evaluation

#### LIBERO Simulation Evaluation
```bash
python experiments/robot/libero/run_libero_eval.py \
  --model_family openvla \
  --pretrained_checkpoint <CHECKPOINT> \
  --task_suite_name <SUITE> \
  --center_crop True
```

#### BridgeData V2 Robot Evaluation
```bash
python experiments/robot/bridge/run_bridgev2_eval.py \
  --model_family openvla \
  --pretrained_checkpoint openvla/openvla-7b
```

### Deployment

```bash
# Start OpenVLA REST API server
python vla-scripts/deploy.py --openvla_path <MODEL_PATH>
```

## Architecture

### High-Level Structure

OpenVLA extends Prismatic VLMs with action prediction capabilities:
- **Vision Backbone**: Processes robot camera images (DINOv2 + SigLIP fusion)
- **LLM Backbone**: Llama-2 based language model that outputs action tokens
- **Action Tokenizer**: Discretizes continuous actions into tokens for the LLM vocabulary

### Key Components

#### 1. Action Tokenization (`prismatic/vla/action_tokenizer.py`)
- Discretizes continuous robot actions (7-DoF) into 256 bins per dimension
- Maps discretized actions to least-used tokens in LLM vocabulary
- Enables language model to predict robot actions as token sequences

#### 2. VLA Model (`prismatic/models/vlas/openvla.py`)
- Lightweight wrapper around PrismaticVLM
- Core method: `predict_action(image, instruction)` → continuous action
- Handles tokenization/detokenization and action normalization/unnormalization

#### 3. Dataset Pipeline (`prismatic/vla/datasets/rlds/`)
- Loads Open-X Embodiment datasets in RLDS format
- **Mixtures** (`oxe/mixtures.py`): Defines dataset combinations with sampling weights
- **Transforms** (`oxe/transforms.py`): Dataset-specific standardization (observation/action spaces)
- **Configs** (`oxe/configs.py`): Dataset metadata (action dims, camera names)

#### 4. Training Configuration (`prismatic/conf/vla.py`)
- Defines VLA training configs as dataclasses with `draccus`
- Each config specifies: base VLM, data mixture, optimization hyperparameters
- Register new configs in `VLARegistry` for training

### Data Flow

**Training**: Image + Instruction → Vision Encoder → LLM → Action Tokens → Loss
**Inference**: Image + Instruction → Vision Encoder → LLM.generate() → Action Tokens → Continuous Actions

### Directory Structure

- `prismatic/`: Core library code
  - `models/`: VLM and VLA model definitions
  - `vla/`: VLA-specific code (action tokenizer, datasets, RLDS loaders)
  - `conf/`: Training configurations
  - `training/`: FSDP training strategies and metrics
- `vla-scripts/`: Main entry points for training, fine-tuning, deployment
- `experiments/robot/`: Robot evaluation scripts (LIBERO, BridgeData V2)
- `scripts/`: Legacy scripts for base VLM training

## Important Implementation Details

### Adding New Datasets

To fine-tune on a custom dataset:

1. Convert dataset to RLDS format (use https://github.com/kpertsch/rlds_dataset_builder)
2. Add dataset config in `prismatic/vla/datasets/rlds/oxe/configs.py`:
   - Specify observation space (camera names, state dims)
   - Specify action space (dimensionality)
3. Add transform function in `prismatic/vla/datasets/rlds/oxe/transforms.py`:
   - Standardize observations to `{"image_*": ..., "EEF_state": ..., "gripper_state": ...}`
   - Standardize actions to 7-DoF format (6-DoF end-effector delta + 1 gripper)
   - Register in `OXE_STANDARDIZATION_TRANSFORMS`
4. Add mixture in `prismatic/vla/datasets/rlds/oxe/mixtures.py`
5. For LoRA: pass `--dataset_name` to `vla-scripts/finetune.py`
6. For full fine-tuning: create new config in `prismatic/conf/vla.py` referencing your mixture

### Action Space

OpenVLA uses 7-DoF actions:
- 6 dimensions: End-effector pose delta (3 position + 3 rotation)
- 1 dimension: Gripper state (binary open/close, normalized to [-1, 1])

Actions are normalized to [-1, 1] during training using per-dataset statistics.

### Prompt Format

OpenVLA uses simple prompt templates:
- `openvla-7b`: `"In: What action should the robot take to {instruction}?\nOut:"`
- `openvla-7b-v01`: Uses Vicuna-style chat template

### Converting Prismatic Checkpoints to HuggingFace

After full fine-tuning, convert to HF format:
```bash
cd <PRISMATIC_RUN_DIR>/checkpoints
ln -s <checkpoint-file> latest-checkpoint.pt
python vla-scripts/extern/convert_openvla_weights_to_hf.py \
  --openvla_model_path_or_id <PRISMATIC_RUN_DIR> \
  --output_hf_model_local_path <OUTPUT_DIR>
```

## Package Versions

Critical dependencies (pinned for reproducibility):
- PyTorch: 2.2.0
- transformers: 4.40.1
- tokenizers: 0.19.1
- timm: 0.9.10
- flash-attn: 2.5.5
- peft: 0.11.1
- tensorflow-datasets: 4.9.3 (downgrade if RLDS loading fails)

## Dataset Naming Conventions

- `bridge_orig`: Original BridgeData V2 from project website (preferred)
- `bridge_oxe`: BridgeData V2 from Open-X GCP bucket (outdated)
- Always rename downloaded `bridge_dataset/` to `bridge_orig/` to match codebase expectations

## Training Strategies

- Default: FSDP (Fully Sharded Data Parallel) for multi-GPU training
- Supports gradient checkpointing and mixed precision (BF16)
- LoRA fine-tuning uses HuggingFace PEFT with DDP
- Batch size = `per_device_batch_size` × `num_gpus` × `grad_accumulation_steps`

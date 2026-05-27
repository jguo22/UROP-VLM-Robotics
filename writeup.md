# Vision Language Action Models for Human–Robot Collaboration

**Subject:** Request for advice — OpenVLA-OFT fine-tuned model is not picking up gears at deploy time

---

## 1. Project Overview

A UR5 arm in a Unity simulation, controlled by a VLA policy, performs assembly of a planetary gearbox: it picks up blue, red, and green gears and places each into its slot. The arm uses inverse kinematics with a suction-cup end-effector. The robot does the manipulation; humans may later bring additional gears within reach.

## 2. Synthetic Training Data

Demonstrations are generated entirely in simulation by an agentic AI that has privileged access to the ground-truth coordinates of each gear and the gearbox slots, and drives the arm via inverse kinematics to pick and place each gear in sequence. The resulting trajectories are recorded and converted to RLDS/TFDS for OpenVLA-OFT fine-tuning. The VLA model, in contrast, sees only the camera image and proprio state — it has to learn the visual grounding the data-collection agent gets for free.

**Scene and randomization.** Each episode is one task: pick up a blue, green, and red gear from a designated starting area on the table and place each into its slot in the gearbox. Gear positions are independently randomized per episode; the gearbox and table layout are fixed. **Training and inference run in the same scene with the same camera, randomization rules, and starting arm pose** — only the sampled gear configuration differs.

**Dataset quality (spot-check).** From human inspection of the final image of randomly sampled episodes, every sampled episode places at least one gear, and roughly 80% place all three. Failures are partial completions, not total failures. We have not measured this over the full dataset.

## 3. Training and Deployment Setup

OpenVLA-OFT, fine-tuned from a pretrained checkpoint on the synthetic UR5 dataset. At deployment the model is hosted on a remote inference server; a Unity controller sends an image plus proprio state, receives an action chunk, and applies one action per tick.

## 4. The Problem

The arm moves with coherent, plausible motion — visibly "reaching toward the gear area" — but never picks up a gear. Specifically:

- It orbits the gear cluster without converging on any one gear.
- It eventually circles back to the starting position and gets stuck.
- Every training demo successfully picks up at least one gear, so the model is not imitating bad behavior.

## 5. Debugging Done So Far

We verified the controller-to-server contract along several axes — degrees vs. radians, world- vs. local-frame coordinates, and left- vs. right-handed coordinate systems — and confirmed that the state inputs and action outputs both match the formats OpenVLA-OFT was trained on. The arm moves with sensible magnitudes and in plausible directions, but still does not converge on a gear.

## 6. Where We Are Stuck

We have suspicions but no confirmed cause. 

- The fine-tuned checkpoint may not have fully converged. We trained the model for 4000 steps with a loss of about 0.04. 
- There may be a subtle deploy-time mismatch we have not yet identified — for example, an image preprocessing difference — silently shifts the model's outputs.

We ran an **offline replay test** — feeding a recorded training episode's images and states to the server frame-by-frame and comparing the returned actions to the ground-truth actions from the CSV. The mean per-frame L2 error was about 0.1. The gripper command's open/closed direction agreed with the recording on 100% of frames, but the continuous Δpos and Δrot components diverge noticeably from the recorded values. This is consistent with the model having learned the gripper timing but not faithfully reproducing the low-level motion of the training trajectories.

## 7. Request

Any advice you can offer on getting OpenVLA-OFT to work in this setting would be very helpful — diagnostics we should run, hyperparameter or training-recipe adjustments, things in our data or setup that look off, or general guidance on fine-tuning OpenVLA-OFT for a task of this kind.                                                
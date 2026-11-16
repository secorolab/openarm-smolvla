"""Compare a saved policy against held-out recordings; no robot interfaces."""
import json
import math
import time
from datetime import datetime
from pathlib import Path

import torch
from lerobot.configs.policies import PreTrainedConfig
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.policies.factory import make_pre_post_processors
from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy


def metrics(errors):
    error = torch.cat(errors, dim=0).abs()
    return {
        "joint_mae_degrees": (error[:, :7].mean(dim=0) * 180 / math.pi).tolist(),
        "mean_joint_error_degrees": float(error[:, :7].mean() * 180 / math.pi),
        "gripper_mae_mm": float(error[:, 7].mean() * 1000),
        "joint_abs_error_p95_degrees": float(torch.quantile(error[:, :7].flatten(), 0.95) * 180 / math.pi),
    }


def main():
    torch.manual_seed(42)
    home = Path.home()
    model = home / "robot_ai/models/smolvla_cube_20k"
    root = home / "robot_ai/episodes/right_cube_30hz/exports/ccf6d66fb41b4a11a30a459426888864"
    episodes = [11, 16, 20, 23, 38, 54]
    config = PreTrainedConfig.from_pretrained(model)
    config.device = "cuda"
    config.load_vlm_weights = False
    policy = SmolVLAPolicy.from_pretrained(model, config=config, strict=True)
    preprocess, postprocess = make_pre_post_processors(policy.config, pretrained_path=str(model))
    horizon = config.chunk_size
    first_errors, chunk_errors, baseline_errors, timings = [], [], [], []
    report = {"model": str(model), "dataset": str(root), "seed": 42, "episodes": {}}
    with torch.inference_mode():
        for episode in episodes:
            dataset = LeRobotDataset(repo_id="local/openarm_right", root=str(root), episodes=[episode], video_backend="pyav")
            raw = dataset.hf_dataset.with_format(None)
            assert set(int(x) for x in raw["episode_index"]) == {episode}
            assert len(dataset) >= horizon
            indices = torch.linspace(0, len(dataset) - horizon, steps=20).round().long().unique().tolist()
            episode_first, episode_chunk, episode_baseline = [], [], []
            for index in indices:
                sample = dataset[index]
                observation = {key: sample[key].clone() for key in (
                    "observation.state", "observation.images.main", "observation.images.right")}
                observation["task"] = sample["task"]
                batch = preprocess(observation)
                policy.reset()
                torch.cuda.synchronize()
                start = time.perf_counter()
                predicted = postprocess(policy.predict_action_chunk(batch))[0].cpu()
                torch.cuda.synchronize()
                timings.append(time.perf_counter() - start)
                target = torch.tensor(raw[index:index + horizon]["action"], dtype=predicted.dtype)
                assert predicted.shape == target.shape == (horizon, 8)
                assert torch.isfinite(predicted).all() and torch.isfinite(target).all()
                error = predicted - target
                episode_first.append(error[:1])
                episode_chunk.append(error)
                episode_baseline.append(sample["observation.state"].cpu().expand_as(target) - target)
            first_errors.extend(episode_first)
            chunk_errors.extend(episode_chunk)
            baseline_errors.extend(episode_baseline)
            result = {"samples": len(indices), "first_action": metrics(episode_first),
                      "full_chunk": metrics(episode_chunk), "hold_current_pose_baseline": metrics(episode_baseline)}
            report["episodes"][str(episode)] = result
            m = result["full_chunk"]
            b = result["hold_current_pose_baseline"]
            print(f"Episode {episode}: chunk joint MAE {m['mean_joint_error_degrees']:.2f} deg; "
                  f"gripper MAE {m['gripper_mae_mm']:.2f} mm; "
                  f"hold-pose baseline {b['mean_joint_error_degrees']:.2f} deg", flush=True)
    report["overall"] = {"first_action": metrics(first_errors), "full_chunk": metrics(chunk_errors),
                         "hold_current_pose_baseline": metrics(baseline_errors),
                         "mean_prediction_seconds_after_first": sum(timings[1:]) / len(timings[1:])}
    output = Path('/tmp') / f"smolvla_validation_{datetime.now():%Y%m%d_%H%M%S_%f}.json"
    output.write_text(json.dumps(report, indent=2))
    print("\nOverall:", json.dumps(report["overall"], indent=2))
    print(f"\nReport saved: {output}")
    print("Offline comparison complete. No robot commands sent. Errors are not a task-success score.")


if __name__ == "__main__":
    main()

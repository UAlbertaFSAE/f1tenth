#!/usr/bin/env python3
"""One Vertex AI job = one model variant, looping through all 5 CV folds.

Pulls data/ from GCS once, builds a deterministic 5-fold split from
train/+valid/ (test/ stays held out, untouched), trains each fold with all
augmentation disabled, uploads each fold's run + a summary.json to GCS.

Usage (inside the training container):
  python3 train.py --model yolo26s.pt --run-name yolo_s_26 \
      --data-gcs gs://bucket/data --output-gcs gs://bucket/runs \
      --epochs 100 --imgsz 832
"""

import argparse
import csv
import json
import random
import subprocess
from pathlib import Path

import yaml
from ultralytics import YOLO

N_FOLDS = 5
SEED = 42

# All augmentation off -- these are ultralytics train() hyperparameters.
NO_AUGMENT = dict(
    hsv_h=0.0,
    hsv_s=0.0,
    hsv_v=0.0,
    degrees=0.0,
    translate=0.0,
    scale=0.0,
    shear=0.0,
    perspective=0.0,
    flipud=0.0,
    fliplr=0.0,
    bgr=0.0,
    mosaic=0.0,
    mixup=0.0,
    copy_paste=0.0,
    auto_augment=None,
    erasing=0.0,
    crop_fraction=1.0,
)

# --augment uses ultralytics' own default augmentation hyperparameters (not
# passed here at all -- omitting them lets train() fall back to its
# defaults: hsv jitter, mosaic, flips, perspective, mixup, copy_paste,
# erasing). The one thing NOT controllable via these hyperparameters is
# ultralytics' optional Albumentations block (Blur/MedianBlur/ToGray/CLAHE
# at ~1% probability each), which only activates if the `albumentations`
# package is importable -- that block includes a grayscale conversion with
# no separate on/off switch. It's uninstalled from the training image
# instead (see Dockerfile), so that block is skipped entirely and no
# grayscale augmentation can happen regardless of this flag.


def gsutil(*args: str) -> None:
    """Run gsutil with parallel transfers, raising if it fails."""
    subprocess.run(["gsutil", "-m", *args], check=True)


def download_data(data_gcs: str, local_root: Path) -> None:
    """Mirror the dataset from GCS once, skipping if it is already present."""
    if (local_root / "data.yaml").exists():
        return
    local_root.mkdir(parents=True, exist_ok=True)
    gsutil("rsync", "-r", data_gcs.rstrip("/") + "/", str(local_root) + "/")


def collect_pairs(split_dir: Path) -> list:
    """Pair every image in a split with its label file."""
    images_dir = split_dir / "images"
    labels_dir = split_dir / "labels"
    pairs = []
    for img in sorted(images_dir.iterdir()):
        if img.suffix.lower() not in (".jpg", ".jpeg", ".png"):
            continue
        label = labels_dir / (img.stem + ".txt")
        pairs.append((img, label))
    return pairs


def make_fold_manifests(pool: list, fold: int, work_dir: Path) -> tuple[Path, Path]:
    """Write the train and val image lists for one deterministic CV fold."""
    rng = random.Random(SEED)
    indices = list(range(len(pool)))
    rng.shuffle(indices)

    fold_bins = [indices[i::N_FOLDS] for i in range(N_FOLDS)]
    val_idx = set(fold_bins[fold])
    train_idx = set(indices) - val_idx

    work_dir.mkdir(parents=True, exist_ok=True)
    train_txt = work_dir / f"train_fold{fold}.txt"
    val_txt = work_dir / f"val_fold{fold}.txt"

    with train_txt.open("w") as f:
        f.writelines(f"{pool[i][0]}\n" for i in sorted(train_idx))
    with val_txt.open("w") as f:
        f.writelines(f"{pool[i][0]}\n" for i in sorted(val_idx))

    print(
        f"[fold {fold}] train={len(train_idx)} val={len(val_idx)} "
        f"(pool={len(pool)}, test/ held out untouched)"
    )
    return train_txt, val_txt


def make_data_yaml(
    base_cfg: dict, train_txt: Path, val_txt: Path, work_dir: Path
) -> Path:
    """Write the ultralytics dataset yaml for one fold."""
    fold_yaml = {
        "train": str(train_txt),
        "val": str(val_txt),
        "nc": base_cfg["nc"],
        "names": base_cfg["names"],
    }
    out_path = work_dir / f"{train_txt.stem}.yaml"
    out_path.write_text(yaml.safe_dump(fold_yaml))
    return out_path


def upload_dir(local_dir: Path, output_gcs: str, sub_path: str) -> None:
    """Upload one fold's run directory to GCS."""
    dest = f"{output_gcs.rstrip('/')}/{sub_path}/"
    gsutil("cp", "-r", str(local_dir), dest)


def fetch_prior_fold_metrics(
    output_gcs: str, run_name: str, fold: int, work_dir: Path
) -> dict | None:
    """Pull back a resumed run's already-uploaded results.csv.

    Keeps summary.json covering every fold, not just the ones this process ran.
    """
    fold_name = f"{run_name}_fold{fold}"
    remote = f"{output_gcs.rstrip('/')}/{run_name}/{fold_name}/{fold_name}/results.csv"
    local = work_dir / f"prior_{fold_name}_results.csv"
    try:
        gsutil("cp", remote, str(local))
    except subprocess.CalledProcessError:
        return None

    with local.open() as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return None
    last = {k.strip(): v.strip() for k, v in rows[-1].items()}
    return {
        "fold": fold,
        "mAP50": float(last.get("metrics/mAP50(B)", "nan")),
        "mAP50-95": float(last.get("metrics/mAP50-95(B)", "nan")),
        "precision": float(last.get("metrics/precision(B)", "nan")),
        "recall": float(last.get("metrics/recall(B)", "nan")),
    }


def main() -> None:
    """Train every CV fold for one model variant and upload the results."""
    p = argparse.ArgumentParser()
    p.add_argument("--model", required=True, help="e.g. yolo26s.pt or yolo26m.pt")
    p.add_argument("--run-name", required=True, help="e.g. yolo_s_26")
    p.add_argument("--data-gcs", required=True)
    p.add_argument("--output-gcs", required=True)
    p.add_argument("--epochs", type=int, default=100)
    p.add_argument("--imgsz", type=int, default=832)
    p.add_argument("--local-data", default="/tmp/data")
    p.add_argument("--work-dir", default="/tmp/fold_work")
    p.add_argument(
        "--start-fold", type=int, default=0, help="resume from this fold (0-indexed)"
    )
    p.add_argument(
        "--augment",
        action="store_true",
        help="use ultralytics' default augmentation instead of NO_AUGMENT",
    )
    args = p.parse_args()

    local_root = Path(args.local_data)
    work_dir = Path(args.work_dir)

    download_data(args.data_gcs, local_root)
    base_cfg = yaml.safe_load((local_root / "data.yaml").read_text())
    pool = collect_pairs(local_root / "train") + collect_pairs(local_root / "valid")
    pool.sort(key=lambda pair: pair[0].name)  # deterministic order before shuffling

    fold_metrics = []
    for fold in range(args.start_fold):
        prior = fetch_prior_fold_metrics(args.output_gcs, args.run_name, fold, work_dir)
        if prior is not None:
            fold_metrics.append(prior)
        else:
            print(
                f"[fold {fold}] WARNING: could not fetch prior metrics, "
                f"summary.json will be missing this fold"
            )

    for fold in range(args.start_fold, N_FOLDS):
        train_txt, val_txt = make_fold_manifests(pool, fold, work_dir)
        data_yaml = make_data_yaml(base_cfg, train_txt, val_txt, work_dir)

        fold_name = f"{args.run_name}_fold{fold}"
        model = YOLO(args.model)
        augment_kwargs = {} if args.augment else NO_AUGMENT
        results = model.train(
            data=str(data_yaml),
            epochs=args.epochs,
            imgsz=args.imgsz,
            seed=SEED,
            name=fold_name,
            project="/tmp/runs",
            **augment_kwargs,
        )

        metrics = {
            "fold": fold,
            "mAP50": float(results.results_dict.get("metrics/mAP50(B)", float("nan"))),
            "mAP50-95": float(
                results.results_dict.get("metrics/mAP50-95(B)", float("nan"))
            ),
            "precision": float(
                results.results_dict.get("metrics/precision(B)", float("nan"))
            ),
            "recall": float(
                results.results_dict.get("metrics/recall(B)", float("nan"))
            ),
        }
        fold_metrics.append(metrics)

        # ultralytics puts output at {project}/{name} when project is passed
        # explicitly -- the "detect/" subfolder only appears when project is
        # left at its default. Using the wrong path here is what silently
        # crashed the container after fold 0 (gsutil found nothing to upload).
        run_dir = Path("/tmp/runs") / fold_name
        upload_dir(run_dir, args.output_gcs, f"{args.run_name}/{fold_name}")

    def mean_std(key: str) -> tuple[float, float]:
        """Mean and standard deviation of one metric across folds."""
        vals = [m[key] for m in fold_metrics]
        mean = sum(vals) / len(vals)
        var = sum((v - mean) ** 2 for v in vals) / len(vals)
        return mean, var**0.5

    summary = {
        "run_name": args.run_name,
        "model": args.model,
        "n_folds": N_FOLDS,
        "folds": fold_metrics,
        "summary": {
            key: dict(zip(("mean", "std"), mean_std(key), strict=True))
            for key in ("mAP50", "mAP50-95", "precision", "recall")
        },
    }
    summary_path = work_dir / f"{args.run_name}_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2))
    gsutil(
        "cp",
        str(summary_path),
        f"{args.output_gcs.rstrip('/')}/{args.run_name}/summary.json",
    )
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()

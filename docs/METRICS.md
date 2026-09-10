# Cone Detection Model Metrics

Comparison of the cone detection models trained for `camera_detection`. Five classes: blue, yellow, orange, large_orange, unknown.

## Results

| run | model | eval | mAP50 | mAP50-95 | P | R |
| --- | --- | --- | --- | --- | --- | --- |
| **yolo_m_26** | yolo26m, 832px | 5-fold mean | **0.805 ±0.024** | **0.554 ±0.017** | 0.865 | **0.756** |
| rfdetr_small | rf-detr-s, 512px | 1 fold (fold4), EMA best | 0.803 | 0.513 | **0.934** | 0.704 |
| yolo_s_26 | yolo26s, 832px | 5-fold mean | 0.688 ±0.021 | 0.432 ±0.013 | 0.815 | 0.640 |
| old_v1 | yolo26s, 832px | single, last epoch | 0.633 | 0.385 | 0.794 | 0.599 |

`yolo_m_26` ships as `src/perception/camera_detection/models/model.pt`.

## Reading the table

**Recall was the problem, and recall is what moved.** `old_v1` missed roughly 40% of cones at 0.599 recall while holding precision at 0.794 — the signature of a model that is not hallucinating cones but failing to see them. `yolo_m_26` takes recall to 0.756, cutting misses from ~40% to ~24%. That matters more than the headline mAP50 gain because everything downstream inherits it: a missed cone on a boundary is a gap, which is a wrong midpoint, which is a wrong path.

**Capacity was worth paying for.** `yolo_s_26` and `yolo_m_26` are the same recipe at the same resolution, differing only in model size, and the jump is +0.117 mAP50 and +0.122 mAP50-95. That is a large gap for a size change alone.

**`old_v1` was superseded by its own architecture.** `yolo_s_26` is the same model and settings evaluated honestly, and it beats `old_v1` by +0.055 mAP50. Most of that difference is evaluation method rather than training: `old_v1` is a single split scored at the last epoch, so it has no cross-fold spread to trust and its numbers are the most optimistic of the four.

**`rfdetr_small` is not a fair comparison yet.** It ties on mAP50 but loses 0.041 mAP50-95, which is a box-localisation gap most likely explained by its 512px input against the YOLO runs' 832px. It has the highest precision of any run at 0.934 and the second-lowest recall at 0.704 — it is conservative, which is the wrong trade for this problem. It was also evaluated on a single fold, so it has no spread to compare against. Re-running it at 832px across all five folds is the only way to know whether the architecture or the input size is responsible.

## Evaluation protocol

The YOLO runs use a deterministic 5-fold split (seed 42) over `train/` + `valid/`, with `test/` held out and untouched, and report the mean and standard deviation across folds. `old_v1` predates this and is a single split scored at the final epoch, so its row is not directly comparable to the rows above it. `rfdetr_small` ran one fold only.

Both YOLO runs trained with all ultralytics augmentation disabled, so these numbers are a clean architecture and capacity comparison rather than an augmentation result.

## Reproducing

Training code and the winning run's config are in `src/perception/camera_detection/training/`:

```
training/train.py                      the 5-fold training driver
training/Dockerfile                    the training image
training/runs/yolo_m_26/args.yaml      the exact ultralytics arguments for the shipped model
training/runs/yolo_m_26/summary.json   per-fold and aggregate metrics
```

```bash
python3 train.py --model yolo26m.pt --run-name yolo_m_26 \
  --data-gcs gs://bucket/data --output-gcs gs://bucket/runs \
  --epochs 100 --imgsz 832
```

## Inference

The model trains at 832px, so the node runs it at 832px — `imgsz` in `camera_detection.yaml`. Leaving it at the ultralytics default of 640 changes the operating point and the numbers above no longer describe what the car is doing.

`models/classes.txt` order must match the order baked into the weights. For `yolo_m_26` that is `blue_cone, large_orange_cone, orange_cone, unknown_cone, yellow_cone` — alphabetical, and not the order the previous model used. Swapping weights without swapping this file relabels every detection.

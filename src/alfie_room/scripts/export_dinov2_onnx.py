#!/usr/bin/env python3
"""
export_dinov2_onnx — one-time export of the DINOv2 ViT-S/14 image encoder to ONNX.

The room recognizer needs a small image encoder as an ONNX file (loaded by
``alfie_room/encoder.py``). This script fetches DINOv2 ViT-S/14 via torch.hub
and exports it to ONNX with a dynamic batch axis. Run it once on the Orin (or any
machine with torch) and drop the resulting file where ``room_node``'s
``model_path`` param points (default ``~/alfiebot_ws/models/dinov2_vits14.onnx``).

    python3 export_dinov2_onnx.py --out ~/alfiebot_ws/models/dinov2_vits14.onnx

Requires: torch, and network access on first run (torch.hub downloads the
weights). onnx is optional (only used for a validity check). The model forward
returns the CLS/global embedding (shape (batch, 384)); ``encoder.Encoder`` also
handles a token-sequence output if a different DINOv2 variant is substituted.
"""
import argparse
import os


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--out", default=os.path.expanduser("~/alfiebot_ws/models/dinov2_vits14.onnx"),
        help="output ONNX path")
    ap.add_argument("--model", default="dinov2_vits14",
                    help="torch.hub DINOv2 model name (default dinov2_vits14)")
    ap.add_argument("--size", type=int, default=224,
                    help="square input size, multiple of 14 (default 224)")
    ap.add_argument("--opset", type=int, default=17, help="ONNX opset (default 17)")
    args = ap.parse_args()

    import torch

    print(f"Loading {args.model} from torch.hub (facebookresearch/dinov2) ...")
    model = torch.hub.load("facebookresearch/dinov2", args.model)
    model.eval()

    os.makedirs(os.path.dirname(os.path.abspath(args.out)) or ".", exist_ok=True)
    dummy = torch.randn(1, 3, args.size, args.size)

    print(f"Exporting to {args.out} (opset {args.opset}) ...")
    torch.onnx.export(
        model, dummy, args.out,
        input_names=["image"], output_names=["embedding"],
        dynamic_axes={"image": {0: "batch"}, "embedding": {0: "batch"}},
        opset_version=args.opset,
    )

    try:
        import onnx
        onnx.checker.check_model(args.out)
        print("ONNX model check: OK")
    except ImportError:
        print("(install 'onnx' to validate the exported model)")

    with torch.no_grad():
        out = model(dummy)
    print(f"Sanity forward output shape: {tuple(out.shape)}")
    print(f"Done -> {args.out}")


if __name__ == "__main__":
    main()

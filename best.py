# -*- coding: utf-8 -*-
"""
best.py —— 把原模型 best.pt 导出成 OpenCV DNN 能直接加载的 ONNX 模型。

为什么会有这个脚本？
    之前生成的 robomaster.onnx 是用新版 torch onnxscript(dynamo) 导出器生成的，
    它有一堆 OpenCV 4.10 DNN 不兼容的毛病：
        1) 所有 Conv 节点缺 kernel_shape 属性；
        2) 输入走动态 shape；
        3) 整体图结构让 OpenCV 在构造时直接抛
           “Input shape redefinition is not allowed / images” 错误。
    因此改用 Ultralytics 自带的导出器重新导出，并用较小的 opset + 固定输入尺寸，
    这样导出的模型 OpenCV 能顺利加载。

注意事项：
    - 需要能在本机 import ultralytics 以及 torch（本仓库即 ultralytics 源码）。
    - 导出的默认参数对 OpenCV 友好：opset=12、imgsz=640、simplify=True、不动态、不加 NMS。
      其中 “不加 NMS” 是必须的：因为 C++ 端（yolo_detector.cpp）自己用 cv::dnn::NMSBoxes
      做后处理，并期望拿到 6 个原始输出张量（每个尺度 bbox + cls）。
    - 运行后会打印导出文件的完整路径，把它复制/覆盖到工程根目录的 robomaster.onnx 即可。
"""

import argparse
import os
import sys

# 尽量引入 ultralytics（本仓库根目录）与 torch
try:
    from ultralytics import YOLO
except ImportError as e:  # 若 ultralytics 没装到 sys.path，给出可读的错误提示
    # 尝试把本仓库根目录加入 sys.path，方便直接从源码目录运行
    repo_root = os.path.dirname(os.path.abspath(__file__))
    if repo_root not in sys.path:
        sys.path.insert(0, repo_root)
    try:
        from ultralytics import YOLO
    except ImportError:
        raise SystemExit(
            "无法导入 ultralytics。请先安装依赖:\n"
            "    pip install ultralytics\n"
            "或在已装好 torch + ultralytics 的虚拟环境中运行本脚本。"
        ) from e


def main():
    # 默认模型路径：你的原模型 best.pt；可通过命令行参数传入覆盖。
    default_pt = os.path.join("/run/media/xiaopi/新加卷/example1/best.pt")

    parser = argparse.ArgumentParser(
        description="把 best.pt 导出为 OpenCV DNN 兼容的 ONNX(.onnx) 模型。"
    )
    parser.add_argument(
        "model",
        nargs="?",
        default=default_pt,
        help=f"输入的 .pt 权重路径（默认: {default_pt}）",
    )
    parser.add_argument(
        "--opset",
        type=int,
        default=12,
        help="ONNX opset 版本，OpenCV 4.10 推荐 12（范围 11~17）。默认 12。",
    )
    parser.add_argument(
        "--imgsz",
        type=int,
        default=640,
        help="导出时网络输入尺寸（固定边长）。默认 640，须与 C++ 端 input_size_ 一致。",
    )
    parser.add_argument(
        "--out",
        default=None,
        help="输出文件路径；默认输出到模型同目录下 best.onnx。",
    )
    args = parser.parse_args()

    if not os.path.isfile(args.model):
        raise SystemExit(f"找不到模型文件：{args.model}")

    # 输出路径默认取模型所在目录，文件名为 best.onnx
    out_path = args.out or os.path.join(os.path.dirname(os.path.abspath(args.model)), "best.onnx")

    print(f"加载模型：{args.model}")
    model = YOLO(args.model)  # 自动识别权重的类别数与结构，不用手动指定类别

    print("开始导出 ONNX（opset=%d, imgsz=%d, simplify=True, nms=False, dynamic=False）..."
          % (args.opset, args.imgsz))
    # 导出参数说明（针对 OpenCV DNN 兼容性）：
    #   format="onnx"   -> 输出 ONNX 格式
    #   opset=12        -> 较低 opset，OpenCV 覆盖率最好
    #   imgsz=640       -> 固定输入尺寸，避免动态 shape 触发 OpenCV 的 images 重定义 bug
    #   simplify=True   -> 用 onnx-simplifier 化简计算图
    #   dynamic=False   -> 关闭动态输入/输出尺寸
    #   nms=False       -> 不在图上附加 NMS（C++ 端自己做 NMS，且期望 6 个原始输出张量）
    #   batch=1         -> 导出批大小为 1
    exported = model.export(
        format="onnx",
        opset=args.opset,
        imgsz=args.imgsz,
        simplify=True,
        dynamic=False,
        nms=False,
        batch=1,
    )

    print(f"导出成功，生成文件：{exported}")

    # 若生成的路径跟 out_path 不一致，复制一份到约定位置
    if os.path.abspath(exported) != os.path.abspath(out_path):
        import shutil
        shutil.copyfile(exported, out_path)
        print(f"已复制到：{out_path}")

    print("\n下一步：把该 .onnx 复制/覆盖到 rm_demo 工程根目录的 robomaster.onnx 后运行：")
    print("    unset GTK_PATH && ./build/rm_demo --yolo 0")


if __name__ == "__main__":
    main()

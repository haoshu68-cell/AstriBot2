此轮不计入有效基线：LD_LIBRARY_PATH 的当前开发库覆盖了基线可执行文件的 DT_RUNPATH。后续 exact_baseline/ 已显式优先加载基线构建目录，并复现 12 个原始失败。

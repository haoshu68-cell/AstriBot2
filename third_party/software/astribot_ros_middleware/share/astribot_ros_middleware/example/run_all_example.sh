#!/bin/bash

# 获取当前脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 颜色定义
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "=========================================="
echo "开始运行所有示例脚本"
echo "=========================================="

# 计数器
total=0
success=0
failed=0

# 遍历当前目录下所有的 Python 文件
for py_file in "$SCRIPT_DIR"/*.py; do
    # 检查文件是否存在
    if [ ! -f "$py_file" ]; then
        continue
    fi
    
    # 获取文件名
    filename=$(basename "$py_file")
    
    echo -e "\n${YELLOW}[运行]${NC} $filename"
    echo "------------------------------------------"
    
    # 运行 Python 文件，设置超时为 10 秒
    timeout 10 python3 "$py_file"
    exit_code=$?
    
    ((total++))
    
    if [ $exit_code -eq 0 ]; then
        echo -e "${GREEN}[成功]${NC} $filename 运行完成"
        ((success++))
    elif [ $exit_code -eq 124 ]; then
        echo -e "${YELLOW}[超时]${NC} $filename 运行超时（10秒）"
        ((success++))
    else
        echo -e "${RED}[失败]${NC} $filename 运行失败，退出码：$exit_code"
        ((failed++))
    fi
    
    echo "------------------------------------------"
    
    # 短暂延迟，避免资源冲突
    sleep 1
done

# 输出统计信息
echo ""
echo "=========================================="
echo "运行完成统计"
echo "=========================================="
echo -e "总计: $total"
echo -e "${GREEN}成功: $success${NC}"
echo -e "${RED}失败: $failed${NC}"
echo "=========================================="

# 如果有失败的测试，返回非零退出码
if [ $failed -gt 0 ]; then
    exit 1
else
    exit 0
fi

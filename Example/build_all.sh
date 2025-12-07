#!/bin/bash

# 定义一个函数来执行递归查找和脚本执行操作
find_and_execute() {
    local search_path="$1"
    # 使用 find 命令递归查找所有名为 user 的目录
    find "$search_path" -type d -name "user" | while read -r user_dir; do
        build_script="$user_dir/build.sh"
        # 检查 build.sh 文件是否存在且可执行
        if [ -x "$build_script" ]; then
            echo "正在 $user_dir 目录执行 build.sh 文件..."
            (cd "$user_dir" && ./build.sh)
            local exit_status=$?
            if [ $exit_status -eq 0 ]; then
                echo "在 $user_dir 目录 build.sh 执行成功。"
            else
                echo "在 $user_dir 目录 build.sh 执行失败，退出状态码: $exit_status。"
            fi
        elif [ -f "$build_script" ]; then
            echo "在 $user_dir 目录的 build.sh 文件不可执行，请赋予执行权限。"
        else
            echo "在 $user_dir 目录未找到 build.sh 文件。"
        fi
    done
}

# 从当前目录开始查找
start_directory="."
find_and_execute "$start_directory"
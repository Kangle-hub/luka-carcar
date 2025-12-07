#!/bin/bash

# 定义一个函数用于查找并删除 out 目录内容
delete_out_dir_content() {
    local search_dir=$1
    # 使用 find 命令查找所有名为 out 的目录
    find "$search_dir" -type d -name "out" | while read -r out_dir; do
        # 检查目录是否存在
        if [ -d "$out_dir" ]; then
            # 删除目录下的所有内容（包括子目录及其内容）
            # rm -rf "$out_dir"/*
            find "$out_dir" -mindepth 1 ! -name "本文件夹作用.txt" -exec rm -rf {} +
            echo "已删除 $out_dir 目录下的所有内容。"
        fi
    done
}

# 从当前目录开始查找
start_dir="."
delete_out_dir_content "$start_dir"
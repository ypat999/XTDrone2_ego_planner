#!/bin/bash

WEB_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "切换到目录: $WEB_DIR"
cd "$WEB_DIR"
echo "启动Web服务器，端口: 8084"
python3 -m http.server 8084

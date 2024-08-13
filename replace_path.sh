#!/bin/bash

# 查找并替换所有文件中的sunny为xiling
find build/ -type f -exec sed -i 's/sunny/xiling/g' {} +

echo "替换完成！"


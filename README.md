# bitbot_cifx_demo

## 将本地工作树强制重置为远程分支状态（不可逆）

以下命令会无条件把当前本地分支重置为对应的远程分支状态，并删除所有未跟踪和被忽略的文件。此操作不可逆，谨慎使用。

```bash
# 在仓库根目录运行：
BRANCH=$(git rev-parse --abbrev-ref HEAD) && \
git fetch --all --prune && \
git reset --hard origin/$BRANCH && \
git clean -fdx
```

说明：

- `git reset --hard origin/$BRANCH` 会重置 HEAD、索引和工作树到远程分支对应的提交。
- `git clean -fdx` 会删除所有未跟踪文件和目录，以及被忽略的文件（例如构建输出）。

仅在你确定要放弃本地所有未提交更改并同步远程时使用。

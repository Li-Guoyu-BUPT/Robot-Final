# 实验室建图数据集

这是实验室场景的图片建图数据集元数据与质量记录。数据按拍摄批次组织在项目原目录：

- `C:\Users\ASUS\Desktop\机器人大作业\建图\建图photos`
- `C:\Users\ASUS\Desktop\机器人大作业\建图\第二批`
- `C:\Users\ASUS\Desktop\机器人大作业\建图\第三批`

三批照片联合处理后共 172 张输入图片，其中 167 张通过退化视角过滤；当前稠密融合结果约 912,972 个点。详细文件清单在 `manifest.json`，质量指标在 `quality.json`，处理过程在 `优化记录.txt`。

原始图片、手机视频和点云结果未复制到 Git 发布目录，以免把仓库变成多 GB 数据仓库。若需要发布原始数据，建议单独使用 Git LFS 或 GitHub Release 附件，并保留此处的 manifest 与校验信息。

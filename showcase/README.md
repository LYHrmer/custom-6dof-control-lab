# 重力补偿实物展示页

原生 HTML/CSS/JS，无构建过程，无后端控制入口。使用项目作者提供的原始录像，支持播放、重播、0.5×慢放、全屏，以及2/7/12/17秒关键帧跳转。页面没有虚构关节数据、无补偿对照或实物精度指标。

先准备本地素材（本机系统Python已提供OpenCV；不需要修改仿真`.venv`）：

```bash
rtk proxy python3 /home/lyh/custom-6dof-control-lab/tools/prepare_showcase_media.py
```

启动本地页面：

```bash
rtk proxy python3 -m http.server 8766 --bind 127.0.0.1 --directory /home/lyh/custom-6dof-control-lab/showcase
```

浏览器访问 `http://127.0.0.1:8766`。也可以直接打开 `index.html`；推荐HTTP方式。已准备素材时，`showcase/`是可独立复制的静态页面目录。

`assets/media/`中的视频与关键帧仅保存在本地并被Git忽略，符合项目的原视频保存约定。GitHub只包含页面源码与准备工具，新克隆需先恢复参考视频再生成素材。页面是本地预览，不自动发布网站或上传视频。

桌面和手机浏览器均保留原生视频控制；不自动播放音频，支持键盘操作与减少动效偏好。来源与验证范围放在页面下方的说明中。

本机 Chromium 已检查桌面 1440×1040 与手机 390×844 布局、视频解码与播放、半速、12秒跳转、重播、来源说明展开、图片载入和键盘焦点；没有横向溢出、页面异常或 HTTP 资源错误。检查结果及截图位于本地 `reports/showcase-check/`，截图含视频画面，亦不上传 GitHub。尚未在真实手机或其他浏览器上验证。

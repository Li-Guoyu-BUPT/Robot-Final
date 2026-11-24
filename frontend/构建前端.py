# -*- coding: utf-8 -*-
from pathlib import Path

root=Path(__file__).resolve().parent
original=(root/'源文件/manual-page.html').read_text(encoding='utf-8-sig')
manual=original.split('<body>',1)[1].split('<script>window.BOOT=',1)[0]
manual=manual.replace('ROBAN 实时建图','人工建图').replace('收到 RGB-D 后自动开始，机器人停下时也持续建图。','动作结束至少4秒后，稳定且配准通过时融合一帧。')
portal='''
<header class="platform-header">
 <a class="platform-brand" href="#home"><span class="brand-mark" aria-hidden="true"></span><span>ROBAN<span class="brand-subtitle">空间感知实验平台</span></span></a>
 <nav class="platform-nav" aria-label="主导航"><a href="#home">工作台</a><a href="#manual">人工建图</a><a href="#gallery">图库</a></nav>
 <span id="platform-connection" class="connection-label">本机实验平台</span>
</header>
<div id="platform-error" class="platform-error" role="status" hidden></div>
<main id="home-page" class="portal-page">
 <div class="page-intro"><div><h1>工作台</h1><p>选择采集方式，管理地图，逐步完成机器人定位与补扫。</p></div><span class="subtle-label">电脑端计算 · 机器人端感知</span></div>
 <div class="home-layout">
  <section class="workflows" aria-label="功能入口">
   <a class="workflow-link" href="#manual"><div><h2>人工建图</h2><span class="availability">可使用</span><p>现场单步操控机器人，同时查看相机与三维地图。</p><ul><li>键盘或鼠标操作</li><li>动作后4秒采一帧</li><li>地图存档与优化</li></ul></div><span class="entry-arrow" aria-hidden="true">→</span></a>
   <a class="workflow-link" href="#gallery"><div><h2>图库</h2><span class="availability">地图管理</span><p>浏览之前的建图结果，选择后续定位与补扫使用的地图。</p><ul><li>查看真实存档</li><li>选择任务地图</li><li>手机视频入口</li></ul></div><span class="entry-arrow" aria-hidden="true">→</span></a>
   <div class="future-work"><h3>接下来的任务</h3><p>手机视频离线重建、地图内定位与自主补扫正在规划，当前入口不触发机器人运动。</p></div>
  </section>
  <aside class="home-observation" aria-label="现场状态"><div class="observation-title"><h2>现场相机</h2><span>实时观测</span></div><div class="home-camera-view"><img id="home-camera" src="/api/camera.jpg" alt="机器人当前相机画面" hidden><p id="home-camera-empty">等待机器人相机</p></div><p id="camera-caption" class="observation-caption">读取连接状态…</p><p id="device-summary">正在获取设备信息。</p><a class="text-link" href="#manual">进入人工建图查看完整状态 →</a></aside>
 </div>
 <section class="recent-section"><div class="section-title"><h2>最近保存</h2><a class="text-link" href="#gallery">查看图库 →</a></div><ul id="recent-maps"></ul><p id="recent-empty" class="muted">地图列表加载中…</p></section>
</main>
<main id="gallery-page" class="portal-page" hidden>
 <div class="page-intro"><div><h1>图库</h1><p>浏览建图存档，选择机器人后续任务使用的地图。</p></div><button id="open-import" class="primary" aria-expanded="false" aria-controls="video-import">手机视频建图</button></div>
 <section id="video-import" class="video-import" hidden aria-label="手机视频入口"><div><h2>选择手机拍摄的视频</h2><p>支持本地MP4预览。视频上传、离线重建与尺度校准将在后续接入。</p><label class="file-label" for="video-input">选择MP4视频</label><input type="file" id="video-input" accept=".mp4,video/mp4"><p id="video-info" role="status">选择MP4视频，在此预览。视频尚不会上传或重建。</p><button disabled>离线重建待接入</button></div><video id="video-preview" controls preload="metadata" hidden aria-label="所选视频预览"></video></section>
 <div class="library-toolbar"><label for="map-search">查找地图<input id="map-search" type="search" placeholder="搜索日期或存档名称"></label><label for="map-filter">地图类型<select id="map-filter"><option value="all">全部地图</option><option value="manual">人工建图</option><option value="optimized">优化地图</option></select></label><button id="refresh-library">刷新地图</button><button id="clear-filter" class="quiet">清除筛选</button></div>
 <div class="library-layout"><section class="map-inventory" aria-label="已保存地图"><div class="section-title"><h2>已保存地图</h2><span id="library-count">读取存档中…</span></div><div class="table-scroll"><table><thead><tr><th>地图</th><th>保存时间</th><th>点数</th><th>类型</th></tr></thead><tbody id="map-table-body"></tbody></table></div><p id="library-empty" class="library-empty" hidden></p></section>
 <aside class="map-detail" aria-label="地图详情"><h2>地图详情</h2><div id="map-detail-empty" class="detail-empty"><span class="detail-symbol" aria-hidden="true"><svg viewBox="0 0 48 48" fill="none" stroke="currentColor" stroke-width="1.5"><path d="m6 12 12-5 12 5 12-5v29l-12 5-12-5-12 5zM18 7v29m12-24v29"/></svg></span><h3>先选择一张地图</h3><p>点击列表中的地图，查看存档信息并选作任务地图。</p></div><div id="map-detail-content" hidden><h3 id="detail-title"></h3><dl><dt>保存时间</dt><dd id="detail-time"></dd><dt>地图来源</dt><dd id="detail-source"></dd><dt>地图规模</dt><dd id="detail-points"></dd></dl><p id="detail-note" class="detail-notice"></p><a id="detail-preview" class="text-link" target="_blank" rel="noopener" hidden>打开三维预览 →</a><p id="selection-status" class="selected-status" role="status"></p><ol class="task-stages"><li><strong>选择地图</strong><span>已支持人工选择</span></li><li><strong>机器人定位</strong><span>待接入跨设备对应与尺度校准</span></li><li><strong>质量评估与自主补扫</strong><span>待接入目标选择和路径执行</span></li></ol><button class="primary" disabled>定位与补扫待接入</button><details><summary>查看存档标识</summary><p id="detail-id"></p></details></div></aside></div>
</main>
'''
travel_controls='''<fieldset class="travel-mode"><legend>如何前往补扫区域</legend>
<label><input type="radio" name="rescan-travel-mode" id="travel-manual" value="manual" checked><span><strong>我来操控</strong><small>键盘或鼠标单步前往</small></span></label>
<label><input type="radio" name="rescan-travel-mode" id="travel-auto" value="auto"><span><strong>机器人自主前往</strong><small>根据地图与实时观测执行 · 待接入</small></span></label>
</fieldset><p id="travel-mode-help" class="detail-notice" role="status"></p>
<div id="auto-prerequisites" class="auto-prerequisites" hidden><h4>自主执行前需完成</h4><ul><li>所选地图内定位与尺度对齐</li><li>补扫目标和可通行路径确定</li><li>实时障碍检测、暂停与人工接管</li></ul></div>
<button id="travel-action" class="primary">进入人工操控</button><p class="travel-footnote">切换模式仅保存选择，不会启动机器人运动。</p>'''
portal=portal.replace('<button class="primary" disabled>定位与补扫待接入</button>',travel_controls)
manual='<section id="rescan-context" class="rescan-context" hidden><p id="rescan-context-text"></p><a href="#gallery">返回图库切换模式</a></section>'+manual
html='<!doctype html><html lang="zh-CN"><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>ROBAN 空间感知实验平台</title><link rel="stylesheet" href="/style.css"></head><body>'+portal+'<div id="manual-page" hidden>'+manual+'</div><script>window.BOOT=__BOOT__;</script><script src="/app.js"></script></body></html>'
(root/'index.html').write_text(html,encoding='utf-8')
manual_js=(root/'源文件/manual-console.js').read_text(encoding='utf-8-sig')
manual_js=manual_js.replace('focused:document.hasFocus()&&!document.hidden','focused:location.hash==="#manual"&&document.hasFocus()&&!document.hidden')
manual_js=manual_js.replace('async function action(op,key){','async function action(op,key){\n  if(location.hash!=="#manual"&&op!=="stop")return;')
manual_js=manual_js.replace('window.addEventListener("keydown",e=>{','window.addEventListener("keydown",e=>{\n  if(location.hash!=="#manual")return;')
manual_js+='\nwindow.manualConsoleLeave=()=>{pressed.clear();requireRelease=true;heartbeat()};\n'
bundle='function startManualConsole(){\n'+manual_js+'\n}\n'+(root/'源文件/portal.js').read_text(encoding='utf-8-sig')
(root/'app.js').write_text(bundle,encoding='utf-8')
print('Built index.html and app.js')

